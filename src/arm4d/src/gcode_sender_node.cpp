/**
 *	@file gcode_sender_node.cpp
 *	@brief ROS2 node that streams G-code over UART with a sliding window.
 *	@details
 *		- Services:
 *			- IN (string): arm4d_interfaces::srv::SendLine  -> "gcode_in_string"
 *			- IN (gcode):  arm4d_interfaces::srv::Gcode     -> "gcode_in"
 *		- Action:
 *			- Homing: arm4d_interfaces::action::Homing      -> "home_robot"
 *		- Serial:
 *			- Streaming TX loop uses a sliding window (MAX_INFLIGHT) so the controller
 *			  planner can blend successive moves (smooth motion).
 *			- Homing uses sendAndWait() and the same serial mutex to avoid interleaving.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <arm4d_interfaces/action/homing.hpp>
#include <arm4d_interfaces/srv/send_line.hpp>
#include <arm4d_interfaces/srv/gcode.hpp>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <regex>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

class GCodeSender : public rclcpp::Node {
public:
	using Homing = arm4d_interfaces::action::Homing;
	using SendLine = arm4d_interfaces::srv::SendLine;
	using Gcode = arm4d_interfaces::srv::Gcode;
	using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;

	/**
	 *	@brief Ctor: opens serial, creates services/action, starts RX/TX threads.
	 */
	GCodeSender()
	: Node("gcode_sender"),
	  stop_(false),
	  last_state_("Unknown")
	{
		declare_parameter<std::string>("port", "/dev/ttyUSB0");
		declare_parameter<int>("baud", 921600);
		declare_parameter<int>("max_inflight", 8);

		std::string port = get_parameter("port").as_string();
		int baud = get_parameter("baud").as_int();
		int max_inflight = get_parameter("max_inflight").as_int();
		MAX_INFLIGHT_ = (max_inflight > 0) ? static_cast<size_t>(max_inflight) : 8;

		fd_ = openSerial(port, baud);
		if (fd_ < 0) {
			RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port.c_str());
			throw std::runtime_error("serial open failed");
		}

		state_pub_ = create_publisher<std_msgs::msg::String>("controller_state", 10);

		send_gcode_string_ = create_service<SendLine>(
			"gcode_in_string",
			std::bind(&GCodeSender::cbGCode_string, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
		);

		send_gcode_ = create_service<Gcode>(
			"gcode_in",
			std::bind(&GCodeSender::cbGCode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
		);

		action_server_ = rclcpp_action::create_server<Homing>(
			this,
			"home_robot",
			std::bind(&GCodeSender::goalCB, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&GCodeSender::cancelCB, this, std::placeholders::_1),
			std::bind(&GCodeSender::execCB, this, std::placeholders::_1)
		);

		rx_thread_ = std::thread(&GCodeSender::rxLoop, this);
		tx_thread_ = std::thread(&GCodeSender::txLoop_streaming, this);
	}

	/**
	 *	@brief Dtor: stops threads and closes serial.
	 */
	~GCodeSender() override {
        stop_ = true;
		q_cv_.notify_all();
		inflight_cv_.notify_all();
		if (rx_thread_.joinable()) rx_thread_.join();
		if (tx_thread_.joinable()) tx_thread_.join();
		if (fd_ >= 0) close(fd_);
	}

private:
	// ---------------------- Serial state ----------------------
	int fd_;
	std::atomic<bool> stop_;
	std::string last_state_;

	// ---------------------- Threads and queues ----------------------
	std::thread rx_thread_, tx_thread_;
	std::mutex q_mtx_;
	std::condition_variable q_cv_;
	std::queue<std::string> workq_;			///<	@brief lines enqueued by services

	// ---------------------- ROS interfaces ----------------------
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
	rclcpp::Service<Gcode>::SharedPtr send_gcode_;
	rclcpp::Service<SendLine>::SharedPtr send_gcode_string_;
	rclcpp_action::Server<Homing>::SharedPtr action_server_;

	// ---------------------- Acks & serialization ----------------------
	std::mutex				serial_mtx_;		///<	@brief serialize all writes (streaming + homing)
	std::mutex				ack_mtx_;			///<	@brief protects ack_line_/ack_ready_
	std::condition_variable	ack_cv_;
	std::string				ack_line_;
	std::atomic<bool>		ack_ready_{false};

	// ---------------------- Inflight tracking (streaming window) ----------------------
	std::mutex inflight_mtx_;
	std::condition_variable inflight_cv_;
	size_t inflight_ = 0;						///<	@brief outstanding cmds not yet acked
	size_t MAX_INFLIGHT_ = 8;					///<	@brief configurable via param "max_inflight"

	// ---------------------- Serial helpers ----------------------
	/**
	 *	@brief Open and configure POSIX serial port.
	 */
	int openSerial(const std::string &port, int baud){
		int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0) return -1;
		RCLCPP_INFO(get_logger(), "Opened serial %s @ %d (max_inflight=%zu)", port.c_str(), baud, MAX_INFLIGHT_);

		struct termios tty{};
		if (tcgetattr(fd, &tty) != 0) return -1;

		cfsetospeed(&tty, baudToConst(baud));
		cfsetispeed(&tty, baudToConst(baud));

		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
		tty.c_iflag &= ~IGNBRK;
		tty.c_lflag = 0;
		tty.c_oflag = 0;
		tty.c_cc[VMIN]  = 0;
		tty.c_cc[VTIME] = 5;
		tty.c_cflag |= (CLOCAL | CREAD);
		tty.c_cflag &= ~(PARENB | PARODD);
		tty.c_cflag &= ~CSTOPB;
		tty.c_cflag &= ~CRTSCTS;

		if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
		return fd;
	}

	/**
	 *	@brief Map baud integer to termios speed_t.
	 */
	static speed_t baudToConst(int baud){
		switch (baud){
			case 9600: return B9600;
			case 19200: return B19200;
			case 38400: return B38400;
			case 57600: return B57600;
			case 115200: return B115200;
			case 230400: return B230400;
			case 460800: return B460800;
			case 921600: return B921600;
			default: return B115200;
		}
	}

	/**
	 *	@brief Write a single G-code line with trailing LF.
	 *	@details Uses serial_mtx_ so streaming and homing never interleave bytes.
	 */
	void serialWriteLine(const std::string &line){
		std::lock_guard<std::mutex> ser_lk(serial_mtx_);
		std::string s = line;
		if (!s.empty() && s.back() != '\n') s.push_back('\n');
		(void)write(fd_, s.c_str(), s.size());
		RCLCPP_INFO(get_logger(), ">> %s", line.c_str());
	}

	// ---------------------- RX loop ----------------------
	/**
	 *	@brief RX reader: builds lines and dispatches.
	 */
	void rxLoop(){
		char buf[256];
		std::string line;
		while (!stop_){
			int n = read(fd_, buf, sizeof(buf));
			if (n <= 0){ std::this_thread::sleep_for(1ms); continue; }
			for (int i = 0; i < n; i++){
				if (buf[i] == '\n'){
					handleLine(line);
					line.clear();
				} else if (buf[i] != '\r'){
					line.push_back(buf[i]);
				}
			}
		}
	}

	/**
	 *	@brief RX: classify and notify only for meaningful acks (ok/error/ALARM). Publish state.
	 */
	void handleLine(const std::string &line){
		RCLCPP_INFO(get_logger(), "<< %s", line.c_str());

		// controller status -> publish
		if (line.rfind("<", 0) == 0){
			auto bar = line.find("|");
			if (bar != std::string::npos)			last_state_ = line.substr(1, bar - 1);
			else if (!line.empty() && line.back() == '>')	last_state_ = line.substr(1, line.size() - 2);
			auto msg = std_msgs::msg::String();
			msg.data = last_state_;
			state_pub_->publish(msg);
		}

		// ack / error / alarm lines wake a waiter and free an inflight slot
		const bool is_ok    = (line.find("ok") != std::string::npos);
		const bool is_err   = (line.find("error") != std::string::npos);
		const bool is_alarm = (line.find("ALARM") != std::string::npos) || (line.find("Alarm") != std::string::npos);

		if (is_ok || is_err || is_alarm){
			{
				std::lock_guard<std::mutex> lk(ack_mtx_);
				ack_line_ = line;
				ack_ready_ = true;
			}
			ack_cv_.notify_all();

			{
				std::lock_guard<std::mutex> lk2(inflight_mtx_);
				if (inflight_ > 0) inflight_--;
			}
			inflight_cv_.notify_all();
		}
	}

	// ---------------------- Blocking send (homing only) ----------------------
	/**
	 *	@brief Send one line and wait for ok/error/ALARM (homing/critical sequences).
	 *	@note Uses same serial mutex as streaming, so bytes never interleave.
	 */
	bool sendAndWait(const std::string &cmd, std::string &resp, int timeout_ms = 5000){
		std::unique_lock<std::mutex> ser_lk(serial_mtx_);	// serialize writers

		// drop stale ack
		{
			std::lock_guard<std::mutex> lk(ack_mtx_);
			ack_ready_ = false;
			ack_line_.clear();
		}

		// NOTE: do not flush here; it can drop important status lines
		std::string s = cmd;
		if (!s.empty() && s.back() != '\n') s.push_back('\n');
		(void)write(fd_, s.c_str(), s.size());
		RCLCPP_INFO(get_logger(), ">> %s", cmd.c_str());

		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
		for (;;){
			std::unique_lock<std::mutex> lk(ack_mtx_);
			if (!ack_cv_.wait_until(lk, deadline, [&]{ return ack_ready_.load(); })){
				RCLCPP_ERROR(get_logger(), "Timeout waiting for ack of %s", cmd.c_str());
				return false;
			}
			resp = ack_line_;
			ack_ready_.store(false);

			const bool is_ok    = (resp.find("ok") != std::string::npos);
			const bool is_err   = (resp.find("error") != std::string::npos);
			const bool is_alarm = (resp.find("ALARM") != std::string::npos) || (resp.find("Alarm") != std::string::npos);

			if (is_ok)				return true;
			if (is_err || is_alarm)	return false;

			// non-ack lines (e.g., status) -> keep waiting
		}
	}

	// ---------------------- TX loop (STREAMING MODE) ----------------------
	/**
	 *	@brief Streaming TX loop: maintains a sliding window of un-acked lines.
	 *	@details
	 *		- If inflight < MAX_INFLIGHT_: send next queued line immediately.
	 *		- Else block until an ack frees a slot.
	 *		- Uses serial_mtx_ for each write, so homing (sendAndWait) can safely preempt.
	 */
	void txLoop_streaming(){
		while (!stop_){
			std::string cmd;

			// wait for a queued line
			{
				std::unique_lock<std::mutex> lk(q_mtx_);
				q_cv_.wait_for(lk, 100ms, [&]{ return stop_ || !workq_.empty(); });
				if (stop_) break;
				if (workq_.empty()) continue;
				cmd = workq_.front();
				workq_.pop();
			}

			// wait until there is room in the sliding window
			{
				std::unique_lock<std::mutex> lk(inflight_mtx_);
				inflight_cv_.wait(lk, [&]{ return stop_ || inflight_ < MAX_INFLIGHT_; });
				if (stop_) break;
				++inflight_;
			}

			// send one line (non-blocking wrt ack)
			serialWriteLine(cmd);
		}
	}

	// ---------------------- Services ----------------------
	/**
	 *	@brief Enqueue a raw G-code string.
	 */
	bool addToQueue(const std::string& str){
		const size_t MAX_QUEUE = 10;
		{
			std::unique_lock<std::mutex> lk(q_mtx_);
			if (workq_.size() >= MAX_QUEUE) {
				RCLCPP_WARN(get_logger(), "Queue full, rejecting line: %s", str.c_str());
				return false;
			}
			workq_.push(str);
		}
		q_cv_.notify_all();
		return true;
	}

	/**
	 *	@brief Service: enqueue one raw line.
	 */
	void cbGCode_string(
		const std::shared_ptr<rmw_request_id_t> /*request_header*/,
		const std::shared_ptr<SendLine::Request> request,
		const std::shared_ptr<SendLine::Response> response
	){
		bool res = addToQueue(request->line);
		response->message = res ? "Line accepted" : "Queue full, try again later";
		response->success = res;
	}

	/**
	 *	@brief Service: convert structured Gcode request to line and enqueue.
	 */
	void cbGCode(
		const std::shared_ptr<rmw_request_id_t> /*request_header*/,
		const std::shared_ptr<Gcode::Request> request,
		const std::shared_ptr<Gcode::Response> response
	){
		std::stringstream str;
		str << std::fixed << std::setprecision(2);
		switch (request->type){
			case 0:
				str << "G0"
				    << "X" << request->l0
				    << "Y" << request->l1
				    << "Z" << request->l2
				    << "A" << request->l3;
				break;
			case 1:
				str << "G1"
				    << "X" << request->l0
				    << "Y" << request->l1
				    << "Z" << request->l2
				    << "A" << request->l3
				    << "F" << request->feed;
				break;
			case 2:
			case 3:
			default:
				// extend as needed
				break;
		}
		response->success = addToQueue(str.str());
	}

	// ---------------------- Action: Homing ----------------------
	/**
	 *	@brief Accept all homing goals.
	 */
	rclcpp_action::GoalResponse goalCB(const rclcpp_action::GoalUUID&, std::shared_ptr<const Homing::Goal>){
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 *	@brief Accept homing cancellations.
	 */
	rclcpp_action::CancelResponse cancelCB(const std::shared_ptr<GoalHandleHoming>){
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	/**
	 *	@brief Execute homing as a synchronous sequence (uses sendAndWait()).
	 */
	std::shared_ptr<GoalHandleHoming> execCB(const std::shared_ptr<GoalHandleHoming> goal_handle){
		auto feedback = std::make_shared<Homing::Feedback>();

		// optional: clear input noise before a blocking sequence
		tcflush(fd_, TCIFLUSH);

		std::vector<std::string> steps = {
			"$HZ",
			"G0 Z20",
			"$HA",
			"$HY",
			"G0 Y45",
			"$HZ",
			"$HX",
			"G0 X45 Y45 Z-45 A45"
		};

		for (const auto &cmd : steps){
			if (goal_handle->is_canceling()){
				auto result = std::make_shared<Homing::Result>();
				result->success = false;
				result->message = "Canceled by client";
				goal_handle->canceled(result);
				return goal_handle;
			}

			std::string resp;
			if (!sendAndWait(cmd, resp, 15000)){	// longer timeout during homing
				auto result = std::make_shared<Homing::Result>();
				result->success = false;
				result->message = "Aborted on command " + cmd + " response: " + resp;
				goal_handle->abort(result);
				return goal_handle;
			}

			feedback->step = "OK: " + cmd;
			goal_handle->publish_feedback(feedback);
		}

		auto result = std::make_shared<Homing::Result>();
		result->success = true;
		result->message = "Homing complete";
		goal_handle->succeed(result);
		return goal_handle;
	}
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GCodeSender>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
