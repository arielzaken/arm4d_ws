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

using namespace std::chrono_literals;

class GCodeSender : public rclcpp::Node {
public:
	using Homing = arm4d_interfaces::action::Homing;
	using SendLine = arm4d_interfaces::srv::SendLine;
	using Gcode = arm4d_interfaces::srv::Gcode;
	using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;

	GCodeSender()
	: Node("gcode_sender"),
	  stop_(false),
	  last_state_("Unknown")
	{
		declare_parameter<std::string>("port", "/dev/ttyUSB0");
		declare_parameter<int>("baud", 115200);

		std::string port = get_parameter("port").as_string();
		int baud = get_parameter("baud").as_int();

		fd_ = openSerial(port, baud);
		if(fd_ < 0){
			RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port.c_str());
			throw std::runtime_error("serial open failed");
		}
		RCLCPP_INFO(get_logger(), "Opened serial %s @ %d", port.c_str(), baud);

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
		tx_thread_ = std::thread(&GCodeSender::txLoop, this);
	}

	~GCodeSender() override {
		stop_ = true;
		if(rx_thread_.joinable()) rx_thread_.join();
		if(tx_thread_.joinable()) tx_thread_.join();
		if(fd_ >= 0) close(fd_);
	}

private:
	// Serial state
	int fd_;
	std::atomic<bool> stop_;
	std::string last_state_;
	std::regex alarm_re_{"^alarm:?\\s*(\\d+)", std::regex::icase};

	// Threads and queues
	std::thread rx_thread_, tx_thread_;
	std::mutex q_mtx_;
	std::condition_variable q_cv_;
	std::queue<std::string> workq_;

	// ROS interfaces
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
	rclcpp::Service<Gcode>::SharedPtr send_gcode_;
	rclcpp::Service<SendLine>::SharedPtr send_gcode_string_;
	rclcpp_action::Server<Homing>::SharedPtr action_server_;

	// ---------------------- Serial helpers ----------------------
	int openSerial(const std::string &port, int baud){
		int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if(fd < 0) return -1;

		struct termios tty{};
		if(tcgetattr(fd, &tty) != 0) return -1;

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

		if(tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
		return fd;
	}

	static speed_t baudToConst(int baud){
		switch(baud){
			case 9600: return B9600;
			case 19200: return B19200;
			case 38400: return B38400;
			case 57600: return B57600;
			case 115200: return B115200;
			default: return B115200;
		}
	}

	void serialWrite(const std::string &line){
		std::string s = line;
		if(!s.empty() && s.back() != '\n') s.push_back('\n');
		write(fd_, s.c_str(), s.size());
	}

	// ---------------------- RX loop ----------------------
	void rxLoop(){
		char buf[256];
		std::string line;
		while(!stop_){
			int n = read(fd_, buf, sizeof(buf));
			if(n <= 0){ std::this_thread::sleep_for(10ms); continue; }
			for(int i=0;i<n;i++){
				if(buf[i] == '\n'){
					handleLine(line);
					line.clear();
				} else if(buf[i] != '\r'){
					line.push_back(buf[i]);
				}
			}
		}
	}

	void handleLine(const std::string &line){
		RCLCPP_INFO(get_logger(), "<< %s", line.c_str());
		if(line.rfind("<",0)==0){
			auto bar = line.find("|");
			if(bar!=std::string::npos)
				last_state_ = line.substr(1, bar-1);
			else if(line.back()=='>')
				last_state_ = line.substr(1, line.size()-2);

			auto msg = std_msgs::msg::String();
			msg.data = last_state_;
			state_pub_->publish(msg);
		}
	}

	// ---------------------- TX loop ----------------------
	void txLoop(){
		while(!stop_){
			std::unique_lock<std::mutex> lk(q_mtx_);
			if(workq_.empty()){
				q_cv_.wait_for(lk, 100ms);
				continue;
			}
			auto cmd = workq_.front();
			workq_.pop();
			lk.unlock();

			serialWrite(cmd);
			RCLCPP_INFO(get_logger(), ">> %s", cmd.c_str());
		}
	}

	// ---------------------- Subscriber ----------------------

	bool addToQueue(const std::string& str){
		// Option: cap queue length
		const size_t MAX_QUEUE = 64;
		{
			std::unique_lock<std::mutex> lk(q_mtx_);
			if (workq_.size() >= MAX_QUEUE) {
				RCLCPP_WARN(get_logger(), "Queue full, rejecting line: %s", str.c_str());
				return false; 
			}
			workq_.push(str);
			q_cv_.notify_all();
		}
		return true;
	}
	
	void cbGCode_string(
		const std::shared_ptr<rmw_request_id_t> /*request_header*/,
		const std::shared_ptr<SendLine::Request> request,
		const std::shared_ptr<SendLine::Response> response
	) {
		bool res = addToQueue(request->line);
		if(res)
			response->message = "Line accepted and acked";
		else
			response->message = "Queue full, try again later";
		response->success = res;
	}

	void cbGCode(
		const std::shared_ptr<rmw_request_id_t> /*request_header*/,
		const std::shared_ptr<Gcode::Request> request,
		const std::shared_ptr<Gcode::Response> response
	) {
		std::stringstream str;
		switch(request->type){
		case 0:
			str << "G0X" << request->l0 << "Y" << request->l1 << "Z" << request->l2 << "A" << request->l3;
			break;
		case 1:
			str << "G1X" << request->l0 << "Y" << request->l1 << "Z" << request->l2 << "A" << request->l3 << "F"<< request->feed;
			break;
		case 2:
			// TODO
			break;
		case 3:
			// TODO
			break;
		default:
			break;
		}

		response->success = addToQueue(str.str());
	}

	// ---------------------- Action server ----------------------
	rclcpp_action::GoalResponse goalCB(const rclcpp_action::GoalUUID&, std::shared_ptr<const Homing::Goal>){
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse cancelCB(const std::shared_ptr<GoalHandleHoming>){
		stop_ = true;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	rclcpp_action::GoalResponse goalResponseCB(){ return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }

	std::shared_ptr<GoalHandleHoming> execCB(const std::shared_ptr<GoalHandleHoming> goal_handle){
		auto feedback = std::make_shared<Homing::Feedback>();
		feedback->step = "Starting homing...";
		goal_handle->publish_feedback(feedback);

		// Minimal homing sequence (replace with your steps)
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
		for(auto &cmd: steps){
			if(goal_handle->is_canceling()) break;
			{
				std::unique_lock<std::mutex> lk(q_mtx_);
				workq_.push(cmd);
				q_cv_.notify_all();
			}
			feedback->step = "Sent " + cmd;
			goal_handle->publish_feedback(feedback);
			std::this_thread::sleep_for(1s); // simulate wait
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
