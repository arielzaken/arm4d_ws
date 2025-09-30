/**
 *	@file IK_node.cpp
 *	@brief ROS2 node: accepts Step service, performs IK, sends Gcode service.
 *	@details
 *		- Service IN: arm4d_interfaces::srv::Step  ("steps_in_IK_service")
 *		- Service OUT: arm4d_interfaces::srv::Gcode ("gcode_in")
 *		- Basic collision guards and 4R planar IK (l0..l3) as provided
 */

#include <rclcpp/rclcpp.hpp>
#include <arm4d_interfaces/srv/gcode.hpp>
#include <arm4d_interfaces/srv/step.hpp>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cmath>

#include <arm4d/utill.h>

using namespace std::chrono_literals;

class IK_node : public rclcpp::Node {
public:
	IK_node()
	: Node("IK_node"), stop_(false)
	{
		RCLCPP_INFO(get_logger(), "IK Node has been started");

		in_steps_service_ = this->create_service<arm4d_interfaces::srv::Step>(
			"steps_in_IK_service",
			std::bind(&IK_node::handle_step_request, this,
				std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
		);

		out_steps_service_ = this->create_client<arm4d_interfaces::srv::Gcode>("gcode_in");
		while (!out_steps_service_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
				throw std::runtime_error("Service wait interrupted");
			}
			RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
		}

		ik_thread_ = std::thread(&IK_node::ikLoop, this);
		tx_thread_ = std::thread(&IK_node::txLoop, this);
	}

	~IK_node() override {
		stop_ = true;
		q_cv_in_.notify_all();
		q_cv_out_.notify_all();
		if (ik_thread_.joinable()) ik_thread_.join();
		if (tx_thread_.joinable()) tx_thread_.join();
	}

private:
	// ROS
	rclcpp::Service<arm4d_interfaces::srv::Step>::SharedPtr in_steps_service_;
	rclcpp::Client<arm4d_interfaces::srv::Gcode>::SharedPtr out_steps_service_;

	// Threads
	std::atomic<bool> stop_;
	std::thread ik_thread_, tx_thread_;

	// Queues
	std::queue<arm4d_interfaces::srv::Step::Request> in_queue_;
	std::queue<arm4d_interfaces::srv::Gcode::Request> out_queue_;
	std::mutex mtx_in_, mtx_out_;
	std::condition_variable q_cv_in_, q_cv_out_;

	/**
	 *	@brief Step service callback: enqueues the request (bounded).
	 */
	void handle_step_request(
		const std::shared_ptr<rmw_request_id_t> /*req_id*/,
		const std::shared_ptr<arm4d_interfaces::srv::Step::Request> request,
		const std::shared_ptr<arm4d_interfaces::srv::Step::Response> response)
	{
		const size_t MAX_QUEUE = 10;
		{
			std::unique_lock<std::mutex> lk(mtx_in_);
			if (in_queue_.size() >= MAX_QUEUE) {
				RCLCPP_WARN(get_logger(), "IN queue full, rejecting step");
				response->success = false;
				return;
			}
			in_queue_.push(*request);
		}
		q_cv_in_.notify_one();
		response->success = true;
	}

	/**
	 *	@brief IK + collision loop: transforms Step→Gcode and enqueues TX.
	 */
	void ikLoop()
	{
		constexpr double shift_x = 127.5;
		constexpr double shift_y = 127.5;

		constexpr double L0x = 71.024;
		constexpr double L0y = 395.0;
		constexpr double L1 = 300.0;
		constexpr double L2 = 300.0;

		constexpr double L1_2 = L1*L1;
		constexpr double L2_2 = L2*L2;

		constexpr double TO_DEG = 180.0 / M_PI;

		double l0 = 0.0, l1 = 0.0, l2 = 0.0, l3 = 0.0;
		bool has_last = false;

		while (!stop_) {
			arm4d_interfaces::srv::Step::Request step_req;
			{
				std::unique_lock<std::mutex> lk(mtx_in_);
				q_cv_in_.wait(lk, [&]{ return stop_ || !in_queue_.empty(); });
				if (stop_) break;
				step_req = in_queue_.front();
				in_queue_.pop();
			}

			// collision checks for linear/rapid
			if (step_req.type == 0 || step_req.type == 1) {
				constexpr double BLOCK_PLANE_Z = 60.0;
				if (step_req.z < BLOCK_PLANE_Z) {
					RCLCPP_ERROR(get_logger(), "Step rejected: below ground");
					continue;
				}
				constexpr double BLOCK_PLANE_X = 75.0;
				if (step_req.x < BLOCK_PLANE_X) {
					RCLCPP_ERROR(get_logger(), "Step rejected: collide X plane");
					continue;
				}
				constexpr double BLOCK_SPHERE_R = 240.0;
				constexpr double BLOCK_SPHERE_X = shift_x;
				constexpr double BLOCK_SPHERE_Y = shift_y;
				constexpr double BLOCK_SPHERE_Z = L0y;
				{
					const double dx = step_req.x - BLOCK_SPHERE_X;
					const double dy = step_req.y - BLOCK_SPHERE_Y;
					const double dz = step_req.z - BLOCK_SPHERE_Z;
					const double dist2 = dx*dx + dy*dy + dz*dz;
					if (dist2 < BLOCK_SPHERE_R*BLOCK_SPHERE_R) {
						RCLCPP_ERROR(get_logger(), "Step rejected: collide SPHERE");
						continue;
					}
				}
				constexpr double BLOCK_CYLINDER_R = 190.0;
				constexpr double BLOCK_CYLINDER_H = 190.0;
				constexpr double BLOCK_CYLINDER_X = shift_x;
				constexpr double BLOCK_CYLINDER_Y = shift_y;
				constexpr double BLOCK_CYLINDER_Z = 0.0;
				if (step_req.z < BLOCK_CYLINDER_Z + BLOCK_CYLINDER_H) {
					const double dx = step_req.x - BLOCK_CYLINDER_X;
					const double dy = step_req.y - BLOCK_CYLINDER_Y;
					const double dist2 = dx*dx + dy*dy;
					if (dist2 < BLOCK_CYLINDER_R*BLOCK_CYLINDER_R) {
						RCLCPP_ERROR(get_logger(), "Step rejected: collide CYLINDER");
						continue;
					}
				}
				constexpr double BLOCK_BOX_X1 = -10.0, BLOCK_BOX_Y1 = 200.0, BLOCK_BOX_Z1 = 0.0;
				constexpr double BLOCK_BOX_X2 = 240.0, BLOCK_BOX_Y2 = 670.0, BLOCK_BOX_Z2 = 190.0;
				if (step_req.x > BLOCK_BOX_X1 && step_req.x < BLOCK_BOX_X2 &&
					step_req.y > BLOCK_BOX_Y1 && step_req.y < BLOCK_BOX_Y2 &&
					step_req.z > BLOCK_BOX_Z1 && step_req.z < BLOCK_BOX_Z2) {
					RCLCPP_ERROR(get_logger(), "Step rejected: collide BOX");
					continue;
				}
				constexpr double BLOCK_BOX2_X1 = -70.0, BLOCK_BOX2_Y1 = -70.0, BLOCK_BOX2_Z1 = -70.0;
				constexpr double BLOCK_BOX2_X2 =  70.0, BLOCK_BOX2_Y2 =  70.0, BLOCK_BOX2_Z2 =  70.0;
				if (step_req.x > BLOCK_BOX2_X1 && step_req.x < BLOCK_BOX2_X2 &&
					step_req.y > BLOCK_BOX2_Y1 && step_req.y < BLOCK_BOX2_Y2 &&
					step_req.z > BLOCK_BOX2_Z1 && step_req.z < BLOCK_BOX2_Z2) {
					RCLCPP_ERROR(get_logger(), "Step rejected: collide BOX2 (units?)");
					continue;
				}
			}

			arm4d_interfaces::srv::Gcode::Request gcode_req{};
			switch (step_req.type) {
			case 0:
			case 1:
			{
				const double shifted_x = step_req.x - shift_x;
				const double shifted_y = step_req.y - shift_y;
				gcode_req.l0 = 90.0 - std::atan2(shifted_y, shifted_x) * TO_DEG;

				const double u = std::sqrt(shifted_x * shifted_x + shifted_y * shifted_y) - L0x;
				const double v = step_req.z - L0y;
				const double alpha = std::atan2(v, u);
				const double r = std::sqrt(u*u + v*v);
				const double beta = std::acos((L1_2 + L2_2 - r*r) / (2.0 * L1 * L2));
				const double gamma = std::acos((r*r + L1_2 - L2_2) / (2.0 * L1 * r));

				gcode_req.l1 = (alpha + gamma) * TO_DEG;
				gcode_req.l2 = (alpha + gamma + beta - M_PI) * TO_DEG;

				gcode_req.l3 = step_req.a * TO_DEG - gcode_req.l2;
				if (!has_last || step_req.time <= 0.0f) {
					gcode_req.type = 0;
				} else {
					gcode_req.type = 1;
					const double dl0 = gcode_req.l0 - l0;
					const double dl1 = gcode_req.l1 - l1;
					const double dl2 = gcode_req.l2 - l2;
					const double dl3 = gcode_req.l3 - l3;
					const double ddeg = std::sqrt(dl0*dl0 + dl1*dl1 + dl2*dl2 + dl3*dl3);
					gcode_req.feed = static_cast<float>((ddeg * 60.0 * 1000.0) / step_req.time);	// deg/min
				}
				break;
			}
			default:
				gcode_req.type = step_req.type;
				gcode_req.l0 = step_req.x;
				gcode_req.l1 = step_req.y;
				gcode_req.l2 = step_req.z;
				gcode_req.l3 = step_req.a;
				gcode_req.feed = step_req.time;
				break;
			}

			l0 = gcode_req.l0; l1 = gcode_req.l1; l2 = gcode_req.l2; l3 = gcode_req.l3;
			has_last = true;

			arm4d::enqueue_blocking(out_queue_, mtx_out_, q_cv_out_, gcode_req, 10);
		}
	}

	/**
	 *	@brief TX loop: sends Gcode requests with retry.
	 */
	void txLoop()
	{
		while (!stop_) {
			arm4d_interfaces::srv::Gcode::Request gcode_req;
			{
				std::unique_lock<std::mutex> lk(mtx_out_);
				q_cv_out_.wait(lk, [&]{ return stop_ || !out_queue_.empty(); });
				if (stop_) break;
				gcode_req = out_queue_.front();
				out_queue_.pop();
			}

			bool sent = false;
			while (!sent && !stop_) {
				auto future = out_steps_service_->async_send_request(
					std::make_shared<arm4d_interfaces::srv::Gcode::Request>(gcode_req)
				);
				if (future.wait_for(2s) == std::future_status::ready) {
					auto resp = future.get();
					if (resp->success) {
						sent = true;
					} else {
						RCLCPP_WARN(get_logger(), "Gcode rejected, retrying...");
						std::this_thread::sleep_for(20us); // back-off
					}
				} else {
					RCLCPP_WARN(get_logger(), "Timeout waiting for Gcode service, retrying...");
				}
				if (!sent) std::this_thread::sleep_for(200ms);
			}
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IK_node>();
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
