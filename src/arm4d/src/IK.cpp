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

#include <Eigen/Dense>

#include <arm4d/utill.h>
#include <arm4d/IKconfig.h>

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
		in_steps_service_->configure_introspection(
            this->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);

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

	uint8_t calculateCollision(const Eigen::Vector4d& step, double r = 60.0) {

		double x = step[0];
		double y = step[1];
		double z = step[2];
		double a = step[3];

		// simple collision checks

		constexpr double BLOCK_PLANE_Z = 0.0;
		if (z < BLOCK_PLANE_Z + r) {
			RCLCPP_ERROR(get_logger(), "Step (%.2f, %.2f, %.2f, %.2f) rejected: below ground",
			 x, y, z, a);
			return 1;
		}
		constexpr double BLOCK_PLANE_X = 15.0;
		if (x < BLOCK_PLANE_X + r) {
			RCLCPP_ERROR(get_logger(), "Step (%.2f, %.2f, %.2f, %.2f) rejected: collide X plane"
				,x, y, z, a);
			return 2;
		}
		constexpr double BLOCK_SPHERE_R = 180.0;
		constexpr double BLOCK_SPHERE_X = IKconfig::shift_x;
		constexpr double BLOCK_SPHERE_Y = IKconfig::shift_y;
		constexpr double BLOCK_SPHERE_Z = IKconfig::L0y;
		{
			const double dx = x - BLOCK_SPHERE_X;
			const double dy = y - BLOCK_SPHERE_Y;
			const double dz = z - BLOCK_SPHERE_Z;
			const double dist2 = dx*dx + dy*dy + dz*dz;
			double block_r = BLOCK_SPHERE_R + r;
			if (dist2 < block_r*block_r) {
				RCLCPP_ERROR(get_logger(), "Step (%.2f, %.2f, %.2f, %.2f) rejected: collide SPHERE"
					,x, y, z, a);
				return 3;
			}
		}
		constexpr double BLOCK_CYLINDER_R = 130.0;
		constexpr double BLOCK_CYLINDER_H = 190.0;
		constexpr double BLOCK_CYLINDER_X = IKconfig::shift_x;
		constexpr double BLOCK_CYLINDER_Y = IKconfig::shift_y;
		constexpr double BLOCK_CYLINDER_Z = 0.0;
		if (z < BLOCK_CYLINDER_Z + BLOCK_CYLINDER_H) {
			const double dx = x - BLOCK_CYLINDER_X;
			const double dy = y - BLOCK_CYLINDER_Y;
			const double dist2 = dx*dx + dy*dy;
			double block_r = BLOCK_CYLINDER_R + r;
			if (dist2 < block_r*block_r) {
				RCLCPP_ERROR(get_logger(), "Step (%.2f, %.2f, %.2f, %.2f) rejected: collide CYLINDER"
					,x, y, z, a);
				return 4;
			}
		}
		constexpr double BLOCK_BOX_X1 = -10.0, BLOCK_BOX_Y1 = 200.0, BLOCK_BOX_Z1 = 0.0;
		constexpr double BLOCK_BOX_X2 = 180.0, BLOCK_BOX_Y2 = 670.0, BLOCK_BOX_Z2 = 130.0;
		double bbx2 = BLOCK_BOX_X2 + r;
		double bby2 = BLOCK_BOX_Y2 + r;
		double bbz2 = BLOCK_BOX_Z2 + r;
		if (x > BLOCK_BOX_X1 && x < bbx2 &&
			y > BLOCK_BOX_Y1 && y < bby2 &&
			z > BLOCK_BOX_Z1 && z < bbz2) {
			RCLCPP_ERROR(get_logger(), "Step (%.2f, %.2f, %.2f, %.2f) rejected: collide BOX"
				,x, y, z, a);
			return 5;
		}
		constexpr double BLOCK_BOX2_X1 = -70.0, BLOCK_BOX2_Y1 = -70.0, BLOCK_BOX2_Z1 = -70.0;
		constexpr double BLOCK_BOX2_X2 =  70.0, BLOCK_BOX2_Y2 =  70.0, BLOCK_BOX2_Z2 =  70.0;
		if (x > BLOCK_BOX2_X1 && x < BLOCK_BOX2_X2 &&
			y > BLOCK_BOX2_Y1 && y < BLOCK_BOX2_Y2 &&
			z > BLOCK_BOX2_Z1 && z < BLOCK_BOX2_Z2) {
			RCLCPP_ERROR(get_logger(), "Step (%.2f, %.2f, %.2f, %.2f) rejected: collide BOX2 (units?)"
				,x, y, z, a);
			return 6;
		}
		return 0; // no collision
	}

	/**
	 *	@brief IK + collision loop: transforms Step→Gcode and enqueues TX.
	 */
	void ikLoop()
	{
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

			}

			arm4d_interfaces::srv::Gcode::Request gcode_req{};
			switch (step_req.type) {
			case 0:
			case 1:
			{
				Eigen::Vector4d step_vec(step_req.x, step_req.y, step_req.z, step_req.a);

				uint8_t coll = calculateCollision(step_vec, CollisionConfig::manipulatorRadius);
				if (coll != 0) {
					RCLCPP_WARN(get_logger(), "Collision detected before shift, rejecting step");
					continue;
				}

				// calculate l0 angle before end-effector offset because it depends on it

				double shifted_x = step_vec[0] - IKconfig::shift_x;
				double shifted_y = step_vec[1] - IKconfig::shift_y;
				const double l0_rad = (M_PI / 2) - std::atan2(shifted_y, shifted_x);

				// apply end-effector offset
				Eigen::Matrix3d startPos;
				startPos <<  0,-1, 0,
				             0, 0, 1,
				            -1, 0, 0;

				Eigen::AngleAxisd manipulator_rot = Eigen::AngleAxisd(startPos);
				manipulator_rot = manipulator_rot * Eigen::AngleAxisd(l0_rad, Eigen::Vector3d::UnitX());
				manipulator_rot = manipulator_rot * Eigen::AngleAxisd(-step_vec[3], Eigen::Vector3d::UnitY());

				step_vec.head<3>() -= manipulator_rot * (IKconfig::endEffectorOffset + IKconfig::baseOffset);


				coll = calculateCollision(step_vec, CollisionConfig::robotRadius);
				if (coll != 0) {
					RCLCPP_WARN(get_logger(), "Collision detected after shift, rejecting step");
					continue;
				}
				
				// IK


				shifted_x = step_vec[0] - IKconfig::shift_x;
				shifted_y = step_vec[1] - IKconfig::shift_y;

				const double u = std::sqrt(shifted_x * shifted_x + shifted_y * shifted_y) - IKconfig::L0x;
				const double v = step_vec[2] - IKconfig::L0y;
				const double alpha = std::atan2(v, u);
				const double r = std::sqrt(u*u + v*v);
				const double beta = std::acos((IKconfig::L1_2 + IKconfig::L2_2 - r*r) / (2.0 * IKconfig::L1 * IKconfig::L2));
				const double gamma = std::acos((r*r + IKconfig::L1_2 - IKconfig::L2_2) / (2.0 * IKconfig::L1 * r));
				
				constexpr double TO_DEG = 180.0 / M_PI;
				gcode_req.l0 = l0_rad * TO_DEG;
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
						// std::this_thread::sleep_for(2us); // back-off
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
