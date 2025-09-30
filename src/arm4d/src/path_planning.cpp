/**
 *	@file Path_Planning_node.cpp
 *	@brief ROS2 Action server that converts a polyline goal into Step service calls.
 *	@details
 *		- Action: arm4d_interfaces::action::MoveOnALine  (points[], velocities[])
 *		- For each segment, emit a Step to "steps_in_IK_service"
 *		- Sends feedback with current_point_index, sets action result at the end.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm4d_interfaces/msg/vector4.hpp>
#include <arm4d_interfaces/action/move_on_a_line.hpp>
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

using namespace std::chrono_literals;
using MoveOnALine = arm4d_interfaces::action::MoveOnALine;
using GoalHandleMoveOnALine = rclcpp_action::ServerGoalHandle<MoveOnALine>;

class Path_Planning_node : public rclcpp::Node {
public:
	/**
	 *	@brief Ctor: starts the action server and TX thread.
	 */
	Path_Planning_node()
	: Node("Path_Planning_node"), stop_(false)
	{
		RCLCPP_INFO(get_logger(), "Path planning Node has been started");

		action_server_ = rclcpp_action::create_server<MoveOnALine>(
			this,
			"move_on_a_line",
			std::bind(&Path_Planning_node::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Path_Planning_node::handle_cancel, this, std::placeholders::_1),
			std::bind(&Path_Planning_node::handle_accepted, this, std::placeholders::_1)
		);

		out_steps_service_ = this->create_client<arm4d_interfaces::srv::Step>("steps_in_IK_service");
		
		while (!out_steps_service_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
				throw std::runtime_error("Service wait interrupted");
			}
			RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
		}

		tx_thread_ = std::thread(&Path_Planning_node::txLoop, this);
	}

	/**
	 *	@brief Dtor: joins threads and clears queues.
	 */
	~Path_Planning_node() override {
		stop_ = true;
		q_cv_out_.notify_all();
		if (tx_thread_.joinable()) tx_thread_.join();
	}

private:
	// ROS handles
	rclcpp::Client<arm4d_interfaces::srv::Step>::SharedPtr out_steps_service_;
	rclcpp_action::Server<MoveOnALine>::SharedPtr action_server_;

	// Threading
	std::atomic<bool> stop_;
	std::thread tx_thread_;

	// Outgoing Step queue
	std::queue<arm4d_interfaces::srv::Step::Request> out_queue_;
	std::mutex mtx_out_;
	std::condition_variable q_cv_out_;

	/**
	 *	@brief Accept all goals.
	 */
	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID & /*uuid*/,
		std::shared_ptr<const MoveOnALine::Goal> goal)
	{
		// basic sanity checks
		if(goal->points.size() != goal->velocities.size()){
			RCLCPP_ERROR(get_logger(), "Points size (%zu) != velocities size (%zu)", goal->points.size(), goal->velocities.size());
			return rclcpp_action::GoalResponse::REJECT;
		}
		if(goal->points.size() != goal->times_from_start.size()){
			RCLCPP_ERROR(get_logger(), "Points size (%zu) != times_from_start size (%zu)", goal->points.size(), goal->times_from_start.size());
			return rclcpp_action::GoalResponse::REJECT;
		}
		if(goal->times_from_start.size() != goal->velocities.size()){
			RCLCPP_ERROR(get_logger(), "velocities size (%zu) != times_from_start size (%zu)", goal->points.size(), goal->times_from_start.size());
			return rclcpp_action::GoalResponse::REJECT;
		}
		if(goal->points.size() < 2){
			RCLCPP_ERROR(get_logger(), "Need at least two points to make a line, got %zu", goal->points.size());
			return rclcpp_action::GoalResponse::REJECT;
		}
		if(goal->times_from_start[0] != 0.0){
			RCLCPP_ERROR(get_logger(), "First time_from_start must be zero, got %f", goal->times_from_start[0]);
			return rclcpp_action::GoalResponse::REJECT;
		}
		if(goal->velocities[0].x != 0.0 || goal->velocities[0].y != 0.0 || goal->velocities[0].z != 0.0 || goal->velocities[0].a != 0.0){
			RCLCPP_ERROR(get_logger(), "First velocity must be zero, got (%f,%f,%f,%f)", goal->velocities[0].x, goal->velocities[0].y, goal->velocities[0].z, goal->velocities[0].a);
			return rclcpp_action::GoalResponse::REJECT;
		}
		if(goal->velocities.back().x != 0.0 || goal->velocities.back().y != 0.0 || goal->velocities.back().z != 0.0 || goal->velocities.back().a != 0.0){
			RCLCPP_ERROR(get_logger(), "Last velocity must be zero, got (%f,%f,%f,%f)", goal->velocities.back().x, goal->velocities.back().y, goal->velocities.back().z, goal->velocities.back().a);
			return rclcpp_action::GoalResponse::REJECT;
		}
		// check for negative times and monotonic increasing times
		double last_time = -1.0;
		for(auto time : goal->times_from_start){
			if(time < 0.0){
				RCLCPP_ERROR(get_logger(), "Negative time_from_start not allowed, got %f", time);
				return rclcpp_action::GoalResponse::REJECT;
			}
			if(time <= last_time){
				RCLCPP_ERROR(get_logger(), "time_from_start must be strictly increasing, got %f after %f", time, last_time);
				return rclcpp_action::GoalResponse::REJECT;
			}
			last_time = time;
		}

		RCLCPP_INFO(get_logger(), "Goal: %zu points", goal->points.size());
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 *	@brief Always accept cancellation.
	 */
	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleMoveOnALine> /*goal_handle*/)
	{
		RCLCPP_INFO(get_logger(), "Cancel requested");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	/**
	 *	@brief Spawn a worker thread for the accepted goal.
	 */
	void handle_accepted(const std::shared_ptr<GoalHandleMoveOnALine> goal_handle)
	{
		std::thread{std::bind(&Path_Planning_node::execute_goal, this, std::placeholders::_1), goal_handle}.detach();
	}

	/**
	 *	@brief Worker: converts points[] + velocities[] to Step segments.
	 *	@details
	 *		- If velocities[i] present, use its magnitude for the segment speed (mm/s).
	 *		- If velocities missing or zero, default to 100 mm/s.
	 *		- Segment time = distance / speed (clamped to >= 1ms).
	 */
	void execute_goal(const std::shared_ptr<GoalHandleMoveOnALine> goal_handle)
	{
		constexpr double dt = 5.0; // mm/s


		const auto goal = goal_handle->get_goal();
		auto feedback = std::make_shared<MoveOnALine::Feedback>();
		auto result = std::make_shared<MoveOnALine::Result>();

		const auto &pts = goal->points;
		const auto &vels = goal->velocities;
        const auto &time = goal->times_from_start;

		// stream each point as a linear step (type=1) from previous point
		for (size_t i = 0; i < pts.size() - 1; ++i) {
			const Eigen::Vector4d xCurrent = Eigen::Vector4d(pts[i].x, pts[i].y, pts[i].z, pts[i].a);
			const Eigen::Vector4d xNext = Eigen::Vector4d(pts[i+1].x, pts[i+1].y, pts[i+1].z, pts[i+1].a);
			const Eigen::Vector4d vCurrent = Eigen::Vector4d(vels[i].x, vels[i].y, vels[i].z, vels[i].a);
			const Eigen::Vector4d vNext = Eigen::Vector4d(vels[i+1].x, vels[i+1].y, vels[i+1].z, vels[i+1].a);
			double dTime = (time[i+1] - time[i]); // the difference in arival time of the target points

			const Eigen::Vector4d a = xCurrent; // noop because compiler is cool, the constant term of the third order polynomial
			const Eigen::Vector4d b = vCurrent; // noop because compiler is cool, the linear term of the third order polynomial
			const Eigen::Vector4d c = (3*xNext-3*xCurrent-2*vCurrent*dTime-vNext*dTime)/(dTime * dTime); // the quadratic term of the third order polynomial
			const Eigen::Vector4d d = (2*xCurrent+(vCurrent+vNext)*dTime-xNext*dTime)/(dTime*dTime*dTime); // the cubic term of the third order polynomial

			for(double currentTime = 0; currentTime < dTime; currentTime += dt){
				if (goal_handle->is_canceling()) {
					result->success = false;
					result->message = "Canceled";
					goal_handle->canceled(result);
					return;
				}
				const Eigen::Vector4d pathPoint = a + b*currentTime + c*currentTime*currentTime + d*currentTime*currentTime*currentTime;
				arm4d_interfaces::srv::Step::Request step{};
				step.type = 1; // linear
				step.x = pathPoint[0];
				step.y = pathPoint[1];
				step.z = pathPoint[2];
				step.a = pathPoint[3];
				step.time = dt;

				arm4d::enqueue_blocking(out_queue_, mtx_out_, q_cv_out_, step, 10);

				feedback->current_point_index = static_cast<uint32_t>(i);
				goal_handle->publish_feedback(feedback);
			}
		}

		result->success = true;
		result->message = "Path streamed";
		goal_handle->succeed(result);
	}

	/**
	 *	@brief TX loop: sends Step requests to IK service with retry.
	 */
	void txLoop()
	{
		while (!stop_) {
			arm4d_interfaces::srv::Step::Request step_req;
			{
				std::unique_lock<std::mutex> lk(mtx_out_);
				q_cv_out_.wait(lk, [&]{ return stop_ || !out_queue_.empty(); });
				if (stop_) break;
				step_req = out_queue_.front();
				out_queue_.pop();
			}

			bool sent = false;
			while (!sent && !stop_) {
				auto future = out_steps_service_->async_send_request(
					std::make_shared<arm4d_interfaces::srv::Step::Request>(step_req)
				);

				if (future.wait_for(2s) == std::future_status::ready) {
					auto resp = future.get();
					if (resp->success) {
						sent = true;
					} else {
						RCLCPP_WARN(get_logger(), "Step rejected by IK, retrying...");
						std::this_thread::sleep_for(20us); // back-off
					}
				} else {
					RCLCPP_WARN(get_logger(), "Timeout waiting for IK service, retrying...");
				}
				if (!sent) std::this_thread::sleep_for(200ms);
			}
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Path_Planning_node>();
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
