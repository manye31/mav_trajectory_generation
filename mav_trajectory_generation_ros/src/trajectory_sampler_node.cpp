/*
 * Copyright (c) 2017, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>

TrajectorySamplerNode::TrajectorySamplerNode(const rclcpp::NodeOptions & options)
    : Node(options.arguments()[0], options),
      publish_whole_trajectory_(true),
      dt_(0.01),
      current_sample_time_(0.0) {

  command_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  trajectory_sub_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory>(
    "path_segments", 10,
    std::bind(&TrajectorySamplerNode::pathSegmentsCallback, this,
                std::placeholders::_1));
  trajectory4D_sub_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>(
    "path_segments_4D", 10,
    std::bind(&TrajectorySamplerNode::pathSegments4DCallback, this,
                std::placeholders::_1));

  stop_srv_ = this->create_service<std_srvs::srv::Empty>(
      "stop_sampling",
      std::bind(&TrajectorySamplerNode::stopSamplingCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  position_hold_client_ = this->create_client<std_srvs::srv::Empty>(
    "back_to_position_hold");

  this->declare_parameter("publish_whole_trajectory", publish_whole_trajectory_);
  this->declare_parameter("dt", dt_);

  publish_whole_trajectory_ = this->get_parameter("publish_whole_trajectory").as_bool();
  dt_ = this->get_parameter("dt").as_double();

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000*dt_)),
    std::bind(&TrajectorySamplerNode::commandExecute, this));
}

TrajectorySamplerNode::~TrajectorySamplerNode() { publish_timer_->cancel(); }

void TrajectorySamplerNode::pathSegmentsCallback(
    const mav_planning_msgs::msg::PolynomialTrajectory::SharedPtr segments_message) {
  RCLCPP_INFO(this->get_logger(), "Received trajectory msg");
  if (segments_message->segments.empty()) {
    RCLCPP_WARN(this->get_logger(), "Trajectory sampler: received empty waypoint message");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Trajectory sampler: received %lu waypoints",
             segments_message->segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        *segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::pathSegments4DCallback(
    const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr segments_message) {
  RCLCPP_INFO(this->get_logger(), "Received trajectory4d msg");
  if (segments_message->segments.empty()) {
    RCLCPP_WARN(this->get_logger(), "Trajectory sampler: received empty waypoint message");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Trajectory sampler: received %lu waypoints",
             segments_message->segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        *segments_message, &trajectory_);
    RCLCPP_INFO(this->get_logger(), "Success %d", success);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::processTrajectory() {
  // Call the service call to takeover publishing commands.
  RCLCPP_INFO(this->get_logger(), "processing (whole? %d) trajectory...", publish_whole_trajectory_);
  // if (position_hold_client_.exists()) {
  //   std_srvs::srv::Empty empty_call;
  //   position_hold_client_.call(empty_call);
  // }

  if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
                                                     &trajectory_points);

    size_t x = 0;
    for (auto s : trajectory_points) {
      RCLCPP_INFO(this->get_logger(), "state %d:\n", x);
      // std::cout << "State: " << x << ":" << std::endl;
      std::cout << s.toString() << std::endl;
      x++;
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
    command_pub_->publish(msg_pub);


  } else {
    current_sample_time_ = 0.0;
    start_time_ = rclcpp::Clock().now();
  }
}

bool TrajectorySamplerNode::stopSamplingCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  publish_timer_->cancel();
  return true;
}

void TrajectorySamplerNode::commandExecute() {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    trajectory_msgs::msg::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      publish_timer_->cancel();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    msg.points[0].time_from_start = rclcpp::Duration(current_sample_time_);
    command_pub_->publish(msg);
    current_sample_time_ += dt_;
  } else {
    publish_timer_->cancel();
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"trajectory_sampler_node"});

  std::shared_ptr<TrajectorySamplerNode> trajectory_sampler_node = 
    std::make_shared<TrajectorySamplerNode>(options);
  RCLCPP_INFO(trajectory_sampler_node->get_logger(), "Created trajectory sampler.");
  rclcpp::spin(trajectory_sampler_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
