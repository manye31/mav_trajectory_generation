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

#ifndef TRAJECTORY_SAMPLER_NODE_H
#define TRAJECTORY_SAMPLER_NODE_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_msgs/msg/polynomial_segment.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

// deprecated
#include <mav_planning_msgs/msg/polynomial_segment4_d.h>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.h>

class TrajectorySamplerNode : public rclcpp::Node {
 public:
  TrajectorySamplerNode(const rclcpp::NodeOptions & options);
  ~TrajectorySamplerNode();

 private:
  void pathSegmentsCallback(
      const mav_planning_msgs::msg::PolynomialTrajectory::SharedPtr segments_message);
  void pathSegments4DCallback(
      const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr segments_message);
  bool stopSamplingCallback(const std_srvs::srv::Empty::Request::SharedPtr request,
                            std_srvs::srv::Empty::Response::SharedPtr response);
  void commandExecute();
  void processTrajectory();

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr trajectory4D_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::Time start_time_;

  // Service client for getting the MAV interface to listen to our sent
  // commands.
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr position_hold_client_;

  // Flag whether to publish entire trajectory at once or not.
  bool publish_whole_trajectory_;
  // Trajectory sampling interval.
  double dt_;
  // Time at currently published trajectory sample.
  double current_sample_time_;

  // The trajectory to sub-sample.
  mav_trajectory_generation::Trajectory trajectory_;
};

#endif  // TRAJECTORY_SAMPLER_NODE_H
