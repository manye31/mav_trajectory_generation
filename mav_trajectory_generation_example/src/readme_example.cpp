/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");
  ros::NodeHandle nh("");

  //! ROS
  ros::Publisher pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  ros::Publisher pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");
  ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++) {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  // -------------------------------------------------------------------------------------------------------------
  //! Make plan
  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

  start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
  vertices.push_back(start);

  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
  middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,1));
  vertices.push_back(middle);

  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,-2,3));
  vertices.push_back(middle);

  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3,-1, 4));
  vertices.push_back(middle);

  end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
  vertices.push_back(end);

  //! Compute segment times
  std::vector<double> segment_times;
  const double v_max = 2.0;
  const double a_max = 2.0;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // segment_times.push_back(3.97);
  // segment_times.push_back(4.47);
  // segment_times.push_back(6.0);

  for (auto i : segment_times) {
    ROS_INFO("Segment %f", i);
  }

  const int N = 10; // he template parameter (N) denotes the number of coefficients of the underlying polynomial,
                    // which has to be even. If we want the trajectories to be snap-continuous,
                    // N needs to be at least 10; for minimizing jerk, 8
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);

  //!! Create trajectory
  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  // -------------------------------------------------------------------------------------------------------------
  //! Evaluating trajectory in time
  //! 1. Sampling the trajectory class
  // Single sample:
  // double sampling_time = 2.0;
  // int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
  // Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);
  // std::cout << "sampling at ";
  // std::cout << sampling_time << ": \n" << sample << std::endl;

  // Sample range:
  // double t_start = 2.0;
  // double t_end = 10.0;
  // double dt = 0.01;
  // std::vector<Eigen::VectorXd> result;
  // std::vector<double> sampling_times; // Optional.
  // trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
  // std::cout << "sampling at times: \n";
  // for (auto i: sampling_times) {
  //   std::cout << i << ", ";
  // }
  // std::cout << "sampling result: \n";
  // for (auto p: result) {
  //   std::cout << p << "\n";
  // }

  //! 2. Convert to mav_msgs::EigenTrajectoryPoint state
  mav_msgs::EigenTrajectoryPoint state;
  mav_msgs::EigenTrajectoryPoint::Vector states;
  bool success;

  // Single sample:
  // double sampling_time = 2.0;
  // bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory, sample_time, &state);

  // Sample range:
  // double t_start = 2.0;
  // double duration = 10.0;
  // double dt = 0.01;
  // success = mav_trajectory_generation::sampleTrajectoryInRange(trajectory, t_start, duration, dt, &states);

  // Whole trajectory:
  double sampling_interval = 0.01;
  success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
  std::cout << success;
  // size_t x = 0;
  // for (auto s : states) {
  //   std::cout << "State: " << x << ":" << std::endl;
  //   std::cout << s.toString() << std::endl;
  //   x++;
  // }

  // -------------------------------------------------------------------------------------------------------------
  //! Viz
  visualization_msgs::MarkerArray markers;
  double distance = 0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  // From Trajectory class:
  // mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

  // From mav_msgs::EigenTrajectoryPoint::Vector states:
  // Sampled by space
  mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);

  // Sampled by time
  // mav_trajectory_generation::drawMavSampledTrajectoryByTime(states, 0.1, frame_id, &markers);
  pub_markers_.publish(markers);

  // -------------------------------------------------------------------------------------------------------------
  //! ROS Publish
  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}