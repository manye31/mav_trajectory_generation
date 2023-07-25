#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "mav_visualization/hexacopter_marker.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hexacopter_publisher");
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);

  std::string frame_id = "state";
  double scale = 1.0;
  bool simple = false;

  node->declare_parameter("frame_id", frame_id);
  node->declare_parameter("scale", scale);
  node->declare_parameter("simple", simple);

  node->get_parameter("frame_id", frame_id);
  node->get_parameter("scale", scale);
  node->get_parameter("simple", simple);

  mav_visualization::HexacopterMarker hex(simple);
  visualization_msgs::msg::MarkerArray markers;

  hex.setLifetime(0.0);
  hex.setAction(visualization_msgs::msg::Marker::ADD);

  std_msgs::msg::Header header;
  header.frame_id = frame_id;

  while (rclcpp::ok()) {
    header.stamp = rclcpp::Clock().now();
    hex.setHeader(header);
    hex.getMarkers(markers, scale, false);
    marker_pub->publish(markers);
    // header.seq++;

    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  return 0;
}