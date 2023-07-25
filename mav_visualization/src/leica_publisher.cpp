/*
 * Copyright (c) 2016, Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "mav_visualization/leica_marker.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("leica_publisher");

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);

  std::string frame_id = "leica";
  double scale = 1.0;

  node->declare_parameter("frame_id", frame_id);
  node->declare_parameter("scale", scale);
  
  node->get_parameter("frame_id", frame_id);
  node->get_parameter("scale", scale);

  mav_visualization::LeicaMarker leica;
  visualization_msgs::msg::MarkerArray markers;

  leica.setLifetime(0.0);
  leica.setAction(visualization_msgs::msg::Marker::ADD);

  std_msgs::msg::Header header;
  header.frame_id = frame_id;

  while (rclcpp::ok()) {
    header.stamp = rclcpp::Clock().now();
    leica.setHeader(header);
    leica.getMarkers(markers, scale, false);
    marker_pub->publish(markers);
    // ++header.seq;

    rclcpp::sleep_for(std::chrono::milliseconds(5000)); 
  }
}
