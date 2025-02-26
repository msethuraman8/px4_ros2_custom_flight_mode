/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

// using MyNodeWithMode = px4_ros2::NodeWithMode<DrawFlightMode>;
using DrawModeExecutorNode = px4_ros2::NodeWithModeExecutor<DrawModeExecutor, DrawFlightMode>;

static const std::string kNodeName = "draw_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrawModeExecutorNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
