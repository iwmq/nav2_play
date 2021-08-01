// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_play_plugins/noti_at_waypoint.hpp"
#include "nav2_play_plugins/straight_line_planner.hpp"


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NotiAtWaypointTest, NoInput)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testNotiAtWaypoint");

  node->declare_parameter("NAP.timeout", 5.0);
  node->declare_parameter("NAP.input_topic", "/waypoint_follower/noti_at_wait");
  node->declare_parameter("NAP.noti_topic", "/waypoint_follower/notification");

  nav2_play_plugins::NotiAtWaypoint nap;
  nap.initialize(node, std::string("NAP"));

  auto start_time = node->now();

  // should wait 5 seconds
  geometry_msgs::msg::PoseStamped pose;
  nap.processAtWaypoint(pose, 0);

  auto end_time = node->now();

  EXPECT_NEAR((end_time - start_time).seconds(), 5.0, 0.01);
}


TEST(NotiAtWaypointTest, GetInput)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testNotiAtWaypoint");
  
  auto pub = node->create_publisher<std_msgs::msg::Empty>("/waypoint_follower/noti_at_wait", 1);
  pub->on_activate();
  auto publish_message = 
    [&, this]() -> void
    {
      rclcpp::Rate(1).sleep();
      auto msg = std::make_unique<std_msgs::msg::Empty>();
      pub->publish(std::move(msg));
      rclcpp::spin_some(node->shared_from_this()->get_node_base_interface());
    };

  node->declare_parameter("NAP.timeout", 5.0);
  node->declare_parameter("NAP.input_topic", "/waypoint_follower/noti_at_wait");
  node->declare_parameter("NAP.noti_topic", "/waypoint_follower/notification");

  nav2_play_plugins::NotiAtWaypoint nap;
  nap.initialize(node, std::string("NAP"));

  auto start_time = node->now();

  // No input, should timeout
  geometry_msgs::msg::PoseStamped pose;
  nap.processAtWaypoint(pose, 0);

  auto end_time = node->now();
  EXPECT_NEAR((end_time - start_time).seconds(), 5.0, 0.01);

  // Has input now, shold work
  std::thread t{publish_message};
  EXPECT_TRUE(nap.processAtWaypoint(pose, 0));
  t.join();
}