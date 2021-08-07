// Copyright (c) 2021, Jay Wang
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

#ifndef NAV2_PLAY_PLUGINS__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_PLAY_PLUGINS__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.h"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"


namespace nav2_play_plugins
{
class StraightLine: public nav2_core::GlobalPlanner
{
    public:
        StraightLine() = default;
        ~StraightLine() = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override;

        void cleanup() override;

        void activate() override;

        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped& start,
            const geometry_msgs::msg::PoseStamped& goal
        ) override;

    private:
        std::shared_ptr<tf2_ros::Buffer> tf_;

        nav2_util::LifecycleNode::SharedPtr node_;

        nav2_costmap_2d::Costmap2D* costmap_;

        std::string global_frame_, name_;

        double interpolation_resolution_;

        rclcpp::Logger logger_{rclcpp::get_logger("straight_line_planner")};
};

}  // namespace nav2_play_plugins

#endif  // NAV2_PLAY_PLUGINS__STRAIGHT_LINE_PLANNER_HPP_
