#include <cmath>
#include <string>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_play_plugins/straight_line_planner.hpp"


namespace nav2_play_plugins
{
    void StraightLine::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        tf_ = tf;
        global_frame_ = costmap_ros->getGlobalFrameID();

        // Parameter initialization
        nav2_util::declare_parameter_if_not_declared(
            node_,
            name_ + ".interpolation_resolution",
            rclcpp::ParameterValue(0.1)
        );
        
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    }

    void StraightLine::cleanup()
    {
        RCLCPP_INFO(
            logger_,
            "CleaningUp plugin %s of type NavfnPlanner",
            name_.c_str()
        );
    }

    void StraightLine::activate()
    {
        RCLCPP_INFO(
            logger_,
            "Activating plugin %s of type NavfnPlanner",
            name_.c_str()
        );
    }

    void StraightLine::deactivate()
    {
        RCLCPP_INFO(
            logger_,
            "Deactivating plugin %s of type NavfnPlanner",
            name_.c_str()
        );
    }

    nav_msgs::msg::Path StraightLine::createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal)
    {
        RCLCPP_INFO(
            logger_,
            "----------------------------------------"
        );
        RCLCPP_INFO(
            logger_,
            "Calling StraightLine::createPlan()"
        );
        RCLCPP_INFO(
            logger_,
            "----------------------------------------"
        );
        nav_msgs::msg::Path global_path;

        // Checking if the goal and start state is in the global frame
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(
                logger_, 
                "Planner will only except start position from %s frame",
                global_frame_.c_str()
            );
            
            return global_path;
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_INFO(
                logger_,
                "Planner will only except goal position from %s frame",
                global_frame_.c_str()
            );

            return global_path;
        }

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // Calculating the number of loops for current value of interpolation_resolution_
        int total_number_of_loop = std::hypot(
            goal.pose.position.x - start.pose.position.x,
            goal.pose.position.y - start.pose.position.y) /
            interpolation_resolution_;
        double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        for (int i = 0; i < total_number_of_loop; ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            global_path.poses.push_back(pose);
        }

        global_path.poses.push_back(goal);

        return global_path;
    }
} // nav2_play_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_play_plugins::StraightLine, nav2_core::GlobalPlanner)