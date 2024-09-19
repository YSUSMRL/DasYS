#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <rclcpp/rclcpp.hpp>
#include "roarm_moveit/srv/move_point_cmd.hpp"
#include "MoveToPoint.hpp"
#include <memory>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "roarm_moveit_cmd/ik.h"

MoveToPoint::MoveToPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("move_to_point_action_node");
    client_ = node_->create_client<roarm_moveit::srv::MovePointCmd>("move_point_cmd");
}

BT::PortsList MoveToPoint::providedPorts()
{
    return { BT::InputPort<float>("x"),
             BT::InputPort<float>("y"),
             BT::InputPort<float>("z") };
}

BT::NodeStatus MoveToPoint::tick()
{
     float x,y,z;
    if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z))
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [x, y, z]");
        return BT::NodeStatus::FAILURE;
    }

    moveit::planning_interface::MoveGroupInterface move_group(node_, "hand");
   

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    RCLCPP_INFO(node_->get_logger(), "x: %f, y: %f, z: %f", x, y, z);
    cartesian_to_polar(1000 * target_pose.position.x, 1000 * target_pose.position.y, &base_r, &BASE_point_RAD);
    simpleLinkageIkRad(l2, l3, base_r, 1000 * target_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "base_r: %f, BASE_point_RAD: %f", base_r, BASE_point_RAD);

    RCLCPP_INFO(node_->get_logger(), "BASE_point_RAD: %f, SHOULDER_point_RAD: %f, ELBOW_point_RAD: %f", BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD);
  
    std::vector<double> target = {BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD, 0};
    move_group.setJointValueTarget(target);

   
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(my_plan);
        RCLCPP_INFO(node_->get_logger(), "MoveToPoint executed successfully");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
        return BT::NodeStatus::FAILURE;
    }
}
