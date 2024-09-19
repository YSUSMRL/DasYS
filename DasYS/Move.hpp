#ifndef MOVE_ARM_ACTION_H
#define MOVE_ARM_ACTION_H

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "roarm_moveit/srv/move_point_cmd.hpp"

class MoveToPoint : public BT::SyncActionNode
{
public:
    MoveToPoint(const std::string& name, const BT::NodeConfiguration& config);
  
    static BT::PortsList providedPorts();
  

    virtual BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<roarm_moveit::srv::MovePointCmd>::SharedPtr client_;
};

#endif  // MOVE_ARM_ACTION_H
