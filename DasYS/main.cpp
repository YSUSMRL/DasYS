#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include "MoveToPoint.hpp"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_point_action_node");


    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<MoveToPoint>("MoveToPoint");

   
    std::string xml_file = "/home/feng/roarm_ws_em0/src/roarm_behavior_nodes/bt_xml/MoveToPoint.xml";
    auto tree = factory.createTreeFromFile(xml_file);

   
    rclcpp::Rate loop_rate(10);  // 10 Hz
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

 
    rclcpp::shutdown();
    return 0;
}
