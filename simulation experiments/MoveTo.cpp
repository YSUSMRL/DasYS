#include "MoveTo.hpp"
#include "std_msgs/msg/bool.hpp" 
MoveTo::MoveTo(const std::string& name,
    const NodeConfig& conf,
    const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
    client = params.nh.lock();
    // 初始化发布器
    task_complete_publisher_ = client->create_publisher<std_msgs::msg::Bool>("task_complete", 10);
}

PortsList MoveTo::providedPorts()
{
    return {
        BT::InputPort<std::string>("location"),
    };
}

bool MoveTo::setGoal(RosActionNode::Goal &goal)
{
    // Retrieve the location from the input port
    std::string loc;
    getInput("location", loc);

    // Load the YAML file containing location information
    const std::string file_path = client->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(file_path);
    // Extract the coordinates for the specified location
    std::vector<float> current_goal = locations[loc].as<std::vector<float>>();
    // Set the goal for the action
    goal.pose.header.stamp = client->now();
    goal.pose.header.frame_id = "map";

    goal.pose.pose.position.x = current_goal[0];
    goal.pose.pose.position.y = current_goal[1];
    float yaw_angle = current_goal[2];
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = sin(yaw_angle / 2);  // Convert yaw to quaternion z component
    goal.pose.pose.orientation.w = cos(yaw_angle / 2);  // Convert yaw to quaternion w component
    //goal.pose.pose.orientation.w = 1.0;
    //goal.pose.pose.orientation.z = 0.0;

    return true;
}

NodeStatus MoveTo::onResultReceived(const RosActionNode::WrappedResult &wr)
{   // 任务完成后发布Bool消息
    std_msgs::msg::Bool msg;
    msg.data = true;
    task_complete_publisher_->publish(msg);
    RCLCPP_INFO(client->get_logger(), "Published task complete message.");

    RCLCPP_INFO(client->get_logger(), "Goal reached\n");
    return NodeStatus::SUCCESS;
}

NodeStatus MoveTo::onFailure(ActionNodeErrorCode error)
{   
    RCLCPP_ERROR(client->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

NodeStatus MoveTo::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    return NodeStatus::RUNNING;
}
