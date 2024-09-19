#include "GenerateNextDestination.hpp"

GenerateNextDestination::GenerateNextDestination(const std::string& name, 
    const NodeConfig& config)
    : SyncActionNode(name, config)
{
}

PortsList GenerateNextDestination::providedPorts()
{
    // new_goal input identify the new goal destination
    return{ BT::OutputPort<std::string>("new_goal")};
}

NodeStatus GenerateNextDestination::tick()
{   
    // mock locations to identify new goal destinations
    std::string locations[] = {"Station A", "Station B", "Parking"};

    // Set as output a random goal location among the available ones
    setOutput("new_goal", locations[(rand() % (sizeof(locations) / sizeof(locations[0])))]);
    
    return NodeStatus::SUCCESS;
}
