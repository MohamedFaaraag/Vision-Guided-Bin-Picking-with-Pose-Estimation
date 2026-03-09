#include "ur5e_bin_picking/bt_nodes.hpp"

namespace ur5e_bin_picking {

BT::PortsList CheckBinEmpty::providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseArray>("detected_poses") };
}

BT::NodeStatus CheckBinEmpty::tick() {
    geometry_msgs::msg::PoseArray poses;
    if (!getInput("detected_poses", poses) || poses.poses.empty()) {
        // No objects left — bin is empty — stop the loop
        return BT::NodeStatus::SUCCESS;
    }
    // Objects remain — continue picking (FAILURE keeps the loop going)
    return BT::NodeStatus::FAILURE;
}

}  // namespace
