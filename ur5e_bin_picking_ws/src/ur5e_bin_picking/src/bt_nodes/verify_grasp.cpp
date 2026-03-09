#include "ur5e_bin_picking/bt_nodes.hpp"
#include <control_msgs/action/gripper_command.hpp>

namespace ur5e_bin_picking {

BT::PortsList VerifyGrasp::providedPorts() { return {}; }

BT::NodeStatus VerifyGrasp::tick() {
    // Method 1: Check gripper position feedback
    // If gripper is fully closed (position ~= 0) then nothing was grasped -> FAILURE
    // If gripper stopped partway (position > threshold) -> object is held -> SUCCESS
    //
    // Read gripper joint state from /joint_states topic
    // finger_joint value: 0.0 = open, 0.8 = fully closed
    // If object is held, finger_joint will be between 0.1 and 0.7

    // Placeholder: assume grasp succeeded
    return BT::NodeStatus::SUCCESS;
}

}  // namespace
