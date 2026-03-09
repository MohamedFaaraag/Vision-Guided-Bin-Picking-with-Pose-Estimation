#include "ur5e_bin_picking/bt_nodes.hpp"

namespace ur5e_bin_picking {

BT::PortsList ExecutePlace::providedPorts() { return {}; }

BT::NodeStatus ExecutePlace::onStart() {
    // Execute the place portion of the MTC task
    // (transport, lower, release, detach, retreat)
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecutePlace::onRunning() {
    // Monitor execution completion
    return BT::NodeStatus::SUCCESS;
}

void ExecutePlace::onHalted() {}

}  // namespace
