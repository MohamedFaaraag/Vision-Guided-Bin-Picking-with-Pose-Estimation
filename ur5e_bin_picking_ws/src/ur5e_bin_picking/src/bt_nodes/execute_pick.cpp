#include "ur5e_bin_picking/bt_nodes.hpp"

namespace ur5e_bin_picking {

BT::PortsList ExecutePick::providedPorts() {
    return { BT::InputPort<bool>("plan_success") };
}

BT::NodeStatus ExecutePick::onStart() {
    bool plan_ok;
    if (!getInput("plan_success", plan_ok) || !plan_ok) {
        return BT::NodeStatus::FAILURE;
    }
    execution_started_ = true;
    execution_complete_ = false;
    // Execute the MTC pick task
    // task->execute(*task->solutions().front());
    execution_complete_ = true;
    execution_success_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecutePick::onRunning() {
    if (execution_complete_) {
        return execution_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void ExecutePick::onHalted() {}

}  // namespace
