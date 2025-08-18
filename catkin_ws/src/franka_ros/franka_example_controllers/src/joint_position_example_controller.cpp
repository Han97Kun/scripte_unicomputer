// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

    bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
        constexpr size_t kNumberOfJoints = 7;
        position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface_ == nullptr) {
            ROS_ERROR(
                    "JointPositionExampleController: Error getting position joint interface from hardware!");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointPositionExampleController: Could not parse joint names");
        }
        if (joint_names.size() != kNumberOfJoints) {
            ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                                     << joint_names.size() << " instead of 7 names!");
            return false;
        }
        position_joint_handles_.resize(kNumberOfJoints);
        for (size_t i = 0; i < kNumberOfJoints; ++i) {
            try {
                position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                        "JointPositionExampleController: Exception getting joint handles: " << e.what());
                return false;
            }
        }
        command_ = std::make_shared<Trajectory>();
        command_sub_ = node_handle.subscribe<std_msgs::Float64MultiArray>(std::string("joint_command"), 1,
                                                                          &JointPositionExampleController::setCommandCallback,
                                                                          this, ros::TransportHints().tcpNoDelay());
        command_trajectory_sub_ = node_handle.subscribe<std_msgs::Float64MultiArray>(std::string("joint_trajectory_command"), 1,
                                                                                     &JointPositionExampleController::setCommandTrajectoryCallback,
                                                                                     this, ros::TransportHints().tcpNoDelay());
        trajectory_finished_pub_.init(node_handle, "/trajectory_finished", 1);
        return true;
    }

    void JointPositionExampleController::starting(const ros::Time & /* time */) {
        for (size_t i = 0; i < 7; ++i) {
            current_pose_[i] = position_joint_handles_[i].getPosition();
            command_->get().push(current_pose_[i]);
        }
        elapsed_time_ = ros::Duration(0.0);
    }

    void JointPositionExampleController::update(const ros::Time &/* time */, const ros::Duration &period) {
        elapsed_time_ += period;
        auto current_command = std::atomic_load(&command_);
        bool current_command_empty = current_command->get().empty();
        if (current_command_empty) {
            for (const auto &angle : current_pose_) {
                current_command->get().push(angle);
            }
        } else if (current_command->getSouldSendCompleted()) {
            executing_trajectory_ = true;
        }
        for (size_t i = 0; i < 7; i++) {
            position_joint_handles_[i].setCommand(current_command->get().front());
            current_pose_[i] = current_command->get().front();
            current_command->get().pop();
        }
        if (current_command_empty && executing_trajectory_) {
            executing_trajectory_ = false;
            trajectory_finished_pub_.msg_.data = current_command->getId();
            trajectory_finished_pub_.unlockAndPublish();
        }
    }

    void JointPositionExampleController::setCommandTrajectoryCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {
        auto tmp = createQueueFromMsg(msg, true);
        std::atomic_store(&command_, tmp);
    }

    void JointPositionExampleController::setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {
        auto tmp = createQueueFromMsg(msg, false);
        std::atomic_store(&command_, tmp);
    }

    std::shared_ptr<Trajectory> JointPositionExampleController::createQueueFromMsg(const std_msgs::Float64MultiArrayConstPtr &msg, bool should_send_completed) {
        auto tmp = std::make_shared<Trajectory>(should_send_completed);
        if(msg->data.size() % 7 != 0) {
            ROS_ERROR("Could not set command. Did not receive multiple of %zu angles (%zu)",
                      position_joint_handles_.size(),
                      msg->data.size());
        }
        for(const auto& angle : msg->data) {
            tmp->get().push(angle);
        }
        return tmp;
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)