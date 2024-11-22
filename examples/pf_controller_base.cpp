/**
 * @file pf_controller_base.cpp
 * @brief Implementation file for the PFControllerBase class.
 * @version 1.0
 * @date 2024-3-5
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include "pf_controller_base.h"

// Constructor
PFControllerBase::PFControllerBase()
{
  // Initialize PointFoot instance and robot command/state objects
  pf_ = limxsdk::PointFoot::getInstance();
  robotstate_on_ = false;
  robot_cmd_ = limxsdk::RobotCmd(pf_->getMotorNumber());
  robot_state_ = limxsdk::RobotState(pf_->getMotorNumber());

  // Subscribe to robot state updates
  pf_->subscribeRobotState([&](const limxsdk::RobotStateConstPtr &msg)
                           {
      mtx_.lock();
      robot_state_ = *msg;
      robotstate_on_ = true;
      mtx_.unlock(); });

  // Subscribe to robot state updates
  pf_->subscribeImuData([&](const limxsdk::ImuDataConstPtr &msg)
                           {
      imu_data_ = *msg;
  });
 }

// Destructor
PFControllerBase::~PFControllerBase()
{
}

// Function to control a single joint
void PFControllerBase::singleJointController(int jointId, double kp, double kd,
                                             double targetPos, double targetVel,
                                             double targetTorque)
{
  robot_cmd_.Kp[jointId] = kp;
  robot_cmd_.Kd[jointId] = kd;
  robot_cmd_.q[jointId] = targetPos;
  robot_cmd_.dq[jointId] = targetVel;
  robot_cmd_.tau[jointId] = targetTorque;
  pf_->publishRobotCmd(robot_cmd_);
}

// Function to control all joints simultaneously
void PFControllerBase::groupJointController(std::vector<float> &kp, std::vector<float> &kd,
                                            std::vector<float> &targetPos, std::vector<float> &targetVel,
                                            std::vector<float> &targetTorque)
{
  for (size_t i = 0; i < pf_->getMotorNumber(); ++i)
  {
    robot_cmd_.Kp[i] = kp[i];
    robot_cmd_.Kd[i] = kd[i];
    robot_cmd_.q[i] = targetPos[i];
    robot_cmd_.dq[i] = targetVel[i];
    robot_cmd_.tau[i] = targetTorque[i];
  }
  pf_->publishRobotCmd(robot_cmd_);
}

// Function to publish zero torque commands
void PFControllerBase::zeroTorque()
{
  for (size_t i = 0; i < pf_->getMotorNumber(); ++i)
  {
    robot_cmd_.Kp[i] = 0.0;
    robot_cmd_.Kd[i] = 0.0;
    robot_cmd_.q[i] = 0.0;
    robot_cmd_.dq[i] = 0.0;
    robot_cmd_.tau[i] = 0.0;
  }
  pf_->publishRobotCmd(robot_cmd_);
}

// Function to publish damping commands
void PFControllerBase::damping()
{
  for (size_t i = 0; i < pf_->getMotorNumber(); ++i)
  {
    robot_cmd_.Kp[i] = 0.0;
    robot_cmd_.Kd[i] = 4.0; // Assuming 4 as the damping value
    robot_cmd_.q[i] = 0.0;
    robot_cmd_.dq[i] = 0.0;
    robot_cmd_.tau[i] = 0.0;
  }
  pf_->publishRobotCmd(robot_cmd_);
}
