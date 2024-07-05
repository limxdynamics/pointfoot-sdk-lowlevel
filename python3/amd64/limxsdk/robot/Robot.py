"""
@brief This file contains the declarations of classes related to the control of robots.

@file Robot.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
from typing import Callable, Any
import limxsdk.datatypes as datatypes
import limxsdk.robot as robot

class Robot(object):
    """
    @class Robot
    @brief Represents a robot with various functionalities.

    This class provides an interface to interact with different types of robots.
    """

    def __init__(self, robot_type: robot.RobotType):
        """
        @brief Initializes a Robot object with a specified type.

        @param robot_type: Type of robot, either RobotType.PointFoot or RobotType.Wheellegged.
        """
        __slots__ = ['robot', 'robot_ip']

        # Default robot IP address
        self.robot_ip = '127.0.0.1'

        # Create a native robot instance based on the specified type
        if robot_type == robot.RobotType.PointFoot:
            self.robot = robot.RobotNative("PointFoot")
        elif robot_type == robot.RobotType.Wheellegged:
            self.robot = robot.RobotNative("Wheellegged")

    def init(self, robot_ip: str = "127.0.0.1"):
        """
        @brief Initializes the robot with a specified IP address.

        @param robot_ip: IP address of the robot.
        @return: True if initialization is successful, False otherwise.
        """
        self.robot_ip = robot_ip
        return self.robot.init(self.robot_ip)

    def getMotorNumber(self):
        """
        @brief Gets the number of motors of the robot.

        @return: Number of motors.
        """
        return self.robot.getMotorNumber()

    def subscribeImuData(self, callback: Callable[[datatypes.ImuData], Any]):
        """
        @brief Subscribes to receive updates on the robot's imu.

        @param callback: Callable[[datatypes.ImuData], Any]: 
                  Callback function to handle imu updates.
        @return: Subscription status.
        """
        return self.robot.subscribeImuData(callback)

    def subscribeRobotState(self, callback: Callable[[datatypes.RobotState], Any]):
        """
        @brief Subscribes to receive updates on the robot's state.

        @param callback: Callable[[datatypes.RobotState], Any]: 
                  Callback function to handle robot state updates.
        @return: Subscription status.
        """
        return self.robot.subscribeRobotState(callback)

    def publishRobotCmd(self, cmd: datatypes.RobotCmd):
        """
        @brief Publishes a robot command.

        @param cmd: Robot command to be published.
        @return: Status of the command publication.
        """
        return self.robot.publishRobotCmd(cmd)

    def getJointOffset(self, timeout: float = -1.0):
        """
        @brief Gets the joint offset of the robot.

        @param timeout: Timeout for getting joint offset (-1 for infinite waiting time).
        @return: Joint offset.
        """
        return self.robot.getJointOffset(timeout)

    def getJointLimit(self, timeout: float = -1.0):
        """
        @brief Gets the joint limit of the robot.

        @param timeout: Timeout for getting joint limit (-1 for infinite waiting time).
        @return: Joint limit.
        """
        return self.robot.getJointLimit(timeout)

    def subscribeSensorJoy(self, callback: Callable[[datatypes.SensorJoy], Any]):
        """
        @brief Subscribes to receive sensor joy updates.

        @param callback: Callable[[datatypes.SensorJoy], Any]: 
                  Callback function to handle sensor joy updates.
        @return: Subscription status.
        """
        return self.robot.subscribeSensorJoy(callback)

    def subscribeDiagnosticValue(self, callback: Callable[[datatypes.DiagnosticValue], Any]):
        """
        @brief Subscribes to receive diagnostic value updates.

        @param callback (Callable[[datatypes.DiagnosticValue], Any]): 
                  Callback function to handle diagnostic value updates.
        @return: Subscription status.
        """
        return self.robot.subscribeDiagnosticValue(callback)

    def setRobotLightEffect(self, effect: datatypes.LightEffect):
        """
        @brief Sets the robot's light effect.

        This method configures the robot's light effect based on the provided effect parameter.
        The `effect` parameter should be an instance of the `LightEffect` enum, which provides 
        predefined values for different light effects. The enum value is converted to its integer 
        representation before being sent to the robot's API.

        @param effect: An instance of the `LightEffect` enum representing the desired robot light effect. 
                      The method converts this enum value to an integer to be compatible with the underlying 
                      robot API. Refer to the `LightEffect` enum for valid values. For example:
                      - `datatypes.LightEffect.STATIC_RED` corresponds to a static red light.
                      - `datatypes.LightEffect.FAST_FLASH_BLUE` corresponds to a fast-flashing blue light.

        @return: `True` if the light effect was successfully set, `False` otherwise. The return value indicates
                whether the operation was successful.
        """
        # Convert the enum to its integer value and call the robot's method
        return self.robot.setRobotLightEffect(effect.value)

