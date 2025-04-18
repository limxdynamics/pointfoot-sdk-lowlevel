/**
 * @file pointfoot.h
 *
 * @brief This file contains the declarations of classes related to the control of pointfoot robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_POINTFOOT_H_
#define _LIMX_SDK_POINTFOOT_H_

#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk {
  /**
   * @brief Class for controlling a pointfoot robot using the LIMX SDK API.
   */
  class LIMX_SDK_API PointFoot : public ApiBase {
    public:
      /**
       * @brief Get an instance of the PointFoot class.
       * @return A pointer to a PointFoot instance (Singleton pattern).
       */
      static PointFoot* getInstance();

      /**
       * @brief Pure virtual initialization method.
       *        This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot.
       *                         For simulation, it is typically set to "127.0.0.1",
       *                         while for a real robot, it may be set to "10.192.1.2".
       * @return True if init successfully, otherwise false.
       */
      bool init(const std::string& robot_ip_address = "127.0.0.1") override;

      /**
       * @brief Get the number of motors in the robot.
       * @return The total number of motors.
       */
      uint32_t getMotorNumber() override;

      /**
       * @brief Override to obtain names of all robot motors.
       * Used for motor identification.
       * @return Vector of motor names; empty if unavailable.
       */
      std::vector<std::string> getMotorNames() override;

      /**
       * @brief Method to subscribe to updates of the robot's IMU (Inertial Measurement Unit) data.
       * @param cb The callback function to be invoked when new IMU data is received.
       */
      void subscribeImuData(std::function<void(const ImuDataConstPtr&)> cb) override;

      /**
       * @brief Subscribe to receive updates about the robot state.
       * The motor order for the state data is as follows:
       * PointFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint
       *   3: abad_R_Joint,  4: hip_R_Joint,  5: knee_R_Joint
       * BipedFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: ankle_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: ankle_R_Joint
       * WheelFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: wheel_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: wheel_R_Joint
       * 
       * @param cb The callback function to be invoked when a robot state update is received.
       */
      void subscribeRobotState(std::function<void(const RobotStateConstPtr&)> cb) override;

      /**
       * @brief Publish a command to control the robot's actions.
       * The motor order for the commnd data is as follows:
       * PointFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint
       *   3: abad_R_Joint,  4: hip_R_Joint,  5: knee_R_Joint
       * BipedFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: ankle_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: ankle_R_Joint
       * WheelFoot:
       *   0: abad_L_Joint,  1: hip_L_Joint,  2: knee_L_Joint,  3: wheel_L_Joint
       *   4: abad_R_Joint,  5: hip_R_Joint,  6: knee_R_Joint,  7: wheel_R_Joint
       * 
       * @param cmd The RobotCmd object representing the desired robot command.
       * @return True if the command was successfully published, otherwise false.
       */
      bool publishRobotCmd(const RobotCmd& cmd) override;

      /**
       * @brief Method to subscribe to sensor inputs related to a joystick from the robot.
       * @param cb The callback function to be invoked when sensor input from a joystick is received from the robot.
       */
      void subscribeSensorJoy(std::function<void(const SensorJoyConstPtr&)> cb) override;

      /**
       * @brief Method to subscribe to diagnostic values from the robot.
       * 
       * Examples:
       * | name        | level  | code | msg
       * |-------------|--------|------|--------------------
       * | imu         | OK     | 0    | - IMU is functioning properly.
       * | imu         | ERROR  | -1   | - Error in IMU.
       * |-------------|--------|------|--------------------
       * | ethercat    | OK     | 0    | - EtherCAT is working fine.
       * | ethercat    | ERROR  | -1   | - EtherCAT error.
       * |-------------|--------|------|--------------------
       * | calibration | OK     | 0    | - Robot calibration successful.
       * | calibration | WARN   | 1    | - Robot calibration in progress.
       * | calibration | ERROR  | -1   | - Robot calibration failed.
       * |-------------|--------|------|--------------------
       * 
       * @param cb The callback function to be invoked when diagnostic values are received from the robot.
       */
      void subscribeDiagnosticValue(std::function<void(const DiagnosticValueConstPtr&)> cb) override;

      /**
       * @brief Set the robot light effect.
       * 
       * This method configures the robot's light effect based on the provided effect parameter.
       * The effect parameter should be an integer corresponding to one of the values defined in the 
       * `LightEffect` enum, which specifies the desired robot light effect.
       * 
       * The `LightEffect` enum provides various options for the light effect, including static colors, 
       * flashing colors with different intensities, and fast or slow flashing patterns.
       * 
       * Example usage:
       * @code
       * robot.setRobotLightEffect(PointFoot::LightEffect::STATIC_RED);  // Sets the robot's light to a static red color
       * robot.setRobotLightEffect(PointFoot::LightEffect::FLASH_BLUE);  // Sets the robot's light to flashing blue
       * @endcode
       * 
       * @param effect An integer representing the desired robot light effect, as defined in the `PointFoot::LightEffect` enum.
       * @return A boolean value indicating whether the robot light effect was successfully set.
       */
      bool setRobotLightEffect(int effect) override;
      
      // Enum type that defines different robot light effects
      enum LightEffect : int {
        STATIC_RED = 0,     // Static red light
        STATIC_GREEN,       // Static green light
        STATIC_BLUE,        // Static blue light
        STATIC_CYAN,        // Static cyan light
        STATIC_PURPLE,      // Static purple light
        STATIC_YELLOW,      // Static yellow light
        STATIC_WHITE,       // Static white light
        LOW_FLASH_RED,      // Low-intensity flashing red light (slow bursts)
        LOW_FLASH_GREEN,    // Low-intensity flashing green light (slow bursts)
        LOW_FLASH_BLUE,     // Low-intensity flashing blue light (slow bursts)
        LOW_FLASH_CYAN,     // Low-intensity flashing cyan light (slow bursts)
        LOW_FLASH_PURPLE,   // Low-intensity flashing purple light (slow bursts)
        LOW_FLASH_YELLOW,   // Low-intensity flashing yellow light (slow bursts)
        LOW_FLASH_WHITE,    // Low-intensity flashing white light (slow bursts)
        FAST_FLASH_RED,     // Fast flashing red light (quick bursts)
        FAST_FLASH_GREEN,   // Fast flashing green light (quick bursts)
        FAST_FLASH_BLUE,    // Fast flashing blue light (quick bursts)
        FAST_FLASH_CYAN,    // Fast flashing cyan light (quick bursts)
        FAST_FLASH_PURPLE,  // Fast flashing purple light (quick bursts)
        FAST_FLASH_YELLOW,  // Fast flashing yellow light (quick bursts)
        FAST_FLASH_WHITE    // Fast flashing white light (quick bursts)
      };

      /**
       * @brief Destructor for the PointFoot class.
       *        Cleans up any resources used by the object.
       */
      virtual ~PointFoot();

    private:
      /**
       * @brief Private constructor to prevent external instantiation of the PointFoot class.
       */
      PointFoot();
  };
}

#endif