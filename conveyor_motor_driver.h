/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_REALTURTLEBOT_MOTOR_DRIVER_H_
#define TURTLEBOT3_REALTURTLEBOT_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define DYNAMIXEL_POWER_ENABLE_PIN      32//
#define ADDR_X_TORQUE_ENABLE            64//
#define ADDR_X_GOAL_VELOCITY            104//
#define ADDR_X_GOAL_POSITION            116//

#define ADDR_X_PROFILE_ACCELERATION     108
#define ADDR_X_PROFILE_VELOCITY         112
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0


#define WHEEL_L_R 1  // WHEEL LEFT REAR
#define WHEEL_R_R 2  // WHEEL RIGHT REAR
#define WHEEL_L_F 3  // WHEEL LEFT FRONT
#define WHEEL_R_F 4  // WHEEL RIGHT FRONT
#define JOINT_L_R 5  // JOINT LEFT REAR
#define JOINT_R_R 6  // JOINT RIGHT REAR
#define JOINT_L_F 7  // JOINT LEFT FRONT
#define JOINT_R_F 8  // JOINT RIGHT FRONT

#define BAUDRATE                        1000000 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define BODY_LENGTH           25.6 //(((((())))))
#define SPEED_ADD_ON          2//(((((((())))))))

#define X_POS_MIN                       0
#define X_POS_MAX                       4095
#define X_POS_CENTER                    2048

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);
  void syncWrite(int address, int length, int value);
  void syncWrite(int address, int length, int* value);
  void syncRead(int address, int length, int* readValues);


 private:
  uint32_t baudrate_;
  float  protocol_version_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
};

#endif // TURTLEBOT3_REALTURTLEBOT_MOTOR_DRIVER_H_
