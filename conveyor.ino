#include "conveyor.h"
#include "conveyor_mode.h"

RC100 Controller;
Turtlebot3MotorDriver motor_driver;
TurtlebotMotion turtlebotMotion;

int rcData = 0;

Dynamixel Dxl;
int vel[4] = {WHEEL_L_R, 0, WHEEL_R_R, 0};
int const_vel = 150;

DynamixelStatus dynamixelStatus;

void setup()
{
  Serial.begin(57600);

  Controller.begin(1);

  Dxl.begin(3);
  Dxl.writeByte(WHEEL_L_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(WHEEL_R_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(WHEEL_L_F, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(WHEEL_R_F, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(JOINT_L_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(JOINT_R_R, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(JOINT_L_F, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(JOINT_R_F, ADDR_TORQUE_ENABLE, ON);

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, dynamixelStatus.setWheelVel(), 8);
  Dxl.syncWrite(ADDR_GOAL_POSITION, 1, dynamixelStatus.setJointAngle(), 8);
}

void loop()
{
  if (Controller.available())
  {
    rcData = Controller.readData();
    Serial.print("Direction : ");
    dynamixelStatus.getDirection(rcData);
    Serial.println(dynamixelStatus.showMode());
    delay(1);

    dynamixelStatus.setParams();
    Dxl.syncWrite(ADDR_GOAL_POSITION, 1, dynamixelStatus.setJointAngle(), 8);
    Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, dynamixelStatus.setWheelVel(), 8);

    Serial.print("aaaa : ");
    Serial.println(dynamixelStatus.distance_from_center);

    // Serial.print("rcData = ");
    // Serial.print(rcData);
    // Serial.print(" LEFT_VEL = ");
    // Serial.print(vel[1]);
    // Serial.print(" RIGHT_VEL = ");
    // Serial.println(vel[3]);

    // dynamixelStatus.getDirection(rcData);
    //
    // if(rcData & RC100_BTN_U)
    // {
    //   vel[1] += velocity;
    //   vel[3] -= velocity;
    // }
    // else if(rcData & RC100_BTN_D)
    // {
    //   vel[1] -= velocity;
    //   vel[3] += velocity;
    // }
    // else if(rcData & RC100_BTN_L)
    // {
    //   vel[1] -= velocity;
    //   vel[3] -= velocity;
    // }
    // else if(rcData & RC100_BTN_R)
    // {
    //   vel[1] += velocity;
    //   vel[3] += velocity;
    // }
    // else if(rcData & RC100_BTN_1)
    // {
    //   const_vel += 10;
    // }
    // else if(rcData & RC100_BTN_3)
    // {
    //   const_vel -= 10;
    // }
    // else if(rcData & RC100_BTN_6)
    // {
    //   vel[1] = const_vel;
    //   vel[3] = -const_vel;
    // }
    // else if(rcData & RC100_BTN_5)
    // {
    //   vel[1] = 0;
    //   vel[3] = 0;
    // }
    // Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
  }
}

void setup()
{
  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for RC100 remote control and cmd_vel
  remote_controller.begin(1);  //57600bps for RC100

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  // Init Motion
  controlRealTurtleBot();
}

void loop()
{
  receiveRemoteControlData();
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlRealTurtleBot);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Receive RC100 remote controller data
*******************************************************************************/
void receiveRemoteControlData(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    turtlebotMotion.getDirection(received_data);

    controlRealTurtleBot();

    remote_controller.begin(1);  // refresh remote controller buffer
  }
}

/*******************************************************************************
* Control Real TurtleBot
*******************************************************************************/
void controlRealTurtleBot()
{
  int pres_pos[8] = {0, };
  int* goal_pos;
  double dist[8] = {0.0, };
  int goal_pos_dist_int[8] = {0, };
  double goal_pos_dist_db[8] = {0.0, };

  turtlebotMotion.setParams();

  for (int motion_num = 0; motion_num < turtlebotMotion.motion_all_num; motion_num++)
  {
    double dist_number = turtlebotMotion.time_duration[motion_num] / 0.008;

    goal_pos = turtlebotMotion.setJointAngle(motion_num);
    motor_driver.syncRead(ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION, pres_pos);

    for (int joint_num = 0; joint_num < 8; joint_num++)
    {
      dist[joint_num] = (goal_pos[joint_num] * 1.0 - pres_pos[joint_num] * 1.0) / dist_number;
      goal_pos_dist_db[joint_num] = pres_pos[joint_num] * 1.0;
    }

    for (int res_num = 0; res_num < (int)dist_number; res_num++)
    {
      for (int joint_num = 0; joint_num < 8; joint_num++)
      {
        goal_pos_dist_db[joint_num] += dist[joint_num];
        goal_pos_dist_int[joint_num] = (int)goal_pos_dist_db[joint_num];
      }

      motor_driver.syncWrite(ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION, goal_pos_dist_int);

      delay(8);
    }

    delay(turtlebotMotion.time_space[motion_num] * 1000.0);
  }
}
