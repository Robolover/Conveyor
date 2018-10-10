#include "conveyor.h"
#include "conveyor_mode.h"

RC100 Controller;
ConveyorMotorDriver motor_driver;
ConveyorMode conveyor_mode;

int rcData = 0;
int const_vel = 150;

void setup()
{
  Serial.begin(57600);

  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for RC100 remote control and cmd_vel
  Controller.begin(1);  //57600bps for RC100
  motor_driver.syncWriteVelocity(ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, conveyor_mode.setWheelVel());
  motor_driver.syncWritePosition(ADDR_GOAL_POSITION, LEN_GOAL_POSITION, conveyor_mode.setJointAngle());

  pinMode(13, OUTPUT);
}

void loop()
{
  if (Controller.available())
  {
    rcData = Controller.readData();
    Serial.print("Direction : ");

    conveyor_mode.getDirection(rcData);
    Serial.println(conveyor_mode.showMode());
    delay(1);

    conveyor_mode.setParams();
    motor_driver.syncWriteVelocity(ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, conveyor_mode.setWheelVel());
    motor_driver.syncWritePosition(ADDR_GOAL_POSITION, LEN_GOAL_POSITION, conveyor_mode.setJointAngle());

    Serial.print("aaaa : ");
    Serial.println(conveyor_mode.distance_from_center);
  }
}
