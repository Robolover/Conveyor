#include "conveyor.h"
#include "conveyor_mode.h"

RC100 Controller;
ConveyorMotorDriver motor_driver;
TurtlebotMotion turtlebotMotion;

int rcData = 0;
int vel[4] = {WHEEL_L_R, 0, WHEEL_R_R, 0};
int const_vel = 150;

void setup()
{
  Serial.begin(57600);

  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for RC100 remote control and cmd_vel
  Controller.begin(1);  //57600bps for RC100

  pinMode(13, OUTPUT);
}

void loop()
{
  if (Controller.available())
  {
    rcData = Controller.readData();
    Serial.print("Direction : ");
    turtlebotMotion.getDirection(rcData);
    Serial.println(turtlebotMotion.showMode());
    delay(1);

    turtlebotMotion.setParams();
    motor_driver.syncWrite(ADDR_GOAL_POSITION, 1, turtlebotMotion.setJointAngle(), 8);
    motor_driver.syncWrite(ADDR_GOAL_VELOCITY, 1, turtlebotMotion.setWheelVel(), 8);
//  motor_driver.syncWrite(ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION, goal_pos_dist_int);

    Serial.print("aaaa : ");
    Serial.println(turtlebotMotion.distance_from_center);

    // Serial.print("rcData = ");
    // Serial.print(rcData);
    // Serial.print(" LEFT_VEL = ");
    // Serial.print(vel[1]);
    // Serial.print(" RIGHT_VEL = ");
    // Serial.println(vel[3]);

    // turtlebotMotion.getDirection(rcData);
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
