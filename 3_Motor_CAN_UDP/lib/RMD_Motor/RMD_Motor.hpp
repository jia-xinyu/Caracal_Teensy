#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <FlexCAN_T4.h>
#include "joint_message.hpp"

#define RPM_TO_RPS 0.104720  // convert rpm to radian per sec
#define DEG_TO_RADIAN 0.017453  // PI/180, convert degree to radian
#define CURRENT_SCALING 43  // 62.06 /* 2048 / 33A */

// #define ESTOP  // turn ESTOP on or off
// #define PRINT_DATA  // turn print on or off
#define PRINT_ERROR

// CAN msgs for sending setpoints
struct setpoint_msgs {
  CAN_message_t setpoints_a[3];
  CAN_message_t setpoints_b[3];
  CAN_message_t setpoints_c[3];
};

// data struct for passing data
struct motor_args {
  // CAN msgs for sending requests, receiving responses
  CAN_message_t req_msgs_vel[3];
  CAN_message_t req_msgs_pos[3];
  // CAN_message_t res_msgs[MAX_READ];
  
  // CAN msgs for sending setpoints
  struct setpoint_msgs setpoint_msgs;

  // join command and joint data
  joint_command joint_CMD;
  joint_data joint_DATA;
};

void can_init();
void can_task();
void can_events();
joint_command *get_can_command();
joint_data *get_can_data();

#endif