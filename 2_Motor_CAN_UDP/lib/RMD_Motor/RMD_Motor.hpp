#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <FlexCAN_T4.h>
#include "joint_message.hpp"

#define RPM_TO_RPS 0.104720     // convert rpm to radian per sec
#define DEG_TO_RADIAN 0.017453  // PI/180, convert degree to radian
// #define CURRENT_SCALING 43      // X8-Pro-6:1, mannual: 62.06 = 2048/33A
#define CURRENT_SCALING 165     // L7015-10T
// #define CURRENT_SCALING 3800    // L5010-10T

// #define ESTOP  // turn ESTOP on or off
// #define PRINT_DATA  // turn print on or off
// #define PRINT_ERROR

// data struct for passing data
struct motor_args {
  // CAN msgs for sending requests, receiving responses
  CAN_message_t req_msgs_vel[3];
  CAN_message_t req_msgs_pos[3];
  
  // CAN msgs for sending setpoints
  CAN_message_t setpoints_a[3];
  CAN_message_t setpoints_b[3];
  CAN_message_t setpoints_c[3];

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