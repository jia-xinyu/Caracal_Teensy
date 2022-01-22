#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <FlexCAN_T4.h>
#include "joint_message.h"

#define PI 3.141593f
#define RPM_TO_RPS 0.104720  // convert rpm to radian per sec
#define DEG_TO_RADIAN 0.017453  // PI/180, convert degree to radian
#define CURRENT_SCALING 43  // 62.06 /* 2048 / 33A */

// Joint Safety Limits
//[TO DO]

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// final joint torque, joint feedback torque
struct torq_msgs {
  float tau_a[3];
  float tau_b[3];
  float tau_c[3];
};

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
  CAN_message_t res_msgs[MAX_READ];

  // CAN msgs for sending setpoints
  struct setpoint_msgs setpoint_msgs;

  // join command and joint data
  joint_command setpoints;
  joint_data datas;

  // final joint torque to motor
  struct torq_msgs torq_input;
  // feedback torque from motor
  struct torq_msgs torq_output;
};

void can_init();

#endif