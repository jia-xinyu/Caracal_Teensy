/*
Torque Control of 3 GYEMS-RMD Motors on a CAN BUS while 
Teensy 4.1 can support 3 CAN BUSes in total.

Author: Jia, Xinyu
Last modified: Jan 22, 2022
*/

#include "RMD_Motor.h"

struct motor_args args_motor;

// arm or leg coordinate should match the motor rotation direction
float side_a[3] = {-1, -1, 1};
float side_b[3] = {1, -1, 1};
float side_c[3] = {1, -1, 1};
// normalize zero position
float offset_a[3] = {0, 0, 0};
float offset_b[3] = {-PI/2, PI/2, -PI/2};
float offset_c[3] = {PI, -PI, PI};

void can_init() {
  /* init CAN BUS */
  can1.begin();
  can1.setBaudRate(1000000);  // 1 Mbps

  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();

  // [TO DO] can2 can3

  /* init req msgs */
	// 0x9c is the command for getting the velocity
	for (int i = 0; i < 3; ++i) {
	  args_motor.req_msgs_vel[i].id = (0x141 +i);
    args_motor.req_msgs_vel[i].len = 8;
		for (int j = 1; j < 8; ++j) {
			args_motor.req_msgs_vel[i].buff[j] = 0;	
		}
		args_motor.req_msgs_vel[i].buff[0] = 0x9c;		
	}

	// 0x92 is the command for getting the multi turns angle
	for (int i = 0; i < 3; ++i) {
    args_motor.req_msgs_pos[i].id = (0x141 + i);
		args_motor.req_msgs_pos[i].data_len = 8;
		for (int j = 1; j < 8; ++j) {
			args_motor.req_msgs_pos[i].buff[j] = 0;	
		}
		args_motor.req_msgs_pos[i].buff[0] = 0x92;	
	}
}

// [TO DO] EStop

