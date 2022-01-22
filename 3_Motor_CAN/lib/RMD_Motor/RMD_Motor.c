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
  can1.setMaxMB(16);  // up to 64 mailbox on Teensy 4
  can1.enableFIFO();  // MB0 callback will be assigned and fired for reception
  can1.enableFIFOInterrupt();  // to be interrupt rather than polling in the loop
  can1.onReceive(FIFO, canSniff);  // only allow FIFO messages
  can1.mailboxStatus();

  can2.begin();
  can2.setBaudRate(1000000);
  can2.setMaxMB(16);
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, canSniff);
  can2.mailboxStatus();

  can3.begin();
  can3.setBaudRate(1000000);
  can3.setMaxMB(16);
  can3.enableFIFO();
  can3.enableFIFOInterrupt();
  can3.onReceive(FIFO, canSniff);
  can3.mailboxStatus();

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
		args_motor.req_msgs_pos[i].len = 8;
		for (int j = 1; j < 8; ++j) {
			args_motor.req_msgs_pos[i].buff[j] = 0;	
		}
		args_motor.req_msgs_pos[i].buff[0] = 0x92;	
	}
}

// [TO DO] EStop
void estop()

void control_comp(struct motor_args *args) {
	// tau = tau_ff + kp * (q_des-q_data) + kd * (qd_des-qd_data)

	for (int i = 0; i < 3; ++i) {
		/*======================== Joint A ========================*/
		if (args->datas.q_abad[i] > A_LIM_P) {
			printf("IMPORTANT: ESTOP APPLIED to abad[%d]\n", i);
			estop(); 
		}	else if (args->datas.q_abad[i] < A_LIM_N) {
			printf("IMPORTANT: ESTOP APPLIED to abad[%d]\n", i);
			estop();
		}

		args->torq_input.tau_a[i] = args->setpoints.tau_a_ff[i] + \
		(args->setpoints.kp_a[i]) * (args->setpoints.q_des_a[i] - args->datas.q_a[i]) + \
		(args->setpoints.kd_a[i]) * (args->setpoints.qd_des_a[i] - args->datas.qd_a[i]);

		args->torq_input.tau_a[i] = args->torq_input.tau_a[i] * side_a[i];

		/*======================== Joint B ========================*/
		if (args->datas.q_hip[i] > H_LIM_P) {
			printf("IMPORTANT: ESTOP APPLIED to hip[%d]\n", i);
			estop(); 
		}	else if (args->datas.q_hip[i] < H_LIM_N) {
			printf("IMPORTANT: ESTOP APPLIED to hip[%d]\n", i);
			estop();
		}

		args->torq_input.tau_b[i] = args->setpoints.tau_b_ff[i] + \
		(args->setpoints.kp_hip[i]) * (args->setpoints.q_des_hip[i] - args->datas.q_hip[i]) + \
		(args->setpoints.kd_hip[i]) * (args->setpoints.qd_des_hip[i] - args->datas.qd_hip[i]);

		args->torq_input.tau_b[i] = args->torq_input.tau_b[i] * side_b[i];

		/*======================== Joint C ========================*/
		if (args->datas.q_knee[i] > K_LIM_P) {
			printf("IMPORTANT: ESTOP APPLIED to knee[%d]\n" , i);
			estop(); 
		}	else if (args->datas.q_knee[i] < K_LIM_N) {
			printf("IMPORTANT: ESTOP APPLIED to knee[%d]\n", i);
			estop();
		}

		args->torq_input.tau_c[i] = args->setpoints.tau_c_ff[i] + \
		(args->setpoints.kp_knee[i]) * (args->setpoints.q_des_knee[i] - args->datas.q_knee[i]) + \
		(args->setpoints.kd_knee[i]) * (args->setpoints.qd_des_knee[i] - args->datas.qd_knee[i]);

		args->torq_input.tau_c[i] = args->torq_input.tau_c[i] * side_c[i];	
	}
}

void pack_torque_cmd(struct motor_args* args) {
  // range: -2048~2048, corresponding to the actual torque current range -33A~33A
  // pack ints into the can buffer
  int16_t iqControl;
	for (int j = 0; j < 3; ++j) {
	 	for (int i = 0; i < 3; ++i) {
      switch (i) {
        case 0:  // joint a
          iqControl = (args->torq_input.tau_a[j]) * CURRENT_SCALING;
          args->setpoint_msgs.setpoints_a[j].id = 0x141+ i;
          args->setpoint_msgs.setpoints_a[j].buff[4] = iqControl&0xff;
          args->setpoint_msgs.setpoints_a[j].buff[5] = (iqControl>>8)&0xff;
          args->setpoint_msgs.setpoints_a[j].len= 8;
          args->setpoint_msgs.setpoints_a[j].buff[0] = 0xA1;
          args->setpoint_msgs.setpoints_a[j].buff[1] = 0;
          args->setpoint_msgs.setpoints_a[j].buff[2] = 0;
          args->setpoint_msgs.setpoints_a[j].buff[3] = 0;
          args->setpoint_msgs.setpoints_a[j].buff[6] = 0;
          args->setpoint_msgs.setpoints_a[j].buff[7] = 0;
          break;
        case 1:	// joint b
          iqControl = (args->torq_input.tau_b[j]) * CURRENT_SCALING;
          args->setpoint_msgs.setpoints_b[j].id = 0x141+ i;
          args->setpoint_msgs.setpoints_b[j].buff[4] = iqControl&0xff;
          args->setpoint_msgs.setpoints_b[j].buff[5] = (iqControl>>8)&0xff;
          args->setpoint_msgs.setpoints_b[j].len= 8;
          args->setpoint_msgs.setpoints_b[j].buff[0] = 0xA1;
          args->setpoint_msgs.setpoints_b[j].buff[1] = 0;
          args->setpoint_msgs.setpoints_b[j].buff[2] = 0;
          args->setpoint_msgs.setpoints_b[j].buff[3] = 0;
          args->setpoint_msgs.setpoints_b[j].buff[6] = 0;
          args->setpoint_msgs.setpoints_b[j].buff[7] = 0;
          break;
        case 2:	// joint c
          iqControl = (args->torq_input.tau_c[j]) * CURRENT_SCALING;
          args->setpoint_msgs.setpoints_c[j].id = 0x141+ i;
          args->setpoint_msgs.setpoints_c[j].buff[4] = iqControl&0xff;
          args->setpoint_msgs.setpoints_c[j].buff[5] = (iqControl>>8)&0xff;
          args->setpoint_msgs.setpoints_c[j].len= 8;
          args->setpoint_msgs.setpoints_c[j].buff[0] = 0xA1;
          args->setpoint_msgs.setpoints_c[j].buff[1] = 0;
          args->setpoint_msgs.setpoints_c[j].buff[2] = 0;
          args->setpoint_msgs.setpoints_c[j].buff[3] = 0;
          args->setpoint_msgs.setpoints_c[j].buff[6] = 0;
          args->setpoint_msgs.setpoints_c[j].buff[7] = 0;
          break;
      }
	 	}
	}            
}

void unpack_reply(struct CAN_message_t *rx_msgs, joint_data *data, torq_msgs *torq_out, int n) {
  // 1. Motor temperature (int8_t, unit 1'C/LSB)
  // 2. Motor torque current(Iq) (int16_t, Range:-2048~2048, real torque current range:-33A~33A)
  // 3. Motor speed (int16_t, 1dps/LSB)
  // 4. Encoder position value (int16_t, 14bit encoder value range 0~16383)
	int64_t pposition;
	int16_t pspeed;
	int16_t ptorque;

	if (rx_msgs->buff[0] == 0x9c)	{
    // feedback speed, torque
		pspeed = (rx_msgs->buff[5]<<8)|(rx_msgs->buff[4]);
		ptorque = (rx_msgs->buff[3]<<8)|(rx_msgs->buff[2]);

		switch (rx_msgs->id) {
			case 0x141:  // joint a
				data->qd_a[n] = (pspeed/6) * DEG_TO_RADIAN * side_a[n];
				torq_out->tau_a[n] = ptorque / CURRENT_SCALING;
				break;
			case 0x142:  // joint b
				data->qd_b[n] = (pspeed/6) * DEG_TO_RADIAN * side_b[n];
				torq_out->tau_b[n] = ptorque / CURRENT_SCALING;
				break;
			case 0x143:  // joint c
				data->qd_c[n] = (pspeed/6) * DEG_TO_RADIAN * side_c[n];
				torq_out->tau_c[n] = ptorque / CURRENT_SCALING;
				break;
		}

	}	else if (rx_msgs->buff[0] == 0x92) {
    // feedback position
		pposition = ( rx_msgs->buff[1] | rx_msgs->buff[2] << 8| \
		rx_msgs->buff[3] << 16 | rx_msgs->buff[4] << 24 | \
		(uint64_t)rx_msgs->buff[5] << 32 |(uint64_t)rx_msgs->buff[6] << 40 | \
		(uint64_t)rx_msgs->buff[7] << 48 );

		switch (rx_msgs->id) {
			case 0x141:  // joint a
				data->q_a[n] = ((pposition*0.01/6) * DEG_TO_RADIAN - offset_abad[n]) * side_a[n];
				break;
			case 0x142:  // joint b
				data->q_b[n] = ((pposition*0.01/6) * DEG_TO_RADIAN - offset_hip[n]) * side_b[n];
				break;
			case 0x143:  // joint c
				data->q_c[n] = ((pposition*0.01/6) * DEG_TO_RADIAN - offset_knee[n]) * side_c[n];
				break;
		}
	}
}    