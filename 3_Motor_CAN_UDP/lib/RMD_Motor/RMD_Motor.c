/*
 * Torque Control of 3 GYEMS-RMD Motors on a CAN BUS while 
 * Teensy 4.1 can support 3 CAN BUSes in total.

 * Author: Jia, Xinyu
 * Last modified: Jan 22, 2022
*/

#include "RMD_Motor.h"

struct motor_args args_motor;

// arm or leg coordinate should match the motor rotation direction
float side_a[3] = {1, 1, 1};
float side_b[3] = {1, 1, 1};
float side_c[3] = {1, 1, 1};
// normalize zero position
float offset_a[3] = {0, 0, 0};
float offset_b[3] = {0, 0, 0};
float offset_c[3] = {0, 0, 0};
// limit of joint angle
float min_a[3] = {0, 0, 0}; float max_a[3] = {0, 0, 0};
float min_b[3] = {0, 0, 0}; float max_b[3] = {0, 0, 0};
float min_c[3] = {0, 0, 0}; float max_c[3] = {0, 0, 0};
// tranmission ratio
float ratio[3] = {1, 1, 1}; 

// [TO DO] EStop
void estop() {
  #ifdef ESTOP

  #endif
}

// print error messages
void print_err(char action[], char info[]) {
  #ifdef PRINT_ERROR
  Serial.print("[MOTOR-CAN-TASK]: Fail to ");
  Serial.print(action); Serial.print(" "); Serial.print(info);
  Serial.println(" msgs, to exit task");
  #endif
}


void can_init() {
  // init CAN BUS
  can1.begin();
  can1.setBaudRate(1000000);  // 1 Mbps
  can1.setMaxMB(16);  // up to 64 mailbox on Teensy 4
  can1.enableFIFO();  // MB0 callback will be assigned and fired for reception
  can1.enableFIFOInterrupt();  // to be interrupt rather than polling in the loop
  can1.onReceive(FIFO, canSniff_1);  // only allow FIFO messages
  can1.mailboxStatus();

  can2.begin();
  can2.setBaudRate(1000000);
  can2.setMaxMB(16);
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, canSniff_2);
  can2.mailboxStatus();

  can3.begin();
  can3.setBaudRate(1000000);
  can3.setMaxMB(16);
  can3.enableFIFO();
  can3.enableFIFOInterrupt();
  can3.onReceive(FIFO, canSniff_3);
  can3.mailboxStatus();

  // init req msgs
  // 0x92 is the command for getting the multi turns angle
  for (int j = 0; j < 3; j++) {
    args_motor.req_msgs_pos[j].id = (0x141 + j);
    args_motor.req_msgs_pos[j].len = 8;
    for (int n = 1; n < 8; n++) {
      args_motor.req_msgs_pos[j].buff[n] = 0;	
    }
    args_motor.req_msgs_pos[j].buff[0] = 0x92;	
  }
  
  // 0x9C is the command for getting the velocity
  for (int j = 0; j < 3; j++) {
    args_motor.req_msgs_vel[j].id = (0x141 +j);
    args_motor.req_msgs_vel[j].len = 8;
    for (int n = 1; n < 8; n++) {
      args_motor.req_msgs_vel[j].buff[n] = 0;	
    }
    args_motor.req_msgs_vel[j].buff[0] = 0x9C;		
  }
}

void control_comp(struct motor_args *args_m) {
  // PD control law: tau = tau_ff + kp * (q_des-q_data) + kd * (qd_des-qd_data)
  for (int i = 0; i < 3; i++) {
    // ESTOP if over angle limit
		if ((args_m->datas.q_a[i]<min_a[i]) || (args_m->datas.q_a[i]>max_a[i]))  estop();
		if ((args_m->datas.q_b[i]<min_b[i]) || (args_m->datas.q_b[i]>max_b[i]))  estop();
		if ((args_m->datas.q_c[i]<min_c[i]) || (args_m->datas.q_c[i]>max_c[i]))  estop();
    
    // Joint A
    args_m->torq_input.tau_a[i] = args_m->setpoints.tau_a_ff[i] + \
    (args_m->setpoints.kp_a[i]) * (args_m->setpoints.q_des_a[i] - args_m->datas.q_a[i]) + \
    (args_m->setpoints.kd_a[i]) * (args_m->setpoints.qd_des_a[i] - args_m->datas.qd_a[i]);
    
    args_m->torq_input.tau_a[i] = args_m->torq_input.tau_a[i] * side_a[i];

    // Joint B
    args_m->torq_input.tau_b[i] = args_m->setpoints.tau_b_ff[i] + \
    (args_m->setpoints.kp_b[i]) * (args_m->setpoints.q_des_b[i] - args_m->datas.q_b[i]) + \
    (args_m->setpoints.kd_b[i]) * (args_m->setpoints.qd_des_b[i] - args_m->datas.qd_bp[i]);

    args_m->torq_input.tau_b[i] = args_m->torq_input.tau_b[i] * side_b[i];

    // Joint C
    args_m->torq_input.tau_c[i] = args_m->setpoints.tau_c_ff[i] + \
    (args_m->setpoints.kp_c[i]) * (args_m->setpoints.q_des_c[i] - args_m->datas.q_c[i]) + \
    (args_m->setpoints.kd_c[i]) * (args_m->setpoints.qd_des_c[i] - args_m->datas.qd_c[i]);

    args_m->torq_input.tau_c[i] = args_m->torq_input.tau_c[i] * side_c[i];	
  }
}

void pack_torque_cmd(struct motor_args *args_m) {
  // range: -2048~2048, corresponding to the actual torque current range -33A~33A
  // pack ints into the can buffer
  int16_t iqControl;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      switch (j) {
        case 0:  // joint a
          iqControl = (args_m->torq_input.tau_a[i]) * CURRENT_SCALING;
          args_m->setpoint_msgs.setpoints_a[i].id = 0x141+ j;
          args_m->setpoint_msgs.setpoints_a[i].buff[4] = iqControl&0xff;
          args_m->setpoint_msgs.setpoints_a[i].buff[5] = (iqControl>>8)&0xff;
          args_m->setpoint_msgs.setpoints_a[i].len = 8;
          args_m->setpoint_msgs.setpoints_a[i].buff[0] = 0xA1;
          args_m->setpoint_msgs.setpoints_a[i].buff[1] = 0;
          args_m->setpoint_msgs.setpoints_a[i].buff[2] = 0;
          args_m->setpoint_msgs.setpoints_a[i].buff[3] = 0;
          args_m->setpoint_msgs.setpoints_a[i].buff[6] = 0;
          args_m->setpoint_msgs.setpoints_a[i].buff[7] = 0;
          break;
        case 1:	// joint b
          iqControl = (args_m->torq_input.tau_b[i]) * CURRENT_SCALING;
          args_m->setpoint_msgs.setpoints_b[i].id = 0x141+ j;
          args_m->setpoint_msgs.setpoints_b[i].buff[4] = iqControl&0xff;
          args_m->setpoint_msgs.setpoints_b[i].buff[5] = (iqControl>>8)&0xff;
          args_m->setpoint_msgs.setpoints_b[i].len = 8;
          args_m->setpoint_msgs.setpoints_b[i].buff[0] = 0xA1;
          args_m->setpoint_msgs.setpoints_b[i].buff[1] = 0;
          args_m->setpoint_msgs.setpoints_b[i].buff[2] = 0;
          args_m->setpoint_msgs.setpoints_b[i].buff[3] = 0;
          args_m->setpoint_msgs.setpoints_b[i].buff[6] = 0;
          args_m->setpoint_msgs.setpoints_b[i].buff[7] = 0;
          break;
        case 2:	// joint c
          iqControl = (args_m->torq_input.tau_c[i]) * CURRENT_SCALING;
          args_m->setpoint_msgs.setpoints_c[i].id = 0x141+ j;
          args_m->setpoint_msgs.setpoints_c[i].buff[4] = iqControl&0xff;
          args_m->setpoint_msgs.setpoints_c[i].buff[5] = (iqControl>>8)&0xff;
          args_m->setpoint_msgs.setpoints_c[i].len = 8;
          args_m->setpoint_msgs.setpoints_c[i].buff[0] = 0xA1;
          args_m->setpoint_msgs.setpoints_c[i].buff[1] = 0;
          args_m->setpoint_msgs.setpoints_c[i].buff[2] = 0;
          args_m->setpoint_msgs.setpoints_c[i].buff[3] = 0;
          args_m->setpoint_msgs.setpoints_c[i].buff[6] = 0;
          args_m->setpoint_msgs.setpoints_c[i].buff[7] = 0;
          break;
      }
    }
	}            
}

void unpack_reply(CAN_message_t *rx_msgs, struct joint_data *data, struct torq_msgs *torq_out, int i) {
  // 1. Motor temperature (int8_t, unit 1'C/LSB)
  // 2. Motor torque current(Iq) (int16_t, Range:-2048~2048, real torque current range:-33A~33A)
  // 3. Motor speed (int16_t, 1dps/LSB)
  // 4. Encoder position value (int16_t, 14bit encoder value range 0~16383)
  // 5. Encoder multiturn position value (int64_t, 0.01deg/LSB)
  int64_t pposition;
  int16_t pspeed;
  int16_t ptorque;

  if (rx_msgs->buff[0] == 0x9C)	{
    // feedback speed, torque
    pspeed = (rx_msgs->buff[5]<<8)|(rx_msgs->buff[4]);
    ptorque = (rx_msgs->buff[3]<<8)|(rx_msgs->buff[2]);

    switch (rx_msgs->id) {
      case 0x141:  // joint a
        data->qd_a[i] = (pspeed/ratio[0]) * DEG_TO_RADIAN * side_a[i];
        torq_out->tau_a[i] = ptorque / CURRENT_SCALING;
        break;
      case 0x142:  // joint b
        data->qd_b[i] = (pspeed/ratio[1]) * DEG_TO_RADIAN * side_b[i];
        torq_out->tau_b[i] = ptorque / CURRENT_SCALING;
        break;
      case 0x143:  // joint c
        data->qd_c[i] = (pspeed/ratio[2]) * DEG_TO_RADIAN * side_c[i];
        torq_out->tau_c[i] = ptorque / CURRENT_SCALING;
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
        data->q_a[i] = ((pposition*0.01/ratio[0]) * DEG_TO_RADIAN - offset_a[i]) * side_a[i];
        break;
      case 0x142:  // joint b
        data->q_b[i] = ((pposition*0.01/ratio[1]) * DEG_TO_RADIAN - offset_b[i]) * side_b[i];
        break;
      case 0x143:  // joint c
        data->q_c[i] = ((pposition*0.01/ratio[2]) * DEG_TO_RADIAN - offset_c[i]) * side_c[i];
        break;
    }
  }
}

// write 3 limbs simultaneously
int request_all(CAN_message_t tx_msgs[3]) {
  int err1, err2, err3;
  for (int j = 0; j < 3; j++) { // joint a, b, c
    err1 = can1.write(tx_msgs[j]);
    err2 = can2.write(tx_msgs[j]);
    err3 = can3.write(tx_msgs[j]);
    // if failed to send req
    if (err1!=1 && err2!=1 && err3!=1) {
      estop();
      return 0;
    }
  }
}

// main function
void can_task(struct motor_args *args_m)) {
  int err;
  int err1, err2, err3;

  // send 0x92 requests on CANBUS
  err = request_all(args_m->req_msgs_pos);
  if (!err) print_err("send", "position");

  // send 0x9C requests on CANBUS
  err = request_all(args_m->req_msgs_vel);
  if (!err) print_err("send", "velocity");

  // read responses from canSniff functions
  
  //------------------------------------------------------------------------

  // do control computations and pack torque command to CAN msgs
  control_comp(args_m);
  pack_torque_cmd(args_m);


  #ifdef PRINT_DATA
  printf("\n------------------Torque ------------------\n");
  print_cmd(&args_m->motor_torq_setpoints);  
  print_data(&args_m->leg_data);
  #endif

  // check CAN txqsize
  // size_t x = sizeof(args_m->setpoint_msgs);
  // Serial.print("Size of setpoints is ");
  // Serial.println(x);

  //------------------------------------------------------------------------
  
  // send msgs
  err1 = can1.write(args_m->setpoint_msgs.setpoints_a[0]);
  err2 = can2.write(args_m->setpoint_msgs.setpoints_a[1]);
  err3 = can3.write(args_m->setpoint_msgs.setpoints_a[2]);

  err1 = can1.write(args_m->setpoint_msgs.setpoints_b[0]);
  err2 = can2.write(args_m->setpoint_msgs.setpoints_b[1]);
  err3 = can3.write(args_m->setpoint_msgs.setpoints_b[2]);

  err1 = can1.write(args_m->setpoint_msgs.setpoints_c[0]);
  err2 = can2.write(args_m->setpoint_msgs.setpoints_c[1]);
  err3 = can3.write(args_m->setpoint_msgs.setpoints_c[2]);
}

// Callback CAN1
void canSniff_1(const CAN_message_t res_msgs_1) {  // 
  unpack_reply(&res_msgs_1, &args_motor->joint_data, &args_motor->torq_output, 1);				
}

// Callback CAN1
void canSniff_2(const CAN_message_t res_msgs_2) {  // 
  unpack_reply(&res_msgs_2, &args_motor->joint_data, &args_motor->torq_output, 2);				
}

// Callback CAN1
void canSniff_3(const CAN_message_t res_msgs_3) {  // 
  unpack_reply(&res_msgs_3, &args_motor->joint_data, &args_motor->torq_output, 3);				
}
