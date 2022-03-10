/*
 * Torque Control of 3 GYEMS-RMD Motors on a CAN BUS while 
 * Teensy 4.1 can support 3 CAN BUSes in total.

 * Author: Jia, Xinyu
 * Last modified: Jan 22, 2022
*/

#include "RMD_Motor.hpp"

struct motor_args args_motor;

// callback functions cannot get access to same address simultaneously
joint_data can_rx[3];

// 3 CAN BUS, define buffer size
FlexCAN_T4<CAN1, RX_SIZE_64, TX_SIZE_32> can1;
FlexCAN_T4<CAN2, RX_SIZE_64, TX_SIZE_32> can2;
FlexCAN_T4<CAN3, RX_SIZE_64, TX_SIZE_32> can3;

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
float ratio[3] = {50, 1, 1};

// [TO DO] EStop
void estop() {
  #ifdef ESTOP

  #endif
}

// Get CAN command pointer
joint_command *get_can_command() {
  return &args_motor.joint_CMD;
}

// Get CAN data pointer
joint_data *get_can_data() {
  return &args_motor.joint_DATA; 
}

//------------------------------------------------------------------------

void pack_torque_cmd(struct motor_args *args_m) {
  // range: -2048~2048, corresponding to the actual torque current range -33A~33A
  // pack ints into the can buffer
  int16_t iqControl;
  // joint a
  for (int i = 0; i < 3; i++) {
    args_m->joint_CMD.tau_a_des[i] = args_m->joint_CMD.tau_a_des[i] * side_a[i] / ratio[0];
    iqControl = (args_m->joint_CMD.tau_a_des[i]) * CURRENT_SCALING;
    args_m->setpoints_a[i].id = 0x141+ 0;
    args_m->setpoints_a[i].buf[4] = iqControl&0xff;
    args_m->setpoints_a[i].buf[5] = (iqControl>>8)&0xff;
    args_m->setpoints_a[i].len = 8;
    args_m->setpoints_a[i].buf[0] = 0xA1;
    args_m->setpoints_a[i].buf[1] = 0;
    args_m->setpoints_a[i].buf[2] = 0;
    args_m->setpoints_a[i].buf[3] = 0;
    args_m->setpoints_a[i].buf[6] = 0;
    args_m->setpoints_a[i].buf[7] = 0;
  }

  // joint b
  for (int i = 0; i < 3; i++) {
    args_m->joint_CMD.tau_b_des[i] = args_m->joint_CMD.tau_b_des[i] * side_b[i] / ratio[1];
    iqControl = (args_m->joint_CMD.tau_b_des[i]) * CURRENT_SCALING;
    args_m->setpoints_b[i].id = 0x141+ 1;
    args_m->setpoints_b[i].buf[4] = iqControl&0xff;
    args_m->setpoints_b[i].buf[5] = (iqControl>>8)&0xff;
    args_m->setpoints_b[i].len = 8;
    args_m->setpoints_b[i].buf[0] = 0xA1;
    args_m->setpoints_b[i].buf[1] = 0;
    args_m->setpoints_b[i].buf[2] = 0;
    args_m->setpoints_b[i].buf[3] = 0;
    args_m->setpoints_b[i].buf[6] = 0;
    args_m->setpoints_b[i].buf[7] = 0;
  }

  // joint c
  for (int i = 0; i < 3; i++) {
    args_m->joint_CMD.tau_c_des[i] = args_m->joint_CMD.tau_c_des[i] * side_c[i] / ratio[2] ;
    iqControl = (args_m->joint_CMD.tau_c_des[i]) * CURRENT_SCALING;
    args_m->setpoints_c[i].id = 0x141+ 2;
    args_m->setpoints_c[i].buf[4] = iqControl&0xff;
    args_m->setpoints_c[i].buf[5] = (iqControl>>8)&0xff;
    args_m->setpoints_c[i].len = 8;
    args_m->setpoints_c[i].buf[0] = 0xA1;
    args_m->setpoints_c[i].buf[1] = 0;
    args_m->setpoints_c[i].buf[2] = 0;
    args_m->setpoints_c[i].buf[3] = 0;
    args_m->setpoints_c[i].buf[6] = 0;
    args_m->setpoints_c[i].buf[7] = 0;
  }
}

void unpack_reply(CAN_message_t rx_msgs, struct joint_data *data, int i) {
  // 1. Motor temperature (int8_t, unit 1'C/LSB)
  // 2. Motor torque current(Iq) (int16_t, Range:-2048~2048, real torque current range:-33A~33A)
  // 3. Motor speed (int16_t, 1dps/LSB)
  // 4. Encoder position value (int16_t, 14bit encoder value range 0~16383)
  // 5. Encoder multiturn position value (int64_t, 0.01deg/LSB)
  int64_t pposition;
  int16_t pspeed;
  int16_t ptorque;

  if (rx_msgs.buf[0] == 0x9C)	{
    // feedback speed, torque
    pspeed = (rx_msgs.buf[5]<<8)|(rx_msgs.buf[4]);
    ptorque = (rx_msgs.buf[3]<<8)|(rx_msgs.buf[2]);

    switch (rx_msgs.id) {
      case 0x141:  // joint a
        data->qd_a[i] = (pspeed/ratio[0]) * DEG_TO_RADIAN * side_a[i];
        data->tau_a[i] = ptorque * ratio[0] * side_a[i] / CURRENT_SCALING;
        break;
      case 0x142:  // joint b
        data->qd_b[i] = (pspeed/ratio[1]) * DEG_TO_RADIAN * side_b[i];
        data->tau_b[i] = ptorque * ratio[1] * side_b[i] / CURRENT_SCALING;
        break;
      case 0x143:  // joint c
        data->qd_c[i] = (pspeed/ratio[2]) * DEG_TO_RADIAN * side_c[i];
        data->tau_c[i] = ptorque * ratio[2] * side_c[i] / CURRENT_SCALING;
        break;
    }

  }	else if (rx_msgs.buf[0] == 0x92) {
    // feedback position
    pposition = ( rx_msgs.buf[1] | rx_msgs.buf[2] << 8| \
    rx_msgs.buf[3] << 16 | rx_msgs.buf[4] << 24 | \
    (uint64_t)rx_msgs.buf[5] << 32 |(uint64_t)rx_msgs.buf[6] << 40 | \
    (uint64_t)rx_msgs.buf[7] << 48 );

    switch (rx_msgs.id) {
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

//------------------------------------------------------------------------

// callback CAN 1
void canSniff_1(const CAN_message_t &res_msgs_1) {
  for (int i = 0; i < res_msgs_1.len; i++) {
    unpack_reply(res_msgs_1, &can_rx[0], 0);
  }
}

// callback CAN 2
void canSniff_2(const CAN_message_t &res_msgs_2) {
  for (int i = 0; i < res_msgs_2.len; i++) {
    unpack_reply(res_msgs_2, &can_rx[1], 1);
  }
}

// callback CAN 3
void canSniff_3(const CAN_message_t &res_msgs_3) {
  for (int i = 0; i < res_msgs_3.len; i++) {
    unpack_reply(res_msgs_3, &can_rx[2], 2);
  }
}

// init CAN BUS
void can_init() {
  // flush
  memset(&args_motor, 0, sizeof(struct motor_args));

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
      args_motor.req_msgs_pos[j].buf[n] = 0;	
    }
    args_motor.req_msgs_pos[j].buf[0] = 0x92;	
  }
  
  // 0x9C is the command for getting the velocity
  for (int j = 0; j < 3; j++) {
    args_motor.req_msgs_vel[j].id = (0x141 +j);
    args_motor.req_msgs_vel[j].len = 8;
    for (int n = 1; n < 8; n++) {
      args_motor.req_msgs_vel[j].buf[n] = 0;	
    }
    args_motor.req_msgs_vel[j].buf[0] = 0x9C;		
  }
}

//------------------------------------------------------------------------

// print error messages
void print_err(char action[], char info[]) {
  #ifdef PRINT_ERROR
  Serial.print("[MOTOR-CAN-TASK]: Fail to ");
  Serial.print(action); Serial.print(" "); Serial.print(info);
  Serial.println(" msgs, to exit task");
  #endif
}

// ESTOP if over angle limit
void angle_limit(struct motor_args *args_m) {
  #ifdef ESTOP
  for (int i = 0; i < 3; i++) {
		if ((args_m->joint_DATA.q_a[i]<min_a[i]) || (args_m->joint_DATA.q_a[i]>max_a[i]))  estop();
		if ((args_m->joint_DATA.q_b[i]<min_b[i]) || (args_m->joint_DATA.q_b[i]>max_b[i]))  estop();
		if ((args_m->joint_DATA.q_c[i]<min_c[i]) || (args_m->joint_DATA.q_c[i]>max_c[i]))  estop();
  }
  #endif
}

void print_data(struct motor_args *args_m) {
  #ifdef PRINT_DATA
  Serial.println("position, velocity, torque of first joint on each 3 CAN BUS");
  Serial.print(args_m->joint_DATA.q_a[0]); Serial.print(" / ");
  Serial.print(args_m->joint_DATA.q_a[1]); Serial.print(" / "); Serial.println(args_m->joint_DATA.q_a[2]);

  Serial.print(args_m->joint_DATA.qd_a[0]); Serial.print(" / ");
  Serial.print(args_m->joint_DATA.qd_a[1]); Serial.print(" / "); Serial.println(args_m->joint_DATA.qd_a[2]);

  Serial.print(args_m->joint_DATA.tau_a[0]); Serial.print(" / ");
  Serial.print(args_m->joint_DATA.tau_a[1]); Serial.print(" / "); Serial.println(args_m->joint_DATA.tau_a[2]); 
  #endif
}

void copy_data(struct motor_args *args_m) {
  // can 1, can 2, can 3
  for (int i = 0; i < 3; i++) {
    args_m->joint_DATA.q_a[i] = can_rx[i].q_a[i];
    args_m->joint_DATA.q_b[i] = can_rx[i].q_b[i];
    args_m->joint_DATA.q_c[i] = can_rx[i].q_c[i];

    args_m->joint_DATA.qd_a[i] = can_rx[i].qd_a[i];
    args_m->joint_DATA.qd_b[i] = can_rx[i].qd_b[i];
    args_m->joint_DATA.qd_c[i] = can_rx[i].qd_c[i];

    args_m->joint_DATA.tau_a[i] = can_rx[i].tau_a[i];
    args_m->joint_DATA.tau_b[i] = can_rx[i].tau_b[i];
    args_m->joint_DATA.tau_c[i] = can_rx[i].tau_c[i];    
  }
}

// main function
void task_fun(struct motor_args *args_m) {
  int err1, err2, err3;

  // send 0x92 requests on CANBUS
  for (int j = 0; j < 3; j++) {  // joint a, b, c
    err1 = can1.write(args_m->req_msgs_pos[j]);
    err2 = can2.write(args_m->req_msgs_pos[j]);
    err3 = can3.write(args_m->req_msgs_pos[j]);
    // if failed to send req
    if (err1!=1 && err2!=1 && err3!=1) {
      char a[5] = "send"; char b[9] = "position";
      print_err(a, b);
    }
  }

  // send 0x9C requests on CANBUS
  for (int j = 0; j < 3; j++) {  // joint a, b, c
    err1 = can1.write(args_m->req_msgs_vel[j]);
    err2 = can2.write(args_m->req_msgs_vel[j]);
    err3 = can3.write(args_m->req_msgs_vel[j]);
    // if failed to send req
    if (err1!=1 && err2!=1 && err3!=1) {
      char c[5] = "send"; char d[9] = "velocity";
      print_err(c, d);
    }
  }

  // read responses from callback functions
  copy_data(args_m);

  //------------------------------------------------------------------------
  // ESTOP if over angle limit
  angle_limit(args_m);

  // send torque command
  pack_torque_cmd(args_m);

  // print data before send command
  print_data(args_m);

  //------------------------------------------------------------------------
  // send msgs
  can1.write(args_m->setpoints_a[0]);
  can2.write(args_m->setpoints_a[1]);
  can3.write(args_m->setpoints_a[2]);

  can1.write(args_m->setpoints_b[0]);
  can2.write(args_m->setpoints_b[1]);
  can3.write(args_m->setpoints_b[2]);

  can1.write(args_m->setpoints_c[0]);
  can2.write(args_m->setpoints_c[1]);
  can3.write(args_m->setpoints_c[2]);
}

// run CAN BUS in main.cpp
void can_task() {
  task_fun(&args_motor);
}

// push received interrupt frames to the callbacks
void can_events() {
  can1.events();
  can2.events();
  can3.events();
}