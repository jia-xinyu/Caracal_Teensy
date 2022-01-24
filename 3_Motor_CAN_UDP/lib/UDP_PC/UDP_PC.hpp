#ifndef _UDP_PC_H
#define _UDP_PC_H

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "joint_message.hpp"

#define RX_MAX_SIZE 48  // 24
#define TX_MAX_SIZE 150  // 24
#define PRINT_SIZE

struct imu_data {
  float accel[3];
  float gyro[3];  // x y z
  float quat[4];  // i,j,k,real
};

struct low2high {
  joint_data _joint_data;
  // imu_data imu_data;
  // force_data _force_data;
};

struct high2low {
  int32_t status; //check ethernet connection
  joint_command _joint_cmd;
};

// data struct for passing data
struct udp_args {
  struct low2high msgs_data;
  struct high2low msgs_cmd;
};

void udp_init();
void udp_task();
high2low *recv_udp_command();
low2high *send_udp_data();

#endif