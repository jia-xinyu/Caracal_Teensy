#ifndef _UDP_PC_H
#define _UDP_PC_H

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "joint_message.hpp"

#define RX_MAX_SIZE 48  // Bytes
#define TX_MAX_SIZE 108  // Bytes
// #define PRINT_SIZE

// 16 bytes
struct force_data {
  float f_r[4];
};

// 108 bytes
struct low2high {
  joint_data _joint_data;
  // force_data _force_data;
};

// 48 bytes
struct high2low {
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