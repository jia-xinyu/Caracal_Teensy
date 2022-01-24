/*
 * Ethernet/UDP server to communicate to a remote PC client where recommended settings are
 * remote IP - 192.168.137.178
 * remote Netmask - 255.0.0.0
 * 
 * Author: Jia, Xinyu
 * Last modified: Jan 24, 2022
*/

#include "UDP_PC.h"

// Define a MAC address and IP address for Teensy 4.1
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 137, 177);
unsigned int localPort = 8080;  // local port to listen on

EthernetUDP udp;  // an EthernetUDP instance

// buffers for receiving and sending data
char recv_buffer[RX_MAX_SIZE];
char send_buffer[];

struct udp_args args_udp;

// Get UDP rx pointer
high2low* recv_ethernet_command() {
 	return &args_udp.msgs_cmd;
}

// Get UDP tx pointer
low2high* send_ethernet_data() {
	return &args_udp.msgs_data;
}

//------------------------------------------------------------------------

// init UDP
void udp_init() {
  Ethernet.init(10);  // Most Arduino shields
  Ethernet.begin(mac, ip);

  // check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  udp.begin(localPort);
  Serial.println("Ethernet cable is connected.");
}

// main function
void server_task(struct udp_args *args_u) {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    #ifdef PRINT_SIZE
    Serial.print("Received packet of size "); Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port "); Serial.println(udp.remotePort());
    #endif

    // read the packet into packetBufffer
    udp.read(recv_buffer, RX_MAX_SIZE);

    // copy received command from UDP rx and copy all data to UDP tx
    memcpy(&args_u->msgs_cmd, &recv_buffer, sizeof(high2low));
    memcpy(&send_buffer, &args_u->msgs_data, sizeof(low2high));

    // send back the reply to the IP address and port
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(send_buffer);
    udp.endPacket();
}

// run UDP
void udp_task() {
  server_task(&args_udp);
}
