/*
 * Ethernet/UDP Communication to PC
 * PC info: 

 * Author: Jia, Xinyu
 * Last modified: Jan 22, 2022
*/

#include "UDP_PC.h"

// Enter a MAC address and IP address for your Teensy 4.1.
// The IP address will be dependent on your local network:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 137, 177);
unsigned int localPort = 8080;  // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udp;

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

void udp_task() {}

