/*
 * Test GYEMS RMD motor
 * Read position; CAN ID - 0x141
*/

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can3;

CAN_message_t msg_pos1;
CAN_message_t msg_pos2;
CAN_message_t msg_pos3;

void canSniff(const CAN_message_t &msg) { // global callback
  Serial.println("motor position");
  
  Serial.print(millis()); 
  Serial.print(", msgs length - "); Serial.print(msg.len); 
  Serial.print(", CAN ID - 0x"); Serial.println(msg.id, HEX);

  for (int i = 0; i < msg.len; i++) {
    Serial.print(msg.buf[i], HEX);
    Serial.print(", ");
  }
  Serial.println("\n");
}


void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("Teensy 4.1 CAN 1 test");

  // LED
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH);

  can1.begin();
  can1.setBaudRate(1000000);  // 1Mbps
  can1.setMaxMB(16);  // up to 64 mailbox on Teensy 4
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();

  can2.begin();
  can2.setBaudRate(1000000);  // 1Mbps
  can2.setMaxMB(16);  // up to 64 mailbox on Teensy 4
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(canSniff);
  can2.mailboxStatus();

  can3.begin();
  can3.setBaudRate(1000000);  // 1Mbps
  can3.setMaxMB(16);  // up to 64 mailbox on Teensy 4
  can3.enableFIFO();
  can3.enableFIFOInterrupt();
  can3.onReceive(canSniff);
  can3.mailboxStatus();
}


void loop() {
  can1.events();
  can2.events();
  can3.events();

  msg_pos1.id = 0x141;  // motor ID = 1
  msg_pos1.len = 8;
  msg_pos1.buf[0] = 0x9C; 
  for(int i = 1;i < 8; i++){
    msg_pos1.buf[i]=0x00;
  }
  
  msg_pos2.id = 0x142;  // motor ID = 2
  msg_pos2.len = 8;
  msg_pos2.buf[0] = 0x9C; 
  for(int i = 1;i < 8; i++){
    msg_pos2.buf[i]=0x00;
  }

  msg_pos3.id = 0x143;  // motor ID = 3
  msg_pos3.len = 8;
  msg_pos3.buf[0] = 0x9C; 
  for(int i = 1;i < 8; i++){
    msg_pos3.buf[i]=0x00;
  }

  // must wait 2 us otherwise RX cannot work
  can1.write(msg_pos1); delay(0.002); 
  can1.write(msg_pos2); delay(0.002); 
  can1.write(msg_pos3); delay(0.002); 

  can2.write(msg_pos1); delay(0.002); 
  can2.write(msg_pos2); delay(0.002); 
  can2.write(msg_pos3); delay(0.002); 

  can3.write(msg_pos1); delay(0.002); 
  can3.write(msg_pos2); delay(0.002); 
  can3.write(msg_pos3); delay(0.002); 
}