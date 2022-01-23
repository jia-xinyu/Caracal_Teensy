#include "UDP_PC.h"
#include "RMD_Motor.h"

void setup() {
  // initialize LED digital pin as an output and turn the LED on
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);

  // start serial USB for monitor
  Serial.begin(9600); delay(400);

  // start the Ethernet
  udp_init();
  
  // start 3 CAN BUS
  can_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  can1.events();
  can2.events();
  can3.events();

  can_task(&args_motor);
}