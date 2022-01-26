#include "UDP_PC.hpp"
#include "RMD_Motor.hpp"

void setup() {
  // initialize LED digital pin as an output and turn the LED on
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);

  // start serial USB for monitor
  Serial.begin(9600); delay(400);

  //------------------------------------------------------------------------
  // start UDP
  udp_init(); delay(100);
  // start 3 CAN BUS
  can_init(); delay(100);

  //------------------------------------------------------------------------
  // clear dirty command & data before sending
  // memset(&_canCommand, 0, sizeof(joint_command));
  // memset(&_canData, 0, sizeof(joint_data));
  // memset(&_forceData, 0, sizeof(force_data));
}


void loop() {
    can_events();

    // get pointers from UDP_PC.c
    low2high *udp_tx = send_udp_data();
    high2low *udp_rx = recv_udp_command();

    // get pointers from RMD_Motor.c
    joint_command *cmd = get_can_command();
    joint_data *data = get_can_data();

    // copy all sensor data to UDP tx
    // memcpy(&udp_tx->_force_data, &_forceData, sizeof(force_data));

    // udp_recv();

    // udp_rx->_joint_cmd.tau_a_des[0] = 50;

    // copy command from UDP rx to CAN BUS
    memcpy(cmd, &udp_rx->_joint_cmd, sizeof(joint_command));
    // compute
    can_task();
    // copy data from CAN BUS to UDP tx
    memcpy(&udp_tx->_joint_data, data, sizeof(joint_data));

    Serial.println(data->q_a[0]);

    udp_send();
}