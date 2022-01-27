#include <FreeRTOS_TEENSY4.h>
#include "UDP_PC.hpp"
#include "RMD_Motor.hpp"

// declare a semaphore handle.
SemaphoreHandle_t sem;

joint_command _canCommand;
joint_data _canData;
// force_data _forceData;

static void runUDP(void* arg) {
  while (1) {
    // take signal from thread CAN BUS
    xSemaphoreTake(sem, portMAX_DELAY);

    // get pointers from UDP_PC.c
    low2high *udp_tx = send_udp_data();
    high2low *udp_rx = recv_udp_command();

    // copy all sensor data to UDP tx
    memcpy(&udp_tx->_joint_data, &_canData, sizeof(joint_data));
    // memcpy(&udp_tx->_force_data, &_forceData, sizeof(force_data));

    udp_task();

    // recevive command from UDP rx
    memcpy(&_canCommand, &udp_rx->_joint_cmd, sizeof(joint_command));

    // 1 kHz
    vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
  }
}

static void runCANBUS(void* arg) {
  while (1) {
    // get pointers from RMD_Motor.c
    joint_command *cmd = get_can_command();
    joint_data *data = get_can_data();

    // copy command to CAN BUS, compute, send data to UDP
    memcpy(cmd, &_canCommand, sizeof(joint_command));
    can_task();
    memcpy(&_canData, data, sizeof(joint_data));

    // give signal to thread UDP
    xSemaphoreGive(sem);

    // 2 kHz
    vTaskDelay((1L * configTICK_RATE_HZ) / 2000L);
  }
}

// static void runForce(void* arg) {
//   while (1) {
//     // memcpy(&_forceData, &data, sizeof(force_data));
//   }
// }


void setup() {
  // initialize LED digital pin as an output and turn the LED on
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);

  // start serial USB for monitor
  Serial.begin(9600); delay(400);

  // create 2 threads and initialize semaphore
  portBASE_TYPE s1, s2;
  sem = xSemaphoreCreateCounting(1, 0);

  //------------------------------------------------------------------------
  // start UDP
  udp_init(); delay(100);
  // start 3 CAN BUS
  can_init(); delay(100);

  //------------------------------------------------------------------------
  // clear dirty command & data before sending
  memset(&_canCommand, 0, sizeof(joint_command));
  memset(&_canData, 0, sizeof(joint_data));
  // memset(&_forceData, 0, sizeof(force_data));

  //------------------------------------------------------------------------
  // create task priorities, 0~9, high numbers denote high priority
  // assigning the same priority will cause 2nd thread 2 to fail (TO DO)
  // configMINIMAL_STACK_SIZE = 90 uin16_t, 180bytes, used by idle task
  s1 = xTaskCreate(runUDP, NULL, configMINIMAL_STACK_SIZE, NULL, 9, NULL);
  s2 = xTaskCreate(runCANBUS, NULL, configMINIMAL_STACK_SIZE, NULL, 8, NULL);

  // check for creation errors
  if (sem == NULL || s1 != pdPASS || s2 != pdPASS ) {
    Serial.println("Creation problem");
    while(1);  // pause
  }

  // start scheduler
  Serial.println("Starting the scheduler !");
  vTaskStartScheduler();

  // will not get here unless there is insufficient RAM
  Serial.println("Insufficient RAM");
  while(1);
}


void loop() {
  can_events();
}