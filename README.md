<div align="center">

# Caracal_Teensy
</div>

<div align="center">
<img width="100" src="doc/Teens4_1.jpg">
<img width="300" src="doc/can_bus.jpg">
</div>

## Introduction #
This repository for Teensy 4.1 functions as a hardware bridge between high-level controllers with Ethernet Ports and [GYEMS-RMD](http://www.gyems.cn/product.html) motors with CAN BUS Protocol. You can directly send torque commands to up to 9 motors and receive feedback data, such as position, velocity and actual torque. In the future, other sensor data will be also intergrated here in the same way.

* `1_Motor_CAN` : CAN BUS protocol of motors to write command and read data
* `2_Motor_CAN_UDP` : ~, UDP protocol to communicate with high-level controllers
* `3_Motor_CAN_UDP_RTOS` : ~, real-time system to schedule tasks
* `demo` : example codes of high-level control (C/C++, MATLAB, Python)
* `doc` : figures
* `tutorial` : Arduino offical tutorials

## Hardware #
* [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) - 600MHz, 8 serial, 3 SPI, 3 I2C, 3 CAN BUS, an Ethernet 10/100 Mbit

* Carrier Board - [Offical](https://copperhilltech.com/teensy-4-1-triple-can-bus-board-with-240x240-lcd-and-ethernet/) or Self-customized

## Dependencies #
* IDE - [PlatformIO](https://www.youtube.com/watch?v=JmvMvIphMnY)

* CAN BUS - [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4)

* UDP - [NativeEthernet](https://github.com/vjmuzik/NativeEthernet), [FNET](https://github.com/vjmuzik/FNET)

* RTOS - [FreeRTOS-Teensy4](https://github.com/juliandesvignes/FreeRTOS-Teensy4)

## How to run #
**1)** Clone this repository on your controller. Here we take a laptop with Ubuntu 18.04 as an example. In principle the code is independend of the environment, so both Windows and Linux should be OK (not yet been tested).

**2)** Connect all devices together.

**3)** Configure you laptop's local IP as the following recommended settings once the wired network is indetified.
```
IPv4 - Manual
Address: 192.168.137.178
Netmask: 255.0.0.0
```

**4)** Compile and run the high-level example code in `demo/udp_c` folder. 
```
sudo gcc client.c -o client
./client
```
For the permission error, run
```
cd ..
sudo chmod u+x ./udp_c -R
```
You can also find Matlab and Python version of the high-level code which fails to run due to inconsistent bytes in UDP. Welcome to help debug :-)

## CAN BUS
The code supports GYEMS-RMD motors with CAN ID `0x141`, `0x142`, `0x143` which means that you must configure the motor ID to 1, 2, 3 by GYEMS software `RMD V2.0.exe` before starting to control motors.

* The tx message `joint command` only includes designed torque information which takes up 48 bytes.
```
float tau_a_des[3];
float tau_b_des[3];
float tau_c_des[3];

int32_t flags[3];
// int32_t checksum;
```

* And the rx message `joint data` takes up 108 bytes.
```
float q_a[3];
float q_b[3];
float q_c[3];

float qd_a[3];
float qd_b[3];
float qd_c[3];

float tau_a[3];
float tau_b[3];
float tau_c[3];

// int32_t flags[3];
// int32_t checksum;
```
So set the send buffer to `TX_SIZE_32` (64 bytes) and the receive buffer to `RX_SIZE_64` (128 bytes).

## UDP #
The UDP rx message includes
```
joint command = 48 bytes
```
while the UDP tx message includes
```
joint data = 108 bytes
force data = xx
```
So set the receive buffer `RX_MAX_SIZE` to 48 bytes while `TX_MAX_SIZE` is 108 bytes currently. Similarly, in high-level control code, set UDP socket buffer `SO_SNDBUF` = 48 bytes, `SO_RCVBUF` = 108 bytes. 

## RTOS #
Currently only 2 threads are scheduled.
* UDP - priority 9; 1 kHz

* CAN BUS - priority 8; 2 kHz

## TO DO #
* Bit checks in UDP and CAN BUS

* Soft stop when an accident happens

* Intergrating other sensors - IMU, force sensor

* Remove or uncomment all `Serial.print` or `Serial.println` funtions since they affect threads running.
