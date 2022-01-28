<div align="center">
# Caracal_Teensy
</div>

## Introduction #
This repository is a hardware bridge between controllers with Ethernet Ports and [GYEMS-RMD](http://www.gyems.cn/product.html) motors with CAN BUS Protocol. You can directly send torque commands to up to 9 motors and receive feedback data, such as position, velocity and actual torque.
In the future, other sensor data will be also intergrated here in the same way.

<div align="center">
<img width="400" src="doc/Teens4_1.gif">
<img width="400" src="doc/trot_out.gif">
</div>

## Hardware #
* [Teensy 4.1](https://www.pjrc.com/store/teensy41.html): 600MHz, 8 serial, 3 SPI, 3 I2C, 3 CAN Bus, an Ethernet 10/100 Mbit

* Carrier Board: [Offical](https://copperhilltech.com/teensy-4-1-triple-can-bus-board-with-240x240-lcd-and-ethernet/) or Self-customized

## Dependencies #
* IDE: [PlatformIO](https://www.youtube.com/watch?v=JmvMvIphMnY)

* CAN BUS: [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4)

* UDP: [NativeEthernet](https://github.com/vjmuzik/NativeEthernet), [FNET](https://github.com/vjmuzik/FNET)

* RTOS: [FreeRTOS](https://github.com/juliandesvignes/FreeRTOS-Teensy4)

===================================================================

## Build #
Download code in path `~/Documents` and run `make_types.sh` if you use lcm-spy to monitor data in real time:
```
git clone https://github.com/Jarvis861/NUS-quadruped-high
cd NUS-quadruped-high/scripts
sudo ./make_types.sh
cd ..
mkdir build
```

Recommend to add shortcut keys in your `.bash_aliases` file:
```
gedit ~/.bash_aliases
```
and copy following lines to `.bash_aliases`:
```
## CAN BUS
Joint (bytes): joint command - 48, joint data - 108

CAN Buffer (bytes): TX_SIZE_32 - 64, RX_SIZE_64 - 128

## UDP #
Before run this high-level controller you must:
* check if the Ethernet wire is connected well
* run the low-level controller first

UDP messages (bytes): joint command - 48, joint data - 108, force data - xx

UDP Buffer (bytes): RX_MAX_SIZE - 48, (TX_MAX_SIZE - 124)

## RTOS #

## TO DO #
