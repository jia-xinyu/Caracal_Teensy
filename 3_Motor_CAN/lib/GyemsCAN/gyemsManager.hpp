#ifndef _GYEMS_CAN_H_
#define _GYEMS_CAN_H_

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>
#include <array>

#include "gyems.hpp"

#define MAX_GYEMS_REQUESTS 12
#define MAX_NO_OF_GYEMS_DEVICES 12
//MAX_READ_BUFFER = MAX_NO_OF_GYEMS_DEVICES * 4
#define MAX_READ_BUFFER 48 
#define CANT4 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>
using std::vector;

// Macros to construct can ids.
#define GET_GYEMSCANID(node_id_) (0x140 | (node_id_ ))

//#define DEBUG 

class GyemsManager {
public:
    // Constructor
    GyemsManager(vector<CAN_message_t> &msgArray, vector<Gyems> &motorArray, CANT4& can_chn): 
    gyems_msg_array(msgArray), gyems_array(motorArray), channel_name(can_chn){}
    int begin();
    int isGyemsId(int canId);
    int getsize();
    void create_lookup();
    int getmotorindex(uint32_t received_node_id);

    int load_periodic_request(uint32_t desired_values, uint32_t node_id);
    int request_values(uint16_t delay);
    int read_values() ;
    int parse_can_frame_and_update_signals(CAN_message_t &frame);

    void pack_torque_cmd(int nodeID, int16_t iqControl);
    void pack_speed_cmd(int nodeID, int32_t speed);
    void unpack_speed_torque_reply(CAN_message_t* msg, uint8_t* temp, int16_t* ptorque,int16_t* pspeed,uint16_t* pposition);
    void pack_off_cmd(int nodeID);
    void pack_stop_cmd(int nodeID);
    void pack_run_cmd(int nodeID);
    void clear_motor_error(int nodeID);
    void read_status_1_error(int nodeID);
    void status_1_reply(CAN_message_t* msg, uint8_t* temp, int16_t* pvoltage,uint8_t* perror_state);
    void read_status_2_data(int nodeID);
    void read_status_3_phase_current(int nodeID);
    void status_3_reply(CAN_message_t* msg, int16_t* pca,int16_t* pcb,int16_t* pcc);
    void pack_multi_torque_cmd(int nodeID, int16_t iqControl, int16_t iqControl2, int16_t iqControl3,int16_t iqControl4);
    void pack_position_1_cmd(int nodeID, int32_t angle);
    void pack_position_2_cmd(int nodeID, int32_t angle, uint16_t max_speed);
    void pack_position_3_cmd(int nodeID, uint16_t angle, uint8_t spinDirection);
    void pack_position_4_cmd(int nodeID, uint16_t angle, uint16_t max_speed, uint8_t spinDirection);
    void read_single_turn_angle(int nodeID);
    void unpack_single_turn_angle(CAN_message_t* msg, uint16_t* pangle);
    void read_multi_turn_angle(int nodeID);
    void unpack_multi_turn_angle(CAN_message_t* msg, int64_t* mangle);
    void set_pos2zero(int nodeID);      
    void write_encoder_offset(int nodeID,uint16_t offset);          
    void read_encoder(int nodeID);      
    void unpack_read_encoder(CAN_message_t* msg, uint16_t* pos, uint16_t* raw, uint16_t* offset);
    
    void write_acceleration2ram(int nodeID,uint16_t accel);
    void read_acceleration(int nodeID);
    void unpack_read_acceleration(CAN_message_t* msg, int32_t* paccel);
    void write_PID(int nodeID,PIDconstant pid);
    void read_PID(int nodeID);
    void unpack_read_PID(CAN_message_t* msg, PIDconstant* pid);
 
    vector<CAN_message_t>& gyems_msg_array;

private:
    // Some private functions
    vector<Gyems>& gyems_array;
    CANT4& channel_name;
    
    int no_of_motor;
    Gyems* imu_ptr;
    uint32_t lookup[MAX_NO_OF_GYEMS_DEVICES];
    
    uint32_t requests[MAX_GYEMS_REQUESTS][2];
    int requestlen=0;
    CAN_message_t tx_buf,rx_buf;

};

 
 


#endif