#include "gyems.hpp"
/*****************************
 * Library for control of GYEMS motors 
 * V1.0 by Chang Hong - Dec 2020
 * ******************************/

Gyems::Gyems(uint8_t nodeId)
{
    // Initialize variables
    node_id = nodeId;
    position=0;
    multi_turn_position=0;
    voltage=0;
    torque=0;
    speed=0;
    phase_ai=0;
    phase_bi=0;
    phase_ci=0;
    temp=0;
    error_state=0;
    raw_position=0;
    encoderoffset=0;
    accel_data=0;
}

/*
uint8_t Gyems::getNodeId(){
    return node_id;
}
*/