#ifndef JOINT_MESSAGE_H
#define JOINT_MESSAGE_H

#include <stdint.h>

// 60 bytes
struct joint_command {
    float q_des_a[3];
    float q_des_b[3];
    float q_des_c[3];

    float qd_des_a[3];
    float qd_des_b[3];
    float qd_des_c[3];

    float kp_a[3];
    float kp_b[3];
    float kp_c[3];

    float kd_a[3];
    float kd_b[3];
    float kd_c[3];

    float tau_a_ff[3];
    float tau_b_ff[3];
    float tau_c_ff[3];

    int32_t flags[3];
    // int32_t checksum;
};

// 132 bytes
struct joint_data {
    float q_a[3];
    float q_b[3];
    float q_c[3];

    float qd_a[3];
    float qd_b[3];
    float qd_c[3];

    int32_t flags[3];
    // int32_t checksum;
};

#endif