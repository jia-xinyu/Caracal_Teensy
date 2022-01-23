#ifndef JOINT_MESSAGE_H
#define JOINT_MESSAGE_H

#include <stdint.h>

// 60 bytes
struct joint_command {
    float      q_des_a[2];
    float      q_des_b[2];
    float      q_des_c[2];

    float      qd_des_a[2];
    float      qd_des_b[2];
    float      qd_des_c[2];

    float      kp_a[2];
    float      kp_b[2];
    float      kp_c[2];

    float      kd_a[2];
    float      kd_b[2];
    float      kd_c[2];

    float      tau_a_ff[2];
    float      tau_b_ff[2];
    float      tau_c_ff[2];

    int32_t    flags[2];
    // int32_t checksum;
};

// 132 bytes
struct joint_data {
    float q_a[2];
    float q_b[2];
    float q_c[2];

    float qd_a[2];
    float qd_b[2];
    float qd_c[2];

    int32_t flags[2];
    // int32_t checksum;
};

#endif