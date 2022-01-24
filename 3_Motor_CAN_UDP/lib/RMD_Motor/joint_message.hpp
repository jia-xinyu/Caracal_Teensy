#ifndef JOINT_MESSAGE_H
#define JOINT_MESSAGE_H

#include <stdint.h>

// 48 bytes = 24 uint16_t
struct joint_command {
    float tau_a_des[3];
    float tau_b_des[3];
    float tau_c_des[3];

    int32_t flags[3];
    // int32_t checksum;
};

// 108 bytes = 54 uint16_t
struct joint_data {
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
};

#endif