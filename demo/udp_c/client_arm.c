/*!
 * @file client_arm.c
 * @brief UDP client example - control 3/6/9 GYEMS-RMD motors
 * 
 * Author: Jia, Xinyu
 * Last modified: Jan 22, 2022
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <stdbool.h>
#include <sys/time.h>

#define PORT            8080
#define SERVER_ADDR     "192.168.137.177"

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

// 16 bytes
struct force_data {
  float f_r[4];
};

// ------------------------------------
// 108 bytes
struct low2high {
  struct joint_data _joint_data;
  // force_data _force_data;
};

// 48 bytes
struct high2low {
  struct joint_command _joint_cmd;
};

// data struct for passing data
struct udp_args {
  struct low2high msgs_data;
  struct high2low msgs_cmd;
};

// ------------------------------------
struct udp_args args_udp;
struct joint_command _canCommand;
struct joint_data _canData;

int sockfd;
struct sockaddr_in servaddr;
int servlen = sizeof(struct sockaddr_in);


// ----------------UDP-------------------
void init_udp_client() {
    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("[UDP-RT-TASK]: Failed to create socket");
        exit(EXIT_FAILURE);
    }

    // check and set buffer size
    int recv_size, send_size;

    // RX buffer
    uint32_t optLen1 = sizeof(recv_size);
    recv_size = sizeof(args_udp.msgs_data);
    setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&recv_size, optLen1);
    printf("[UDP-RT-TASK]: Recv Buffer length: %d\n", recv_size);  // 108

    // TX buffer
    uint32_t optLen2 = sizeof(send_size);
    send_size = sizeof(args_udp.msgs_cmd);
    setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (char *)&recv_size, optLen2);
    printf("[UDP-RT-TASK]: Send Buffer length: %d\n", send_size);  // 48

    memset(&servaddr, 0, sizeof(servaddr));
        
    // Filling server information
    servaddr.sin_family = AF_INET;  // IPv4
    servaddr.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    servaddr.sin_port = htons(PORT);
}

void udp_task(struct udp_args *args_u) {
    int err;

    // Send cmd and status to low-level controller PC104
    err = sendto(sockfd, &args_u->msgs_cmd, sizeof(args_u->msgs_cmd), MSG_CONFIRM, (const struct sockaddr *) &servaddr, servlen);

    // Receive data from motor & imu & joystick from PC104
    err = recvfrom(sockfd, &args_u->msgs_data, sizeof(args_u->msgs_data), MSG_DONTWAIT, (struct sockaddr *) &servaddr, (socklen_t*) &servlen);

    // [TO DO] add warnings
}

// run in main
void UDP_client_run() {
    udp_task(&args_udp);
}

// get udp rx msgs
struct low2high *recv_udp_data() {
    return &args_udp.msgs_data;
}

// get udp tx msgs
struct high2low *send_udp_cmd() {
    return &args_udp.msgs_cmd;
}


// ------------ Parameters -----------------
float Kp = 10;          // L5010-10T (9), L5015-10T (10), L7010-23T (19), L7015-10T (35)
float Kd = 1;           // L5010-10T (1), L5015-10T (1), L7010-23T (3), L7015-10T (3)
float Ki = 0.1;         // useless 
float tau_limit = 13.;  // N.m, L5010-10T (0.26x50), L5015-10T (0.38x50), L7010-23T (0.61x30), L7015-10T (1x30)

float dt = 0.002;       // designed control dt, 500Hz
int runTime = 5000;     // sec


// --- Position Control ---
int sgn (float val) {
    return (0.<val) - (val<0.);  // 1 for positive, 0 for 0, -1 for negative
}
float errors = 0.; 
float pd_control(float q_des, float q_data, float qd_des, float qd_data) {
    #if 1
    // --- PD ---
    float tau_des = Kp*(q_des - q_data) + Kd*(qd_des - qd_data);
    #else
    // --- PID ---
    errors += q_des - q_data;
    float tau_des = Kp*(q_des - q_data) + Kd*(qd_des - qd_data) + Ki*errors*dt;
    #endif

    return tau_des;
}


// ---------------- Main function ------------------
int main() {
    init_udp_client();
    memset(&_canCommand, 0, sizeof(struct joint_command));
    memset(&_canData, 0, sizeof(struct joint_data));

    // create a file
	if (!access("./actuator_data.txt", 0)) {
		if (!remove("./actuator_data.txt")) {
			printf("[Main]: Remove previous test results\n");
		}
	}
	FILE *fp = fopen("./actuator_data.txt", "a+");
	if (fp == NULL) {
		printf("File cannot open!\n");
		exit(0);
	}

    float q_data_A[3]; float qd_data_A[3]; float tau_data_A[3];  // joint a
    float q_data_B[3]; float qd_data_B[3]; float tau_data_B[3];  // joint b
    float q_data_C[3]; float qd_data_C[3]; float tau_data_C[3];  // joint c

    bool firstRun = true;   // get data before send command
    int n = runTime/dt;

    float meas_dt = 0.;     // measured dt
    struct timeval start_loop, finish_loop;  // timer

    for (int T = 0; T < n; T++) {
        gettimeofday(&start_loop, NULL);
        
        // get pointers
        struct high2low *udp_tx = send_udp_cmd();
        struct low2high *udp_rx = recv_udp_data();

        // ------------ compute --------------
        // flush command
        float q_des_A[3]; float qd_des_A[3]; float tau_des_A[3];
        float q_des_B[3]; float qd_des_B[3]; float tau_des_B[3];
        float q_des_C[3]; float qd_des_C[3]; float tau_des_C[3];

        // q_des = M_PI/6; qd_des = 0.;
        float q_des = (M_PI/2) * sin((2*M_PI/200000)*T);
        float qd_des = (M_PI/2) * (M_PI/100000) * cos((2*M_PI/200000)*T);

        for (int i = 0; i < 3; i++) {
            q_des_A[i] = q_des; qd_des_A[i] = qd_des;
            q_des_B[i] = q_des; qd_des_B[i] = qd_des;
            q_des_C[i] = q_des; qd_des_C[i] = qd_des;

            tau_des_A[i] = pd_control(q_des_A[i], q_data_A[i], qd_des_A[i], qd_data_A[i]);
            tau_des_B[i] = pd_control(q_des_B[i], q_data_B[i], qd_des_B[i], qd_data_B[i]);
            tau_des_C[i] = pd_control(q_des_C[i], q_data_C[i], qd_des_C[i], qd_data_C[i]);

            // clip torque for safety
            tau_des_A[i] = fminf(fmaxf(tau_des_A[i], -tau_limit), tau_limit);
            tau_des_B[i] = fminf(fmaxf(tau_des_B[i], -tau_limit), tau_limit);
            tau_des_C[i] = fminf(fmaxf(tau_des_C[i], -tau_limit), tau_limit);
        }

        if (firstRun) {
            for (int i = 0; i < 3; i++) {
                tau_des_A[i] = 0.; tau_des_B[i] = 0.; tau_des_C[i] = 0.;
            }
            firstRun = false;
        }


        // ------------input command--------------
        // CAN 1, joint 4/5/6; CAN 2 NA; CAN 3, joint 1/2/3
        for (int i = 0; i < 3; i++) {
            _canCommand.tau_a_des[i] = tau_des_A[i]; // joint 1/4
            _canCommand.tau_b_des[i] = tau_des_B[i]; // joint 2/5
            _canCommand.tau_c_des[i] = tau_des_C[i]; // joint 3/6
        }

        // ------------- transmit ----------------
        // copy command to UDP tx, run, receive all sensor data from UDP rx
        memcpy(&udp_tx->_joint_cmd, &_canCommand, sizeof(struct joint_command));
        UDP_client_run();
        memcpy(&_canData, &udp_rx->_joint_data, sizeof(struct joint_data));

        // ------------ output data --------------
        for (int i = 0; i < 3; i++) {
            q_data_A[i] = _canData.q_a[i]; qd_data_A[i] = _canData.qd_a[i]; tau_data_A[i] = _canData.tau_a[i];
            q_data_B[i] = _canData.q_b[i]; qd_data_B[i] = _canData.qd_b[i]; tau_data_B[i] = _canData.tau_b[i];
            q_data_C[i] = _canData.q_c[i]; qd_data_C[i] = _canData.qd_c[i]; tau_data_C[i] = _canData.tau_c[i];
        }


        // ------------ print result --------------
        #if 1
        // printf("[UDP-RT-TASK]: Send torque [%f]\n", tau_des);
        // printf("[UDP-RT-TASK]: Read position [%f], velocity [%f], torque [%f]\n", q_data, qd_data, tau_data);
		// #else
        int i = 2;  // CAN 1/2/3
        fprintf(fp, "%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", T, \
            q_des_A[i], q_data_A[i], qd_des_A[i], qd_data_A[i], tau_des_A[i], tau_data_A[i], \
            q_des_B[i], q_data_B[i], qd_des_B[i], qd_data_C[i], tau_des_B[i], tau_data_B[i], \
            q_des_C[i], q_data_C[i], qd_des_C[i], qd_data_C[i], tau_des_C[i], tau_data_C[i]);
        #endif


        // ---------- check frequency ------------
        int delay_time = dt * 1000000;  // microsecond, us
        // usleep(delay_time);

        gettimeofday(&finish_loop, NULL);
        int64_t exec_time_ms = (finish_loop.tv_sec-start_loop.tv_sec)*1000 + \
            (finish_loop.tv_usec-start_loop.tv_usec)/1000;  // millisecond, ms
        meas_dt = exec_time_ms/1000.0;  // second

        if (meas_dt > dt) {
            printf("[Main]: Too slow! Control frequency: [%.1f] Hz, Desired frequency: [%.1f] Hz\n", 1/meas_dt, 1/dt);
        } else {
            printf("[Main]: Each step time is [%f] sec\n", meas_dt);
        }
        printf("\n");
    }

    close(sockfd);
    return 0;
}
