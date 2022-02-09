// Client side implementation of UDP client-server model
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

// ------------------------------------
// 16 bytes
struct force_data {
  float f_r[4];
};

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

// ------------control-----------------
float m = 1;            // kg
float g = 9.81;         // m/s^2
float l = 0.25;         // m
float Kp = 3;
float Kd = 0;
float tau_limit = 10.;   // N.m, L5010-10T (0.26), L7015-10T (1x30)
float columb_fric = 2.5;  // Columb Friction, N.m
float k = 1.;           // energy shaping
float b = 0.1;          // energy shaping

// === Method 1 ===
float gravity_compensation(float q_data) {
    return m * g * l* sin(q_data);
}

// === Method 2 ===
int sgn (float val) {
    return (0.<val) - (val<0.);  // 1 for positive, 0 for 0, -1 for negative
}
float pd_control(float q_des, float q_data, float qd_des, float qd_data) {
    float tau_des = Kp*(q_des - q_data) + Kd*(qd_des - qd_data) + sgn(qd_data)*columb_fric;
    // float tau_des = Kp*(q_des - q_data) + Kd*(qd_des - qd_data);
    return tau_des;
}

// === Method 3 ===
float total_energy(float position, float velocity) {
    return 0.5*m*pow((l*velocity),2) + m*g*l*(1.0-cos(position));  // kinetic + potential
}
float energy_shaping(float pos, float vel) {
    float current_energy = total_energy(pos, vel);
    float des_energy = total_energy(M_PI, 0.);  // push the system towards the upright position

    float tau_des = 0.;
    if (pos==0. && vel==0.) {
        tau_des = 0.1 * tau_limit;  // start torque
    } else {
        tau_des = -k*vel*(current_energy - des_energy) + b*vel;
    }
    return tau_des;
}

// ----------------Main function------------------
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

    float q_data = 0.; float qd_data = 0.; float tau_data = 0.;
    bool firstRun = true;   // get data before send command
    float dt = 0.002;       // designed control dt, 500Hz
    int runTime = 100000;       // sec
    int n = runTime/dt;

    float meas_dt = 0.;     // measured dt
    struct timeval start_loop, finish_loop;  // timer

    for (int i = 0; i < n; i++) {
        gettimeofday(&start_loop, NULL);
        
        // get pointers
        struct high2low *udp_tx = send_udp_cmd();
        struct low2high *udp_rx = recv_udp_data();

        // ------------input command--------------
        float q_des = 0.; float qd_des = 0.; float tau_des = 0.;  // flush command
        switch (2) {
            case 0:
                tau_des = 2.5;  // measure Columb friction
                break;
            case 1:
                tau_des = gravity_compensation(q_data);
                break;
            case 2:
                // q_des = (M_PI/2) * sin((2*M_PI/2)*i);
                q_des = 0;
                qd_des = 0.;
                tau_des = pd_control(q_des, q_data, qd_des, qd_data);
                // tau_des += gravity_compensation(q_data);
                break;
            case 3:
                // switch to position control to keep at the unstable position
                if (fabs(q_data) < (M_PI-0.2)) {
                    q_des = q_data; qd_des = 0.;
                    tau_des = pd_control(q_des, q_data, qd_des, qd_data);
                    tau_des += gravity_compensation(q_data);
                } else {
                    tau_des = energy_shaping(q_data, qd_data);
                }
                break;
        }
        if (firstRun) {
            tau_des = 0.;
            firstRun = false;
        }
        tau_des = fminf(fmaxf(tau_des, -tau_limit), tau_limit);  // clip torque for safety

        // CAN 1, joint A
        _canCommand.tau_a_des[0] = tau_des;


        // -------------transmit----------------
        // copy command to UDP tx, run, receive all sensor data from UDP rx
        memcpy(&udp_tx->_joint_cmd, &_canCommand, sizeof(struct joint_command));
        UDP_client_run();
        memcpy(&_canData, &udp_rx->_joint_data, sizeof(struct joint_data));


        // ------------output data--------------
        q_data = _canData.q_a[0];
        qd_data = _canData.qd_a[0];
        tau_data = _canData.tau_a[0];

        // ------------print result--------------
        #if 1
        printf("[UDP-RT-TASK]: Send torque [%f]\n", tau_des);
        printf("[UDP-RT-TASK]: Read position [%f], velocity [%f], torque [%f]\n", q_data, qd_data, tau_data);
		// #else
        fprintf(fp, "%d %.3f %.3f %.3f %.3f %.3f %.3f \n", i, \
			q_des, q_data, qd_des, q_data, tau_des, tau_data);
		#endif


        // ----------check frequency------------
        i += 1;
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
