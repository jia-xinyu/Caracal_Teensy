// Client side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

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

// ------------------------------------
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

// ------------------------------------
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

// ------------------------------------
int main() {
    init_udp_client();

    memset(&_canCommand, 0, sizeof(struct joint_command));
    memset(&_canData, 0, sizeof(struct joint_data));
    int flag;

    while (1) {
        // get pointers
        struct high2low *udp_tx = send_udp_cmd();
        struct low2high *udp_rx = recv_udp_data();

        // ------------input command--------------
        float torque  = 1;
        _canCommand.tau_a_des[0] = torque;

        // copy command to UDP tx
        memcpy(&udp_tx->_joint_cmd, &_canCommand, sizeof(struct joint_command));
        
        // run UDP
        UDP_client_run();
        
        // receive all sensor data from UDP rx
        memcpy(&_canData, &udp_rx->_joint_data, sizeof(struct joint_data));

        // ------------output data--------------
        float position = _canData.q_a[0];
        printf("[UDP-RT-TASK]: Read position joint a [%f]\n", position); 
    }

    close(sockfd);

    return 0;
}
