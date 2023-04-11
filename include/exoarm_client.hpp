#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "robot_comm.pb.h"

#define PORT 8080


class ExoArmClient{
    
    public:
        ExoArmClient();
        int8_t init();
        int8_t init_tcp_client();

        int8_t set_robot_power();
        int8_t set_robot_enable();
        int8_t set_controller_commands();
        int8_t reset_robot_faults();

        int8_t get_robot_state();
        int8_t get_robot_active_fault();
        int8_t get_controller_feedback();

    private:

        ClientToRobot protoClientToRobot;
        RobotToClient protoRobotToClient;

        int status, valread, client_fd;
        struct sockaddr_in serv_addr;
        char* hello = "Hello from client";
        char buffer[1024] = { 0 };


};