#include "exoarm_client.hpp"
#include <stdio.h>
#include <chrono>
#include <thread>


int main(int argc, char *argv[]){

    if(argc < 3){
        std::cout << "Missing arguments" << std::endl;
        return -1;
    }
    
    char* server_addr = argv[1];
    int server_port = atoi(argv[2]);
    
    std::cout << static_cast <const void *> (server_addr) << std::endl;
    std::cout << server_port << std::endl;

    ExoArmClient exoarm_client;
    
    if(exoarm_client.init_tcp_client(server_addr, server_port) < 0){
        return -1;
    };

    float joint_position_ref[] = {1,2,3,4,5,6};
    float joint_velocity_ref[] = {1,2,3,4,5,6};
    float cartesian_stiffness_coeffs[] = {1,2,3,4,5,6};
    float cartesian_damping_coeffs[] = {1,2,3,4,5,6};

    exoarm_client.set_robot_power(1);
    exoarm_client.set_robot_enable(0);
    exoarm_client.set_controller_inputs(joint_position_ref,
                                        joint_velocity_ref,
                                        cartesian_stiffness_coeffs,
                                        cartesian_damping_coeffs);

    int8_t send_success, receive_success;
    
    while(1){

        send_success = exoarm_client.send_control_packet();
        std::cout << "Send success: " << static_cast<int16_t>(send_success) << std::endl;

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        receive_success = exoarm_client.receive_control_packet();
        std::cout << "Receive success: " << static_cast<int16_t>(receive_success) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

}