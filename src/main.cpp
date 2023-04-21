#include "exoarm_client.hpp"
#include <stdio.h>
#include <chrono>
#include <thread>

ExoArmClient exoarm_client;
std::chrono::_V2::system_clock::time_point previous;

void signal_handler(int signum){
    exoarm_client.disconnect_tcp();
    exit(1);
}


void send_cmd(){
    
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( now - previous ).count();
    
    //std::cout << duration<< std::endl;
    exoarm_client.send_success = exoarm_client.send_cmd_packet();
    //std::cout << "Send success: " << static_cast<int16_t>(exoarm_client.send_success) << std::endl;
    previous = now;

}

void timer_start(std::function<void(void)> func, unsigned int interval)
{
  std::thread([func, interval]()
  { 
    while (true)
    { 
      auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
      func();
      std::this_thread::sleep_until(x);
    }
  }).detach();
}

int main(int argc, char *argv[]){

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

    if(argc < 4){
        std::cout << "Missing arguments" << std::endl;
        return -1;
    }
    
    char* server_addr = argv[1];
    int server_port = atoi(argv[2]);
    
/*     std::cout << static_cast <const void *> (server_addr) << std::endl;
    std::cout << server_port << std::endl; */
    
    if(exoarm_client.init_tcp_client(server_addr, server_port) < 0){
        return -1;
    };

    float joint_position_ref[] = {std::stof(argv[5]),1,1,1,1,1};
    float joint_velocity_ref[] = {1,1,1,1,1,1};
    float cartesian_stiffness_coeffs[] = {1,1,1,1,1,1};
    float cartesian_damping_coeffs[] = {1,1,1,1,1,1};

    exoarm_client.set_robot_power(atoi(argv[3]));
    exoarm_client.set_robot_enable(atoi(argv[4]));
    exoarm_client.set_controller_inputs(joint_position_ref,
                                        joint_velocity_ref,
                                        cartesian_stiffness_coeffs,
                                        cartesian_damping_coeffs);

    int8_t send_success, receive_success;
    
    std::thread listening_thread(&ExoArmClient::listen, &exoarm_client);
    
    timer_start(send_cmd, 1);

    listening_thread.join();
/*     while(1){

        send_success = exoarm_client.send_cmd_packet();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    } */

}

