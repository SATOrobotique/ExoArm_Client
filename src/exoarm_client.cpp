#include "exoarm_client.hpp"

ExoArmClient::ExoArmClient(){

}

int8_t ExoArmClient::init_tcp_client(const char* addr, int port){
    
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
  
    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, addr, &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return -1;
    }
  
    if ((status = connect(client_fd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
            
        printf("\nConnection Failed \n");
        return -1;
    }
    else{
        printf("Connected");
    }


    FD_ZERO(&input_fd);
    FD_SET(client_fd, &input_fd);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    return 0;
    
}

void ExoArmClient::set_robot_inputs(bool power_state,
                                    bool enable_state,
                                    float joint_position_ref[6],
                                    float joint_velocity_ref[6],
                                    float cartesian_stiffness_coeffs[6],
                                    float cartesian_damping_coeffs[6]){

    
    set_robot_power(power_state);
    set_robot_enable(enable_state);
    set_controller_inputs(joint_position_ref,
                          joint_velocity_ref,
                          cartesian_stiffness_coeffs,
                          cartesian_damping_coeffs);

}

void ExoArmClient::set_robot_power(bool power_state){
    
    protoClientToRobot.set_robot_power(power_state);

}

void ExoArmClient::set_robot_enable(bool enable_state){
    
    protoClientToRobot.set_robot_enable(enable_state);

}

void ExoArmClient::set_controller_inputs(float* joint_position_ref,
                                         float* joint_velocity_ref,
                                         float* cartesian_stiffness_coeffs,
                                         float* cartesian_damping_coeffs){

    RepeatedField<float>* p_joint_position_ref = protoClientToRobot.mutable_controller_inputs()->mutable_joint_position_ref();
    p_joint_position_ref->Assign(joint_position_ref, joint_position_ref+6);

    RepeatedField<float>* p_joint_velocity_ref = protoClientToRobot.mutable_controller_inputs()->mutable_joint_velocity_ref();
    p_joint_velocity_ref->Assign(joint_velocity_ref, joint_velocity_ref+6);

    RepeatedField<float>* p_cartesian_stiffness_coeffs = protoClientToRobot.mutable_controller_inputs()->mutable_cartesian_stiffness_coeffs();
    p_cartesian_stiffness_coeffs->Assign(cartesian_stiffness_coeffs, cartesian_stiffness_coeffs+6);

    RepeatedField<float>* p_cartesian_damping_coeffs = protoClientToRobot.mutable_controller_inputs()->mutable_cartesian_damping_coeffs();
    p_cartesian_damping_coeffs->Assign(cartesian_damping_coeffs, cartesian_damping_coeffs+6);

}


int8_t ExoArmClient::send_control_packet(){

    protoClientToRobot.set_header(protoClientToRobot.header() + 1);
    std::string output_str;
    protoClientToRobot.SerializeToString(&output_str);
    
    if(send(client_fd, output_str.c_str(), output_str.size(), 0) < 0){
        std::cout << "Send failed" << std::endl;
        return -1;
    }

    return 0;

}

int8_t ExoArmClient::receive_control_packet(){

    int n = select(client_fd+1, &input_fd, NULL, NULL, &timeout);
    if(n == 0){
        return -1;
    }
    
    valread = recv(client_fd, buffer, sizeof(buffer), 0);

    std::cout << valread << std::endl;
    
    std::string input_str = std::string(buffer, valread); 
    bool parsing_success = protoRobotToClient.ParseFromString(input_str);

    if(parsing_success == true){
        return 0;
    }
    else{
        return -1;
    }

}


void ExoArmClient::get_robot_outputs(bool* robot_state,
                                     bool* robot_active_fault,
                                     float* joint_position_fb,
                                     float* joint_velocity_fb){

    *robot_state = get_robot_state();
    *robot_active_fault = get_robot_active_fault();
    get_controller_outputs(joint_position_fb, joint_velocity_fb);

}

int32_t ExoArmClient::get_robot_state(){

    return protoRobotToClient.robot_state();

}

int32_t ExoArmClient::get_robot_active_fault(){

    return protoRobotToClient.fault_type();

}

void ExoArmClient::get_controller_outputs(float* joint_position_fb,
                                          float* joint_velocity_fb){

    std::copy(protoRobotToClient.controller_outputs().joint_position_fb().begin(),
            protoRobotToClient.controller_outputs().joint_position_fb().end(),
            joint_position_fb);

    std::copy(protoRobotToClient.controller_outputs().joint_velocity_fb().begin(),
              protoRobotToClient.controller_outputs().joint_velocity_fb().end(),
              joint_velocity_fb);
}