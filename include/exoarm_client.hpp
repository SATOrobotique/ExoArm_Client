#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include "robot_comm.pb.h"

using namespace google::protobuf;

class ExoArmClient{
    
    public:
        ExoArmClient();
        int8_t init();
        int8_t init_tcp_client(const char* ip, int port);

        void set_robot_power(bool power_state);
        void set_robot_enable(bool enable_state);
        void set_controller_inputs(float joint_position_ref[6],
                                   float joint_velocity_ref[6],
                                   float cartesian_stiffness_coeffs[6],
                                   float cartesian_damping_coeffs[6]);
        void set_robot_inputs(bool power_state,
                              bool enable_state,
                              float joint_position_ref[6],
                              float joint_velocity_ref[6],
                              float cartesian_stiffness_coeffs[6],
                              float cartesian_damping_coeffs[6]);

        int8_t send_control_packet();

        int8_t receive_control_packet();


        int32_t get_robot_state();
        int32_t get_robot_active_fault();
        void get_controller_outputs(float* joint_position_fb,
                                    float* joint_velocity_fb);

        void get_robot_outputs(bool* robot_state,
                               bool* robot_active_fault,
                               float* joint_position_fb,
                               float* joint_velocity_fb);




    private:

        ClientToRobot protoClientToRobot;
        RobotToClient protoRobotToClient;

        int status, valread, client_fd;
        struct sockaddr_in serv_addr;
        
        char buffer[256] = { 0 };

        pthread_t reading_thread;

        fd_set input_fd;
        struct timeval timeout;


};