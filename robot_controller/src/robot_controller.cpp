#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "force_srv_file/srv/add_force_vectors.hpp"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <chrono>
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>
#include <ctime>
using namespace std::chrono_literals;

std::ofstream Plots;
class RobotController : public rclcpp::Node 
{
public:
    RobotController() : Node("effort_controller_node") 
    {
        //Created a publisher to the topic /forward_effort_controller/command with queue size 10
        publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);       

        //Created a server with service /input_angles. Client will be through the terminal    
        server = this->create_service<force_srv_file::srv::AddForceVectors>("/input_angles",
        std::bind(&RobotController::InputCallback, this, std::placeholders::_1, std::placeholders::_2));

        //Created a subscriber to /joint_states topic to get position and velocity at runtime
        subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
                                        std::bind(&RobotController::JointStateSub, this, std::placeholders::_1));


    }
     
private:
    void JointStateSub(const sensor_msgs::msg::JointState value)
    {
        array1_[0] = value.position[0];
        array1_[1] = value.position[1];
        array1_[2] = value.position[2];
        array1_[3] = value.velocity[0];
        array1_[4] = value.velocity[1];
        array1_[5] = value.velocity[2];

        double error1 = v[0] - array1_[0];
        double error1_dot = 0 - array1_[3];
        
        double error2 = v[1] - array1_[1];
        double error2_dot = 0 - array1_[4];        
        
        double error3 = v[2] - array1_[2];
        double error3_dot = 0 - array1_[5];
        std::cout<<"error3="<<error3<<std::endl;
        std::cout<<"error3_dot="<<error3_dot<<std::endl;
        double u1 = k_p_1*(error1) + k_d_1*(error1_dot);
        double u2 = k_p_2*(error2) + k_d_2*(error2_dot);
        double u3 = 9.81 + k_p_3*(error3) + k_d_3*(error3_dot);
        
        //Creating time variable of milliseconds/10
        int time = value.header.stamp.sec*100 + value.header.stamp.nanosec/10000000;
        
        //Creating a CSV file to store the desired, runtime positions and the time.
        Plots.open ("f_plots.csv",std::ios::app);
		Plots <<v[0]<< "," <<v[1]<< "," <<v[2]<< "," <<array1_[0]<< "," <<array1_[1]<< "," <<array1_[2]<< "," <<time<<"\n";
    	Plots.close();
          		

        //Storing the value in the variable effort and publishing the data variable from effort.
        auto effort = std_msgs::msg::Float64MultiArray();
        effort.data = {u1, u2, u3};
        publisher->publish(effort);     



    }

    void InputCallback(const force_srv_file::srv::AddForceVectors::Request::SharedPtr request,
                       force_srv_file::srv::AddForceVectors::Response::SharedPtr response)
    {
        v[0] = request->v1[0];
        v[1] = request->v1[1];
        v[2] = request->v1[2];

        response->v2[0] = u1;
        response->v2[1] = u2;
        response->v2[2] = u3;

    }
    rclcpp::Service<force_srv_file::srv::AddForceVectors>::SharedPtr server;    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;
    double array1_[6]; //Global array that gets updated in subscriber.
    double v[3]; //Array to store request.
    double u1,u2,u3; //Global effort values for response.
    double k_d_1=5.0, k_p_1=1.0; //Global gain variables for each joint.
    double k_d_2=2.5, k_p_2=1.0;
    double k_d_3=50, k_p_3=2000;


};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}