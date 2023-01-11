#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "force_srv_file/srv/add_force_vectors.hpp"
#include "srv_file/srv/foundations_three.hpp"
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
class Vel_Controller : public rclcpp::Node 
{
public:
    Vel_Controller() : Node("velocity_controller_node"), error1_prev(0), error2_prev(0), error3_prev(0) //Initializing the previous error as zero
    {
        //Created a publisher to the topic /forward_effort_controller/command with queue size 10
        subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
        std::bind(&Vel_Controller::JointStateSub, this, std::placeholders::_1));
        
        publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);       

        //Created a server with service /input_angles. Client will be through the terminal    
        server = this->create_service<force_srv_file::srv::AddForceVectors>("/input_velocities",
        std::bind(&Vel_Controller::InputCallback, this, std::placeholders::_1, std::placeholders::_2));

        server_ = this->create_service<srv_file::srv::FoundationsThree>("/joint_vel",
        std::bind(&Vel_Controller::OutputCallback, this, std::placeholders::_1, std::placeholders::_2));

        //Created a subscriber to /joint_states topic to get position and velocity at runtime

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

        //Defining the inverse jacobian 
        double Jinv[3][3] = {{(cos(array1_[0])*cos(array1_[1]) - sin(array1_[0])*sin(array1_[1]))/(l1*sin(array1_[1])*pow(cos(array1_[0]),2)+ l1*sin(array1_[1])*pow(sin(array1_[0]),2)),(cos(array1_[0])*sin(array1_[1]) + cos(array1_[1])*sin(array1_[0]))/(l1*sin(array1_[1])*pow(cos(array1_[0]),2) + l1*sin(array1_[1])*pow(sin(array1_[0]),2)),0},
                        {-(l1*cos(array1_[0]) + l2*cos(array1_[0])*cos(array1_[1]) - l2*sin(array1_[0])*sin(array1_[1]))/(l1*l2*sin(array1_[1])*pow(cos(array1_[0]),2) + l1*l2*sin(array1_[1])*pow(sin(array1_[0]),2)), -(l1*sin(array1_[0]) + l2*cos(array1_[0])*sin(array1_[1]) + l2*cos(array1_[1])*sin(array1_[0]))/(l1*l2*sin(array1_[1])*pow(cos(array1_[0]),2) + l1*l2*sin(array1_[1])*pow(sin(array1_[0]),2)),0},
                        {0  ,0, -1}};
        //Calculating the desired joint velocities in all the three directions
        double j_v_1 = (Jinv[0][0] * v[0]) + (Jinv[0][1] * v[1]) + (Jinv[0][2] * v[2]);
        double error1 = j_v_1 - array1_[3];
        double error1_dot = error1 - error1_prev;
        error1_prev = error1;


        double j_v_2 = (Jinv[1][0] * v[0]) + (Jinv[1][1] * v[1]) + (Jinv[1][2] * v[2]);
        double error2 = j_v_2 - array1_[4];
        double error2_dot = error2 - error2_prev;
        error2_prev = error2;


        double j_v_3 = (Jinv[2][0] * v[0]) + (Jinv[2][1] * v[1]) + (Jinv[2][2] * v[2]);
        double error3 = j_v_3 - array1_[5];
        double error3_dot = error3 - error3_prev;
        error3_prev = error3;

        //Getting Effort values fro the control equations
        double u1 = k_p_1*(error1) + k_d_1*(error1_dot);
        double u2 = k_p_2*(error2) + k_d_2*(error2_dot);
        double u3 = k_p_3*(error3) + k_d_3*(error3_dot);

        //Defining the jacobian so as to plot the plot
        double jacob[3][3] = {{-l1*sin(array1_[0]) - l2*cos(array1_[0])*sin(array1_[1]) - l2*cos(array1_[1])*sin(array1_[0]), - l2*cos(array1_[0])*sin(array1_[1]) - l2*cos(array1_[1])*sin(array1_[0]),  0}, 
                              {l1*cos(array1_[0]) + l2*cos(array1_[0])*cos(array1_[1]) - l2*sin(array1_[0])*sin(array1_[1]),  l2*cos(array1_[0])*cos(array1_[1]) - l2*sin(array1_[0])*sin(array1_[1]),  0},
                              {0 ,0 ,-1}};

        //Calculating the runtime end effector velocity in x, y and z directions
        double vel_1 = (jacob[0][0] * array1_[3]) + (jacob[0][1] * array1_[4]) + (jacob[0][2] * array1_[5]);
        double vel_2 = (jacob[1][0] * array1_[3]) + (jacob[1][1] * array1_[4]) + (jacob[1][2] * array1_[5]);
        double vel_3 = (jacob[2][0] * array1_[3]) + (jacob[2][1] * array1_[4]) + (jacob[2][2] * array1_[5]);
        //Creating time variable of milliseconds/10
        int time = value.header.stamp.sec*100 + value.header.stamp.nanosec/10000000;
        
        //Creating a CSV file to store the desired, runtime positions and the time.
        Plots.open ("f_p3_plots.csv",std::ios::app);
		Plots <<v[0]<< "," <<v[1]<< "," <<v[2]<< "," <<vel_1<< "," <<vel_2<< "," <<vel_3<< "," <<time<<"\n";
    	Plots.close();

        //Storing the value in the variable effort and publishing the data variable from effort.
        auto effort = std_msgs::msg::Float64MultiArray();
        effort.data = {u1, u2, u3};
        publisher->publish(effort);     

    } 
    //Function to call the service to convert the desired end effector velocities to desired joint velocities
    void InputCallback(const force_srv_file::srv::AddForceVectors::Request::SharedPtr request,
                       force_srv_file::srv::AddForceVectors::Response::SharedPtr response)
    {
        v[0] = request->v1[0];
        v[1] = request->v1[1];
        v[2] = request->v1[2];
        
        //Locally using jacob so as to ge the eend effector runtime velocity
        double Jinv[3][3] = {{(cos(array1_[0])*cos(array1_[1]) - sin(array1_[0])*sin(array1_[1]))/(l1*sin(array1_[1])*pow(cos(array1_[0]),2)+ l1*sin(array1_[1])*pow(sin(array1_[0]),2)),(cos(array1_[0])*sin(array1_[1]) + cos(array1_[1])*sin(array1_[0]))/(l1*sin(array1_[1])*pow(cos(array1_[0]),2) + l1*sin(array1_[1])*pow(sin(array1_[0]),2)),0},
                        {-(l1*cos(array1_[0]) + l2*cos(array1_[0])*cos(array1_[1]) - l2*sin(array1_[0])*sin(array1_[1]))/(l1*l2*sin(array1_[1])*pow(cos(array1_[0]),2) + l1*l2*sin(array1_[1])*pow(sin(array1_[0]),2)), -(l1*sin(array1_[0]) + l2*cos(array1_[0])*sin(array1_[1]) + l2*cos(array1_[1])*sin(array1_[0]))/(l1*l2*sin(array1_[1])*pow(cos(array1_[0]),2) + l1*l2*sin(array1_[1])*pow(sin(array1_[0]),2)),0},
                        {0  ,0, -1}};
        double j_v_1 = (Jinv[0][0] * v[0]) + (Jinv[0][1] * v[1]) + (Jinv[0][2] * v[2]);
        double j_v_2 = (Jinv[1][0] * v[0]) + (Jinv[1][1] * v[1]) + (Jinv[1][2] * v[2]);
        double j_v_3 = (Jinv[2][0] * v[0]) + (Jinv[2][1] * v[1]) + (Jinv[2][2] * v[2]);

        std::cout<<"Desired joint velocity for joint 1:"<<j_v_1<<std::endl;
        std::cout<<"Desired joint velocity for joint 2:"<<j_v_2<<std::endl;
        std::cout<<"Desired joint velocity for joint 3:"<<j_v_3<<std::endl;

        response->v2[0] = j_v_1;
        response->v2[1] = j_v_2;
        response->v2[2] = j_v_3;


    }
    //Function to call the service to convert the runtime joint velocities to runtime end effector velocities
    void OutputCallback(const srv_file::srv::FoundationsThree::Request::SharedPtr request,
                       srv_file::srv::FoundationsThree::Response::SharedPtr response)
    {
        double a = 0;
        double b = 0;
        double c = 0;
        a = request->i1[0] ;
        b = request->i1[1] ;
        c = request->i1[2] ;
        //Locally using jacob so as to ge the eend effector runtime velocity
        double jacob[3][3] = {{-l1*sin(array1_[0]) - l2*cos(array1_[0])*sin(array1_[1]) - l2*cos(array1_[1])*sin(array1_[0]), - l2*cos(array1_[0])*sin(array1_[1]) - l2*cos(array1_[1])*sin(array1_[0]),  0}, 
                              {l1*cos(array1_[0]) + l2*cos(array1_[0])*cos(array1_[1]) - l2*sin(array1_[0])*sin(array1_[1]),  l2*cos(array1_[0])*cos(array1_[1]) - l2*sin(array1_[0])*sin(array1_[1]),  0},
                              {0 ,0 ,-1}};

        double vel_0_1 = (jacob[0][0] * array1_[3]) + (jacob[0][1] * array1_[4]) + (jacob[0][2] * array1_[5]);
        double vel_0_2 = (jacob[1][0] * array1_[3]) + (jacob[1][1] * array1_[4]) + (jacob[1][2] * array1_[5]);
        double vel_0_3 = (jacob[2][0] * array1_[3]) + (jacob[2][1] * array1_[4]) + (jacob[2][2] * array1_[5]);
        std::cout<<"The runtime velocity for end effector in x:"<<vel_0_1<<std::endl;
        std::cout<<"The runtime velocity for end effector in y:"<<vel_0_2<<std::endl;
        std::cout<<"The runtime velocity for end effector in z:"<<vel_0_3<<std::endl;
        response->i2[0] = vel_0_1;
        response->i2[1] = vel_0_2;
        response->i2[2] = vel_0_3;

    }

    rclcpp::Service<force_srv_file::srv::AddForceVectors>::SharedPtr server;    
    rclcpp::Service<srv_file::srv::FoundationsThree>::SharedPtr server_;    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;
    double array1_[6]; //Global array that gets updated in subscriber.
    double v[3]; //Array to store request.
    double u[3];
    double u1,u2,u3; //Global effort values for response.
    double k_d_1=0.01, k_p_1=6.0; //Global gain variables for each joint.
    double k_d_2=0.01, k_p_2=6.0;
    double k_d_3=0.01, k_p_3=6.0;
    double error1_prev;//Globally declared the previous error so it can be initialized in the constuctor.
    double error2_prev;
    double error3_prev;
    double vel_1, vel_2, vel_3; //Global variable for runtime end effector velocity.
    double l1 = 1.0; 
    double l2 = 1.0;
};  
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Vel_Controller>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}