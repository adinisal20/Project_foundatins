#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <chrono>
#include <bits/stdc++.h>
using namespace std::chrono_literals;

class FKine : public rclcpp::Node 
{
public:
    FKine() : Node("FKine") 
    {
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&FKine::get_pose, this, std::placeholders::_1));
        publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("forward_kinematics", 10);
        RCLCPP_INFO(this->get_logger(), "Forward kinematics is started and the pose will be published on the topic : /forward_kinematics");    

    }
		//Declaring a destructor
	
    // ~FKine(){        
            
    //         }
private:

    void get_pose(const sensor_msgs::msg::JointState value)
    {
    	double q1, q2, d;
        q1 = value.position[0];
        q2 = value.position[1];
        d = value.position[2];
        double q3 = 0;
        int a1=1;
        int a2=1;
        int a3 = 0;

        double A_1[4][4] = {{cos(q1), -(sin(q1) * cos(0)), (sin(q1) * sin(0)), a1*cos(q1)}, 
        {(sin(q1)), (cos(q1)*cos(0)), -(cos(q1)*sin(0)), a1*sin(q1)},
        {0, sin(0), cos(0), 0},
        {0, 0, 0, 1}};

        double A_2[4][4] = {{cos(q2), -(sin(q2) * cos(M_PI)), (sin(q2) * sin(M_PI)), a2*cos(q2)}, 
        {(sin(q2)), (cos(q2)*cos(M_PI)), -(cos(q2)*sin(M_PI)), a2*sin(q2)},
        {0, sin(M_PI), cos(M_PI), 0},
        {0, 0, 0, 1}};

        double A_3[4][4] = {{cos(q3), -(sin(q3) * cos(0)), (sin(q3) * sin(0)), a3*cos(q3)}, 
        {(sin(q3)), (cos(q3)*cos(0)), -(cos(q3)*sin(0)), a3*sin(q3)},
        {0, sin(0), cos(0), d},
        {0, 0, 0, 1}};       

        double rslt[4][4];
        double T[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                rslt[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    rslt[i][j] += A_1[i][k] * A_2[k][j];
           
        }
        }
        }
        for (int p = 0; p < 4; p++) {
            for (int q = 0; q < 4; q++) {
                T[p][q] = 0;
                for (int r = 0; r < 4; r++) {
                    T[p][q] += rslt[p][r] * A_3[r][q];
           
        }
        }
        auto value = std_msgs::msg::Float64MultiArray();
        value.data = {T[0][3], T[1][3], T[2][3]};
        publisher->publish(value);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;    // Joint_state topic subscriber
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FKine>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
