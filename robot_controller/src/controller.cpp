#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "force_srv_file/srv/add_force_vectors.hpp"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <chrono>
#include <bits/stdc++.h>
using namespace std::chrono_literals;

class Controller : public rclcpp::Node 
{
public:
    Controller() : Node("controller_node")
    {
        publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);       
        server = this->create_service<force_srv_file::srv::AddForceVectors>("/input_values",
        std::bind(&Controller::InputCallback, this, std::placeholders::_1, std::placeholders::_2));
        // timer = this->create_wall_timer(std::chrono::milliseconds(10),
        //                                 std::bind(&Controller::PublishEfforts, this));    //Change time
        subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
                                        std::bind(&Controller::JointStateSub, this, std::placeholders::_1));
    }
     
private:
    // void PublishEffort()
    // {
    //     auto array = std_msgs::msg::Float64MultiArray();
    //     array.data = {u1, u2, u3};
    //     publisher->publish(array);
    // }

    void JointStateSub(const sensor_msgs::msg::JointState value)
    {
        array1_[0] = value.position[0];
        array1_[1] = value.position[1];
        array1_[2] = value.position[2];
        array1_[3] = value.velocity[0];
        array1_[4] = value.velocity[1];
        array1_[5] = value.velocity[2];
    }

    void InputCallback(const force_srv_file::srv::AddForceVectors::Request::SharedPtr request,
                       force_srv_file::srv::AddForceVectors::Response::SharedPtr response)
    {
        std::cout<<"Reference positions:"<<std::endl;
        std::cout<<request->v1[0]<<std::endl<<request->v1[1]<<std::endl<<request->v1[2]<<std::endl;

        double error1 = array1_[0] - request->v1[0];
        std::cout<<error1<<std::endl;
        std::cout<<"Array01 = "<<array1_[0]<<std::endl;
        double E1 = error1;
        double Kp_1 = 3;
        double Kd_1 = 500;
        // double Ki_1 = 0.000001;

        double error2 = array1_[1] - request->v1[1];
        std::cout<<error2<<std::endl;
        std::cout<<"Array02 = "<<array1_[1]<<std::endl;
        double E2 = error2;        
        double Kp_2 = 5;
        double Kd_2 = 40;
        // double Ki_2 = 0.00001;

        double error3 = array1_[2] - request->v1[2]; 
        std::cout<<error3<<std::endl;
        std::cout<<"Array03 = "<<array1_[2]<<std::endl;
        double E3 = error3;
        double Kp_3 = 0.5;
        double Kd_3 = 0.065*5;
        // double Ki_3 = 0.0001;
        
        while (abs(error1) > 0.1 || abs(error2) > 0.1 || abs(error3) > 0.1)
        {
            // struct value = sensor_msgs::msg::JointState() ;
            
            // array1_[0] = value->position[0];
            // array1_[1] = value->position[1];
            // array1_[2] = value->position[2];
            // array1_[3] = value->velocity[0];
            // array1_[4] = value->velocity[1];
            // array1_[5] = value->velocity[2];

            double error1 = array1_[0] - request->v1[0];
            std::cout<<"Error1="<<error1<<" "<<"E1="<<E1<<std::endl;
            double u1 = Kp_1*error1 - Kd_1*(error1 - E1);
            E1 = error1;
            std::cout<<"u1 = "<<u1<<std::endl;

            double error2 = array1_[1] - request->v1[1];
            double u2 = Kp_2*error2 - Kd_2*(error2 - E2);
            E2 = error2 - E2;
            std::cout<<"u2 = "<<u2<<std::endl;

            double error3 = array1_[2] - request->v1[2];
            E3 = error3 - E3;  
            double u3 = Kp_3*error3 - Kd_3*(error3 - E3);
            std::cout<<"u3 = "<<u3<<std::endl;

            std::cout<<error1<<std::endl;
            std::cout<<"Array1 = "<<array1_[0]<<std::endl;
            std::cout<<error2<<std::endl;
            std::cout<<"Array2 = "<<array1_[1]<<std::endl;
            std::cout<<error3<<std::endl;
            std::cout<<"Array3 = "<<array1_[2]<<std::endl;

            if ((u3 > 0) && (u3 < 2.1))
            {
                u3 = 2.1;
            }
            if ((u3 < 0) && (abs(u3) < 2.1))
            {
                u3 = -2.1;
            }

            auto array = std_msgs::msg::Float64MultiArray();
            array.data = {u1, u2, u3};
            publisher->publish(array);

            response->v2[0] = u1;
            response->v2[1] = u2;
            response->v2[2] = u3;
        }

        //Define controller here. Allocate values to array_[0], array_[1], array_[2]separately.
        // PublishEffort(); 
        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<force_srv_file::srv::AddForceVectors>::SharedPtr server;    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber;
    double array1_[6];
    double u1,u2,u3;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}