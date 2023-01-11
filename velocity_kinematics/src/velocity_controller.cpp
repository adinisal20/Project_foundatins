#include "rclcpp/rclcpp.hpp"
#include <math.h> 
class VController : public rclcpp::Node 
{
public:
    VController() : Node("velocity_controller") 
    {
        server_1 = this->create_service<force_srv_file::srv::AddForceVectors>("/joint_vel_receiver",
        std::bind(&VController::Jacob, this, std::placeholders::_1, std::placeholders::_2));

        server_2 = this->create_service<force_srv_file::srv::AddForceVectors>("/joint_vel_receiver",
        std::bind(&VController::Inv_Jacob, this, std::placeholders::_1, std::placeholders::_2));

    }
     
private:
    void Jacob(const force_srv_file::srv::AddForceVectors::Request::SharedPtr request,
                     force_srv_file::srv::AddForceVectors::Response::SharedPtr response)
    {
        double q_1_dot = request->v[0];
        double q_2_dot = request->v[1];
        double q_3_dot = request->v[2];
        
        double s_1 = 
        double jacob[3][3] = [[-l1*sin()]]
    }

    void Inv_Jacob(const force_srv_file::srv::AddForceVectors::Request::SharedPtr request,
                         force_srv_file::srv::AddForceVectors::Response::SharedPtr response)
    {
        
    }
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VController>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}