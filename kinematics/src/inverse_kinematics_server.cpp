#include "rclcpp/rclcpp.hpp"
#include "force_srv_file/srv/add_force_vectors.hpp"
#include <memory>

void add(const std::shared_ptr<force_srv_file::srv::AddForceVectors::Request> request,
          std::shared_ptr<force_srv_file::srv::AddForceVectors::Response> response)
{ 
  double r_ = sqrt(pow(request->v1[0], 2) + pow(request->v1[1], 2));
  double fi =  acos(pow(r_, 2)/(2*r_));
  double alpha = atan2(request->v1[1], request->v1[0]);
  response->v2[0] = alpha - fi;
  response->v2[1] = atan2(r_*sin(fi), (r_*cos(fi))-1);
  response->v2[2] = -request->v1[2];
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nv1: [%f %f %f]" ,
                request->v1[0], request->v1[1], request->v1[2]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f %f %f]", (double)response->v2[0], response->v2[1], response->v2[2]);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("IKinematics_server");

  rclcpp::Service<force_srv_file::srv::AddForceVectors>::SharedPtr service =
    node->create_service<force_srv_file::srv::AddForceVectors>("IKinematics", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to output the pose");

  rclcpp::spin(node);
  rclcpp::shutdown();
}