#include "rclcpp/rclcpp.hpp"
#include "force_srv_file/srv/add_force_vectors.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: IKinematics_client v1");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("IKinematics_client");
  rclcpp::Client<force_srv_file::srv::AddForceVectors>::SharedPtr client =
    node->create_client<force_srv_file::srv::AddForceVectors>("IKinematics");
  auto request = std::make_shared<force_srv_file::srv::AddForceVectors::Request>();
  request->v1[0] = atoll(argv[1]);
  request->v1[1] = atoll(argv[2]);
  request->v1[2] = atoll(argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
   
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  { 
 
   auto vector=result.get()->v2;
   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ": q1:%f, q2:%f, d:%f", vector[0], vector[1], vector[2]); 
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_force_vectors");
  }

  rclcpp::shutdown();
  return 0;
}