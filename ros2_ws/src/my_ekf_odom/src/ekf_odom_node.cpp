#include <my_ekf_odom/ekf_odom_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto component = std::make_shared<autobin::EKFODOMComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}