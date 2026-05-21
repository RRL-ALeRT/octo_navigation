#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class GetPose : public rclcpp::Node {
public:
    GetPose();
    geometry_msgs::msg::TransformStamped get_pose(std::string base, std::string target);

};