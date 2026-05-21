#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include "GetPose.h"

class GetPose : public rclcpp::Node {
public:
    GetPose() : Node("get_pose") {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

    geometry_msgs::msg::TransformStamped get_pose(std::string base, std::string target){
        while(!buffer_->canTransform(
            base,
            target,
            tf2::TimePointZero
        )){}
            try {
                geometry_msgs::msg::TransformStamped transform =
                buffer_->lookupTransform(
                    base,
                    target,
                    tf2::TimePointZero
                );
                return transform;

            }
            catch (const tf2::TransformException &ex){
                RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
            }
            
    }

private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
};