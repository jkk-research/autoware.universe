#ifndef DURO_LOCALIZER__DURO_LOCALIZER_HPP_
#define DURO_LOCALIZER__DURO_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <functional>
#include <memory>

class DuroLocalizer : public rclcpp::Node
{
    public:
        DuroLocalizer();
    private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_with_covariance_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
        rclcpp::Publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr pub_localization_init_state_;
        rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pose_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_;

        double gpsOffsetX;
        double gpsOffsetY;
        double gpsOffsetZ;

        void pose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg_);
        void velocity_callback(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::VelocityReport>& msg_);
};

#endif // DURO_LOCALIZER__DURO_LOCALIZER_HPP_