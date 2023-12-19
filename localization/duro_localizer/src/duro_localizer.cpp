#include "duro_localizer/duro_localizer.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

DuroLocalizer::DuroLocalizer() : Node("duro_localizer")
{
    // Initialize base_link - GPS transform
    tf2_ros::Buffer tfBuffer(this->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::msg::TransformStamped gps2baselink_transform = tfBuffer.lookupTransform("lexus3/gps", "lexus3/base_link", rclcpp::Time(0), rclcpp::Duration(3, 0));
    gpsOffsetX = gps2baselink_transform.transform.translation.x;
    gpsOffsetY = gps2baselink_transform.transform.translation.y;
    gpsOffsetZ = gps2baselink_transform.transform.translation.z;

    // Initialize publishers
    pub_pose_with_covariance_    = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/pose_estimator/pose_with_covariance", 1);
    pub_pose_                    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose_twist_fusion_filter/pose", 1);
    pub_localization_init_state_ = this->create_publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>("/localization/initialization_state", 1);
    pub_acceleration_            = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("/localization/acceleration", 1);
    pub_init_pose_               = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose3d", 1);

    // Initialize subscribers
    sub_pose_     = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/lexus3/gps/duro/current_pose_with_cov", 1, std::bind(&DuroLocalizer::pose_callback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1, std::bind(&DuroLocalizer::velocity_callback, this, std::placeholders::_1));

    // Initialize tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void DuroLocalizer::pose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg_)
{
    // Calculate rotational offset
    tf2::Quaternion orientation_offset(-msg_->pose.pose.orientation.x, -msg_->pose.pose.orientation.y, -msg_->pose.pose.orientation.z, msg_->pose.pose.orientation.w);
    
    // quaternion start_orientation_offset to roll pitch yaw
    double roll_so, pitch_so, yaw_so;
    tf2::Matrix3x3(orientation_offset).getRPY(roll_so, pitch_so, yaw_so);

    // rotate gps.pose.position.x and y by orientation_offset Z (yaw) around origo
    double rot_sin = sin(-yaw_so);
    double rot_cos = cos(-yaw_so);

    double posX = gpsOffsetX;
    double posY = gpsOffsetY;

    double correctedOffsetX = posX * rot_cos - posY * rot_sin;
    double correctedOffsetY = posX * rot_sin + posY * rot_cos;

    // Offset gps
    geometry_msgs::msg::PoseWithCovarianceStamped gps;
    gps.header = msg_->header;
    gps.pose.pose.position.x = msg_->pose.pose.position.x + correctedOffsetX;
    gps.pose.pose.position.y = msg_->pose.pose.position.y + correctedOffsetY;
    gps.pose.pose.position.z = msg_->pose.pose.position.z + gpsOffsetZ;
    gps.pose.pose.orientation = msg_->pose.pose.orientation;

    
    // Publish pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = gps.header;
    pose_msg.pose = gps.pose.pose;
    pub_pose_->publish(pose_msg);

    // Publish pose with covariance
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = gps.header;
    pose_cov_msg.pose = gps.pose;
    pub_pose_with_covariance_->publish(pose_cov_msg);

    // // Publish init pose
    // pub_init_pose_->publish(pose_cov_msg);

    // Publish localization initialization state
    autoware_adapi_v1_msgs::msg::LocalizationInitializationState state;
    state.stamp = gps.header.stamp;
    state.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
    pub_localization_init_state_->publish(state);

    // Publish tf
    geometry_msgs::msg::TransformStamped tf;
    tf.header = gps.header;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = gps.pose.pose.position.x;
    tf.transform.translation.y = gps.pose.pose.position.y;
    tf.transform.translation.z = gps.pose.pose.position.z;
    tf.transform.rotation = gps.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
}

void DuroLocalizer::velocity_callback(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::VelocityReport>& msg_)
{
    // Publish acceleration
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header = msg_->header;
    accel.accel.accel.linear.x = msg_->longitudinal_velocity;
    accel.accel.accel.linear.y = msg_->lateral_velocity;
    accel.accel.accel.linear.z = 0.0;
    accel.accel.accel.angular.x = 0.0;
    accel.accel.accel.angular.y = 0.0;
    accel.accel.accel.angular.z = 0.0;
    pub_acceleration_->publish(accel);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DuroLocalizer>());
    rclcpp::shutdown();
    return 0;
}