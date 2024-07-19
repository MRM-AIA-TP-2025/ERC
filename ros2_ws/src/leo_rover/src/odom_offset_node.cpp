#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <unordered_map>
#include <vector>

class OdomCorrectionNode : public rclcpp::Node
{
public:
    OdomCorrectionNode() : Node("odom_offset_node")
    {
        // Known Aruco tag poses in the world frame
        known_tag_poses_[1] = { -7.003807f, 7.617856f, 1.172268f };
        known_tag_poses_[4] = { 4.826193f, 4.817856f, 1.072268f };
        known_tag_poses_[5] = { 1.706193f, -9.572144f, 1.582268f };
        known_tag_poses_[8] = { 0.866193f, 0.047856f, 1.452268f };
        known_tag_poses_[12] = { 6.336193f, -6.492144f, 1.982268f };
        known_tag_poses_[13] = { -8.823807f, -11.012144f, 1.792268f };

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_unfiltered", 10, std::bind(&OdomCorrectionNode::odom_callback, this, std::placeholders::_1));

        tvec_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "aruco/tvec", 10, std::bind(&OdomCorrectionNode::tvec_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void tvec_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        tag_id_ = static_cast<int>(msg->data[0]);
        tvec_ = { msg->data[1], msg->data[2], msg->data[3] };

        if (known_tag_poses_.find(tag_id_) != known_tag_poses_.end())
        {
            calculate_offset(tvec_, tag_id_);
        }
    }

    void calculate_offset(const std::vector<float>& tvec, int tag_id)
    {
        if (current_odom_)
        {
            std::array<float, 3> rover_pos = {
                static_cast<float>(current_odom_->pose.pose.position.x),
                static_cast<float>(current_odom_->pose.pose.position.y),
                static_cast<float>(current_odom_->pose.pose.position.z)
            };

            std::array<float, 3> calculated_tag_pos = {
                rover_pos[0] + tvec[0],
                rover_pos[1] + tvec[1],
                rover_pos[2] + tvec[2]
            };

            const std::array<float, 3>& known_tag_pose = known_tag_poses_[tag_id];
            offset_ = {
                known_tag_pose[0] - calculated_tag_pos[0],
                known_tag_pose[1] - calculated_tag_pos[1],
                known_tag_pose[2] - calculated_tag_pos[2]
            };
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = msg;
        apply_offset_and_publish();
    }

    void apply_offset_and_publish()
    {
        if (!current_odom_)
            return;

        nav_msgs::msg::Odometry corrected_odom = *current_odom_;

        corrected_odom.pose.pose.position.x += offset_[0];
        corrected_odom.pose.pose.position.y += offset_[1];
        corrected_odom.pose.pose.position.z += offset_[2];

        publisher_->publish(corrected_odom);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tvec_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::unordered_map<int, std::array<float, 3>> known_tag_poses_;
    std::vector<float> tvec_;
    int tag_id_ = -1;
    std::array<float, 3> offset_ = { 0.0f, 0.0f, 0.0f };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomCorrectionNode>());
    rclcpp::shutdown();
    return 0;
}
