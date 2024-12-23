#include <cmath>
#include <iostream>
#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <vector>
#include "std_msgs/msg/float32_multi_array.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("custom_target_movement");

// グローバル変数
float hand_x = 0.3;
float hand_y = 0.15;
float hand_z = 0.2;
float digree = 0.0;
float tilt = 0.0;
float grab = 0.0;

double roll = 90.0, pitch = 0.0, yaw = 90.0;

// MoveGroupInterfaceをグローバル変数として定義
std::shared_ptr<MoveGroupInterface> move_group_arm;
std::shared_ptr<MoveGroupInterface> move_group_gripper;

// 手のデータを出力する関数
void print_hand_data()
{
    std::cout << "hand_x:   " << hand_x << std::endl;
    std::cout << "hand_y:   " << hand_y << std::endl;
    std::cout << "hand_z:   " << hand_z << std::endl;
    std::cout << "digree:   " << digree << std::endl;
    std::cout << "tilt:     " << tilt << std::endl;
    std::cout << "grab:     " << grab << std::endl;
}

void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 5) {
        std::cerr << "Error: Received data does not have enough elements (expected 5)." << std::endl;
        return;
    }

    // グローバル変数に代入
    hand_x = 0.5-(msg->data[1] / 1000.0);
    hand_y = (msg->data[0] / 1000.0)-0.2;
    hand_z = 0.3;
    digree = 0;
    tilt = 0;
    grab = msg->data[4];

    // データを表示
    print_hand_data();

    // 入力値を用いてPoseを設定
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = hand_x;
    target_pose.position.y = hand_y;
    target_pose.position.z = hand_z;

    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);

    // MoveGroupInterfaceを使用して目標座標へ移動
    move_group_arm->setPoseTarget(target_pose);
    // if(grab == 1){
    //     gripper_joint_values[0] = angles::from_degrees(60);
    //     move_group_gripper.setJointValueTarget(gripper_joint_values);
    //     move_group_gripper.move();
    // }else{
    //     gripper_joint_values[0] = angles::from_degrees(15);
    //     move_group_gripper.setJointValueTarget(gripper_joint_values);
    //     move_group_gripper.move();
    // }
    

    if (move_group_arm->move() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Move successful!");
    } else {
        RCLCPP_ERROR(LOGGER, "Failed to reach target position.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("listener_node");
    
    auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
    auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

    move_group_arm = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm->setMaxVelocityScalingFactor(1.0);
    move_group_arm->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper->setMaxVelocityScalingFactor(1.0);
    move_group_gripper->setMaxAccelerationScalingFactor(1.0);

    auto subscription = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/hand_topic", 10, callback);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
