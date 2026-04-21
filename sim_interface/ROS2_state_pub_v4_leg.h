/*
 * ROS2 state publisher bridge for MuJoCo simulation (speedbot_v4 leg-only).
 * It publishes IMU and joint states so RViz can subscribe and visualize
 * the same topics used by real robot workflows.
 */
#pragma once

#include "controller_config.h"
#include "data_bus.h"
#include <array>
#include <memory>
#include <string>

#ifndef OPENLOONG_HAS_ROS2
#define OPENLOONG_HAS_ROS2 0
#endif

#ifndef OPENLOONG_HAS_TF2
#define OPENLOONG_HAS_TF2 0
#endif

#if OPENLOONG_HAS_ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#if OPENLOONG_HAS_TF2
#include <tf2_ros/transform_broadcaster.h>
#endif
#endif

class ROS2_StatePub_V4_Leg
{
public:
    ROS2_StatePub_V4_Leg();
    ~ROS2_StatePub_V4_Leg();

    bool initialize(const ControllerConfig &config, std::string *errMsg = nullptr);
    void spinSome();
    void publishState(const DataBus &busIn);

private:
#if OPENLOONG_HAS_ROS2
    static void rpyToQuat(double roll, double pitch, double yaw, double quatWxyz[4]);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesPub_;
#if OPENLOONG_HAS_TF2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
#endif

    std::string topicImu_;
    std::string topicJointStates_;
    bool isInitialized_{false};

    // /joint_states publish order aligned with ref/walking_parameters/walking_controller.py::ROS_JOINT_NAMES
    static const std::array<std::string, 12> kJointNamesRosOrder;
    // Mapping: ros_order_index -> internal pin/dataBus index (left6 + right6)
    static const std::array<int, 12> kRosToInternalIndex;
#endif
};
