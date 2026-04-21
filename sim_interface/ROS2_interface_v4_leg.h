/*
 * ROS2 interface for speedbot_v4 leg-only real robot backend.
 * It subscribes IMU/joint state feedback and publishes 12-leg action commands
 * in the same topic layout as the RL deployment pipeline.
 */
#pragma once

#include "controller_config.h"
#include "data_bus.h"
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifndef OPENLOONG_HAS_ROS2
#define OPENLOONG_HAS_ROS2 0
#endif

#if OPENLOONG_HAS_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#endif

class ROS2_Interface_V4_Leg
{
public:
    ROS2_Interface_V4_Leg();
    ~ROS2_Interface_V4_Leg();

    bool initialize(const ControllerConfig &config, std::string *errMsg = nullptr);
    void spinSome();
    bool isReady() const;
    bool hasFreshData(double timeoutSec) const;

    void dataBusWrite(DataBus &busIn);
    void setMotorsPosition(const std::vector<double> &qDesIn);

private:
#if OPENLOONG_HAS_ROS2
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    static void quatToRpy(double qw, double qx, double qy, double qz, double rpyOut[3]);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actionCmdPub_;

    std::string topicImu_;
    std::string topicJointStates_;
    std::string topicActionCmd_;

    bool isInitialized_{false};
    bool imuReceived_{false};
    bool jointStatesReceived_{false};
    bool yawInited_{false};
    double yawSingle_{0.0};
    int yawRound_{0};

    mutable std::mutex dataMutex_;
    TimePoint lastImuTime_;
    TimePoint lastJointStatesTime_;

    std::array<double, 4> quatWxyz_{{1.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> baseAngVel_{{0.0, 0.0, 0.0}};
    std::array<double, 3> baseAcc_{{0.0, 0.0, 0.0}};
    std::array<double, 12> motorsPos_{{0.0}};
    std::array<double, 12> motorsVel_{{0.0}};
    std::array<double, 12> motorsEff_{{0.0}};

    static const std::array<std::string, 12> kJointNamesPinOrder;
#endif
};
