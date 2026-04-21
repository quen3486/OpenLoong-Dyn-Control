/*
 * ROS2 interface implementation for speedbot_v4 leg-only real robot backend.
 */
#include "ROS2_interface_v4_leg.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

#if OPENLOONG_HAS_ROS2
const std::array<std::string, 12> ROS2_Interface_V4_Leg::kJointNamesPinOrder = {
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
#endif

ROS2_Interface_V4_Leg::ROS2_Interface_V4_Leg() = default;

ROS2_Interface_V4_Leg::~ROS2_Interface_V4_Leg()
{
#if OPENLOONG_HAS_ROS2
    if (rclcpp::ok() && node_ != nullptr && executor_ != nullptr)
    {
        executor_->remove_node(node_);
    }
#endif
}

bool ROS2_Interface_V4_Leg::initialize(const ControllerConfig &config, std::string *errMsg)
{
#if OPENLOONG_HAS_ROS2
    topicImu_ = config.rosTopicImu;
    topicJointStates_ = config.rosTopicJointStates;
    topicActionCmd_ = config.rosTopicActionCmd;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("openloong_mpc_wbc_leg_real");

    imuSub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        topicImu_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { this->imuCallback(msg); });
    jointStatesSub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topicJointStates_, 50,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        { this->jointStatesCallback(msg); });
    actionCmdPub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topicActionCmd_, 20);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    isInitialized_ = true;
    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
#else
    (void)config;
    if (errMsg != nullptr)
    {
        *errMsg = "ROS2 support is disabled at build time.";
    }
    return false;
#endif
}

void ROS2_Interface_V4_Leg::spinSome()
{
#if OPENLOONG_HAS_ROS2
    if (isInitialized_)
    {
        if (executor_ != nullptr)
        {
            executor_->spin_some();
        }
    }
#endif
}

bool ROS2_Interface_V4_Leg::isReady() const
{
#if OPENLOONG_HAS_ROS2
    std::lock_guard<std::mutex> lock(dataMutex_);
    return isInitialized_ && imuReceived_ && jointStatesReceived_;
#else
    return false;
#endif
}

bool ROS2_Interface_V4_Leg::hasFreshData(double timeoutSec) const
{
#if OPENLOONG_HAS_ROS2
    const double timeoutSafe = std::max(0.01, timeoutSec);
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (!imuReceived_ || !jointStatesReceived_)
    {
        return false;
    }
    const double imuAge = std::chrono::duration<double>(now - lastImuTime_).count();
    const double jointAge = std::chrono::duration<double>(now - lastJointStatesTime_).count();
    return imuAge <= timeoutSafe && jointAge <= timeoutSafe;
#else
    (void)timeoutSec;
    return false;
#endif
}

void ROS2_Interface_V4_Leg::dataBusWrite(DataBus &busIn)
{
#if OPENLOONG_HAS_ROS2
    std::array<double, 4> quatCopy;
    std::array<double, 3> angVelCopy;
    std::array<double, 3> accCopy;
    std::array<double, 12> posCopy;
    std::array<double, 12> velCopy;
    std::array<double, 12> effCopy;

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        quatCopy = quatWxyz_;
        angVelCopy = baseAngVel_;
        accCopy = baseAcc_;
        posCopy = motorsPos_;
        velCopy = motorsVel_;
        effCopy = motorsEff_;
    }

    double rpyNow[3]{0.0, 0.0, 0.0};
    quatToRpy(quatCopy[0], quatCopy[1], quatCopy[2], quatCopy[3], rpyNow);

    if (!yawInited_)
    {
        yawSingle_ = rpyNow[2];
        yawRound_ = 0;
        yawInited_ = true;
    }
    else
    {
        if ((rpyNow[2] - yawSingle_) > M_PI * 0.5)
        {
            yawRound_ -= 1;
        }
        else if ((rpyNow[2] - yawSingle_) < -M_PI * 0.5)
        {
            yawRound_ += 1;
        }
        yawSingle_ = rpyNow[2];
    }
    rpyNow[2] = yawSingle_ + yawRound_ * 2.0 * M_PI;

    busIn.rpy[0] = rpyNow[0];
    busIn.rpy[1] = rpyNow[1];
    busIn.rpy[2] = rpyNow[2];
    busIn.baseAngVel[0] = angVelCopy[0];
    busIn.baseAngVel[1] = angVelCopy[1];
    busIn.baseAngVel[2] = angVelCopy[2];
    busIn.baseAcc[0] = accCopy[0];
    busIn.baseAcc[1] = accCopy[1];
    busIn.baseAcc[2] = accCopy[2];

    // Position/velocity are estimated later in StateEst; keep sensor-side base pose neutral here.
    busIn.basePos[0] = 0.0;
    busIn.basePos[1] = 0.0;
    busIn.basePos[2] = 0.0;
    busIn.baseLinVel[0] = 0.0;
    busIn.baseLinVel[1] = 0.0;
    busIn.baseLinVel[2] = 0.0;

    for (int i = 0; i < 12; i++)
    {
        busIn.motors_pos_cur[i] = posCopy[static_cast<size_t>(i)];
        busIn.motors_vel_cur[i] = velCopy[static_cast<size_t>(i)];
        busIn.motors_tor_cur[i] = effCopy[static_cast<size_t>(i)];
    }

    busIn.fL[0] = 0.0;
    busIn.fL[1] = 0.0;
    busIn.fL[2] = 0.0;
    busIn.fR[0] = 0.0;
    busIn.fR[1] = 0.0;
    busIn.fR[2] = 0.0;
    busIn.updateQ();
#else
    (void)busIn;
#endif
}

void ROS2_Interface_V4_Leg::setMotorsPosition(const std::vector<double> &qDesIn)
{
#if OPENLOONG_HAS_ROS2
    if (!isInitialized_ || actionCmdPub_ == nullptr)
    {
        return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(29, 0.0);
    const size_t n = std::min<size_t>(12, qDesIn.size());
    for (size_t i = 0; i < n; i++)
    {
        msg.data[i] = qDesIn[i];
    }
    actionCmdPub_->publish(msg);
#else
    (void)qDesIn;
#endif
}

#if OPENLOONG_HAS_ROS2
void ROS2_Interface_V4_Leg::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    quatWxyz_[0] = msg->orientation.w;
    quatWxyz_[1] = msg->orientation.x;
    quatWxyz_[2] = msg->orientation.y;
    quatWxyz_[3] = msg->orientation.z;
    baseAngVel_[0] = msg->angular_velocity.x;
    baseAngVel_[1] = msg->angular_velocity.y;
    baseAngVel_[2] = msg->angular_velocity.z;
    baseAcc_[0] = msg->linear_acceleration.x;
    baseAcc_[1] = msg->linear_acceleration.y;
    baseAcc_[2] = msg->linear_acceleration.z;
    lastImuTime_ = Clock::now();
    imuReceived_ = true;
}

void ROS2_Interface_V4_Leg::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::unordered_map<std::string, size_t> nameToIndex;
    nameToIndex.reserve(msg->name.size());
    for (size_t i = 0; i < msg->name.size(); i++)
    {
        nameToIndex[msg->name[i]] = i;
    }

    std::lock_guard<std::mutex> lock(dataMutex_);
    int foundCount = 0;
    for (size_t j = 0; j < kJointNamesPinOrder.size(); j++)
    {
        const auto it = nameToIndex.find(kJointNamesPinOrder[j]);
        if (it == nameToIndex.end())
        {
            continue;
        }
        const size_t idx = it->second;
        if (idx < msg->position.size())
        {
            motorsPos_[j] = msg->position[idx];
        }
        if (idx < msg->velocity.size())
        {
            motorsVel_[j] = msg->velocity[idx];
        }
        else
        {
            motorsVel_[j] = 0.0;
        }
        if (idx < msg->effort.size())
        {
            motorsEff_[j] = msg->effort[idx];
        }
        else
        {
            motorsEff_[j] = 0.0;
        }
        foundCount++;
    }

    if (foundCount >= 10)
    {
        jointStatesReceived_ = true;
        lastJointStatesTime_ = Clock::now();
    }
}

void ROS2_Interface_V4_Leg::quatToRpy(double qw, double qx, double qy, double qz, double rpyOut[3])
{
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    rpyOut[0] = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::fabs(sinp) >= 1.0)
    {
        rpyOut[1] = std::copysign(M_PI / 2.0, sinp);
    }
    else
    {
        rpyOut[1] = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    rpyOut[2] = std::atan2(siny_cosp, cosy_cosp);
}
#endif
