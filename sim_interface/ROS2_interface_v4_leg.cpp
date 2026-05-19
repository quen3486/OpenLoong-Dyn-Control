/*
 * ROS2 interface implementation for speedbot_v4 leg-only real robot backend.
 */
#include "ROS2_interface_v4_leg.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

#if OPENLOONG_HAS_ROS2
namespace
{
constexpr size_t kLegJointCount = 12;
constexpr size_t kV4JointCount = 22;
constexpr size_t kCommandJointCount = 23;
constexpr size_t kCommandSections = 3;
constexpr size_t kCommandSize = kCommandJointCount * kCommandSections;

constexpr std::array<const char *, kCommandJointCount> kCommandJointOrder = {
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint"};

constexpr std::array<double, kCommandJointCount> kDefaultCommandPosition{{0.0}};

static_assert(kCommandJointOrder.size() == kCommandJointCount, "command joint order size mismatch");
static_assert(kCommandSize == 69, "speedbot_v4 real command contract must stay 69 values");

bool hasFinitePrefix(const std::vector<double> &values, size_t count)
{
    if (values.size() < count)
    {
        return false;
    }
    for (size_t i = 0; i < count; i++)
    {
        if (!std::isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}
} // namespace

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
    std::cout << "[ROS2] action command contract: Float64MultiArray[69]=[pos23][vel23][torque23], "
              << kCommandJointOrder.front() << " ... " << kCommandJointOrder.back() << std::endl;
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

bool ROS2_Interface_V4_Leg::initializeCommandPublisherOnly(const ControllerConfig &config, std::string *errMsg)
{
#if OPENLOONG_HAS_ROS2
    topicActionCmd_ = config.rosTopicActionCmd;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("openloong_mpc_wbc_sim_real_openloop_cmd");
    actionCmdPub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topicActionCmd_, 20);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    imuSub_.reset();
    jointStatesSub_.reset();
    imuReceived_ = false;
    jointStatesReceived_ = false;
    isInitialized_ = true;

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    std::cout << "[ROS2] action command contract: Float64MultiArray[69]=[pos23][vel23][torque23], "
              << kCommandJointOrder.front() << " ... " << kCommandJointOrder.back() << std::endl;
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

void ROS2_Interface_V4_Leg::setMotorsCommand(const std::vector<double> &qDesIn,
                                             const std::vector<double> &dqDesIn,
                                             const std::vector<double> &tauFfIn)
{
#if OPENLOONG_HAS_ROS2
    (void)dqDesIn;
    (void)tauFfIn;
    if (!isInitialized_ || actionCmdPub_ == nullptr)
    {
        return;
    }

    const bool hasLegCommand = hasFinitePrefix(qDesIn, kLegJointCount);
    const bool hasV4Command = hasFinitePrefix(qDesIn, kV4JointCount);

    if (!hasLegCommand)
    {
        const size_t warnCount = ++commandInvalidWarnCount_;
        if (warnCount == 1 || warnCount % 1000 == 0)
        {
            std::cerr << "[ROS2] command publish skipped: expected finite position vector with at least 12 values"
                      << " for Float64MultiArray[69]=[pos23][vel23][torque23], got pos=" << qDesIn.size()
                      << ", topic=" << topicActionCmd_ << std::endl;
        }
        return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(kCommandSize, 0.0);
    for (size_t i = 0; i < kCommandJointCount; i++)
    {
        msg.data[i] = kDefaultCommandPosition[i];
    }
    for (size_t i = 0; i < kLegJointCount; i++)
    {
        msg.data[i] = qDesIn[i];
    }
    if (hasV4Command)
    {
        for (size_t i = 0; i < 5; i++)
        {
            msg.data[13 + i] = qDesIn[12 + i];
            msg.data[18 + i] = qDesIn[17 + i];
        }
    }
    actionCmdPub_->publish(msg);
#else
    (void)qDesIn;
    (void)dqDesIn;
    (void)tauFfIn;
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

    std::array<double, 12> posValues{{0.0}};
    std::array<double, 12> velValues{{0.0}};
    std::array<double, 12> effValues{{0.0}};
    std::array<size_t, 12> msgIndices{{0}};
    std::vector<std::string> issues;
    auto addIssue = [&issues](const std::string &issue)
    {
        if (issues.size() < 6)
        {
            issues.push_back(issue);
        }
    };

    bool valid = true;
    int validCount = 0;
    for (size_t j = 0; j < kJointNamesPinOrder.size(); j++)
    {
        const auto it = nameToIndex.find(kJointNamesPinOrder[j]);
        if (it == nameToIndex.end())
        {
            valid = false;
            addIssue("missing " + kJointNamesPinOrder[j]);
            continue;
        }
        const size_t idx = it->second;

        bool fieldsPresent = true;
        if (idx >= msg->position.size())
        {
            valid = false;
            fieldsPresent = false;
            addIssue(kJointNamesPinOrder[j] + " missing position");
        }
        if (idx >= msg->velocity.size())
        {
            valid = false;
            fieldsPresent = false;
            addIssue(kJointNamesPinOrder[j] + " missing velocity");
        }
        if (idx >= msg->effort.size())
        {
            valid = false;
            fieldsPresent = false;
            addIssue(kJointNamesPinOrder[j] + " missing effort");
        }
        if (!fieldsPresent)
        {
            continue;
        }

        const double q = msg->position[idx];
        const double dq = msg->velocity[idx];
        const double tau = msg->effort[idx];
        if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau))
        {
            valid = false;
            addIssue(kJointNamesPinOrder[j] + " contains NaN/Inf");
            continue;
        }

        posValues[j] = q;
        velValues[j] = dq;
        effValues[j] = tau;
        msgIndices[j] = idx;
        validCount++;
    }

    if (!valid || validCount != static_cast<int>(kJointNamesPinOrder.size()))
    {
        bool shouldWarn = false;
        size_t warnCount = 0;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            warnCount = ++jointStatesInvalidWarnCount_;
            shouldWarn = (warnCount == 1 || warnCount % 5000 == 0);
        }
        if (shouldWarn)
        {
            std::cerr << "[ROS2] ignored invalid " << topicJointStates_
                      << " feedback: ready " << validCount << "/"
                      << kJointNamesPinOrder.size()
                      << " joints with finite position/velocity/effort.";
            for (const std::string &issue : issues)
            {
                std::cerr << " " << issue << ";";
            }
            if (issues.size() >= 6)
            {
                std::cerr << " ...";
            }
            std::cerr << std::endl;
        }
        return;
    }

    bool shouldPrintMapping = false;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        motorsPos_ = posValues;
        motorsVel_ = velValues;
        motorsEff_ = effValues;
        jointStatesReceived_ = true;
        lastJointStatesTime_ = Clock::now();
        if (!jointStatesMappingPrinted_)
        {
            jointStatesMappingPrinted_ = true;
            shouldPrintMapping = true;
        }
    }

    if (shouldPrintMapping)
    {
        std::cout << "[ROS2] " << topicJointStates_
                  << " feedback ready: 12/12 joints, position/velocity/effort finite." << std::endl;
        std::cout << "[ROS2] " << topicJointStates_ << " -> MPC joint mapping:" << std::endl;
        for (size_t j = 0; j < kJointNamesPinOrder.size(); j++)
        {
            std::cout << "  mpc[" << j << "] " << kJointNamesPinOrder[j]
                      << " <- msg[" << msgIndices[j] << "]" << std::endl;
        }
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
