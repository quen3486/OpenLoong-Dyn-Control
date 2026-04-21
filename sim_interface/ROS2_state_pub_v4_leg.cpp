/*
 * ROS2 state publisher bridge implementation for MuJoCo simulation
 * (speedbot_v4 leg-only).
 */
#include "ROS2_state_pub_v4_leg.h"

#include <algorithm>
#include <cmath>

#if OPENLOONG_HAS_ROS2
const std::array<std::string, 12> ROS2_StatePub_V4_Leg::kJointNamesRosOrder = {
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_ankle_roll_joint", "left_ankle_pitch_joint", "right_hip_yaw_joint",
    "left_knee_joint", "right_knee_joint", "right_hip_roll_joint",
    "right_ankle_pitch_joint", "right_hip_pitch_joint", "right_ankle_roll_joint"};
const std::array<int, 12> ROS2_StatePub_V4_Leg::kRosToInternalIndex = {
    0, 1, 2, 5, 4, 7, 3, 9, 6, 10, 8, 11};
#endif

ROS2_StatePub_V4_Leg::ROS2_StatePub_V4_Leg() = default;

ROS2_StatePub_V4_Leg::~ROS2_StatePub_V4_Leg()
{
#if OPENLOONG_HAS_ROS2
    if (rclcpp::ok() && node_ != nullptr && executor_ != nullptr)
    {
        executor_->remove_node(node_);
    }
#endif
}

bool ROS2_StatePub_V4_Leg::initialize(const ControllerConfig &config, std::string *errMsg)
{
#if OPENLOONG_HAS_ROS2
    topicImu_ = config.rosTopicImu;
    topicJointStates_ = config.rosTopicJointStates;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("openloong_mpc_wbc_leg_mujoco_state_pub");
    imuPub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topicImu_, rclcpp::SensorDataQoS());
    jointStatesPub_ = node_->create_publisher<sensor_msgs::msg::JointState>(topicJointStates_, 50);
#if OPENLOONG_HAS_TF2
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
#endif
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

void ROS2_StatePub_V4_Leg::spinSome()
{
#if OPENLOONG_HAS_ROS2
    if (isInitialized_ && executor_ != nullptr)
    {
        executor_->spin_some();
    }
#endif
}

void ROS2_StatePub_V4_Leg::publishState(const DataBus &busIn)
{
#if OPENLOONG_HAS_ROS2
    if (!isInitialized_ || node_ == nullptr || imuPub_ == nullptr || jointStatesPub_ == nullptr)
    {
        return;
    }

    const auto stamp = node_->get_clock()->now();

    sensor_msgs::msg::Imu imuMsg;
    imuMsg.header.stamp = stamp;
    imuMsg.header.frame_id = "imu_link";

    double quatWxyz[4]{1.0, 0.0, 0.0, 0.0};
    rpyToQuat(busIn.rpy[0], busIn.rpy[1], busIn.rpy[2], quatWxyz);
    imuMsg.orientation.w = quatWxyz[0];
    imuMsg.orientation.x = quatWxyz[1];
    imuMsg.orientation.y = quatWxyz[2];
    imuMsg.orientation.z = quatWxyz[3];
    imuMsg.angular_velocity.x = busIn.baseAngVel[0];
    imuMsg.angular_velocity.y = busIn.baseAngVel[1];
    imuMsg.angular_velocity.z = busIn.baseAngVel[2];
    imuMsg.linear_acceleration.x = busIn.baseAcc[0];
    imuMsg.linear_acceleration.y = busIn.baseAcc[1];
    imuMsg.linear_acceleration.z = busIn.baseAcc[2];
    imuPub_->publish(imuMsg);

    sensor_msgs::msg::JointState jointMsg;
    jointMsg.header.stamp = stamp;
    jointMsg.name.assign(kJointNamesRosOrder.begin(), kJointNamesRosOrder.end());
    jointMsg.position.assign(12, 0.0);
    jointMsg.velocity.assign(12, 0.0);
    jointMsg.effort.assign(12, 0.0);

    const size_t nPos = busIn.motors_pos_cur.size();
    const size_t nVel = busIn.motors_vel_cur.size();
    const size_t nEff = busIn.motors_tor_cur.size();
    for (size_t i = 0; i < kRosToInternalIndex.size(); i++)
    {
        const size_t src = static_cast<size_t>(kRosToInternalIndex[i]);
        if (src < nPos)
        {
            jointMsg.position[i] = busIn.motors_pos_cur[src];
        }
        if (src < nVel)
        {
            jointMsg.velocity[i] = busIn.motors_vel_cur[src];
        }
        if (src < nEff)
        {
            jointMsg.effort[i] = busIn.motors_tor_cur[src];
        }
    }
    jointStatesPub_->publish(jointMsg);

#if OPENLOONG_HAS_TF2
    if (tfBroadcaster_ != nullptr)
    {
        geometry_msgs::msg::TransformStamped tfMsg;
        tfMsg.header.stamp = stamp;
        tfMsg.header.frame_id = "world";
        tfMsg.child_frame_id = "base_link";
        tfMsg.transform.translation.x = busIn.basePos[0];
        tfMsg.transform.translation.y = busIn.basePos[1];
        tfMsg.transform.translation.z = busIn.basePos[2];
        tfMsg.transform.rotation.w = quatWxyz[0];
        tfMsg.transform.rotation.x = quatWxyz[1];
        tfMsg.transform.rotation.y = quatWxyz[2];
        tfMsg.transform.rotation.z = quatWxyz[3];
        tfBroadcaster_->sendTransform(tfMsg);
    }
#endif
#else
    (void)busIn;
#endif
}

#if OPENLOONG_HAS_ROS2
void ROS2_StatePub_V4_Leg::rpyToQuat(double roll, double pitch, double yaw, double quatWxyz[4])
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    quatWxyz[0] = cr * cp * cy + sr * sp * sy;
    quatWxyz[1] = sr * cp * cy - cr * sp * sy;
    quatWxyz[2] = cr * sp * cy + sr * cp * sy;
    quatWxyz[3] = cr * cp * sy - sr * sp * cy;
}
#endif
