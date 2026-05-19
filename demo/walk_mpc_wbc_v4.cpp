/*
 * walk_mpc_wbc_v4: MPC + WBC walking demo for speedbot_v4 robot.
 */
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface_v4.h"
#include "ROS2_interface_v4_leg.h"
#include "PVT_ctrl_v4.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn_v4.h"
#include "useful_math.h"
#include "wbc_priority_v4.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include "controller_config.h"
#include "json/json.h"
#include "weld_trajectory.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cctype>
#include <chrono>
#include <future>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>
#include "StateEst.h"

char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/scene_v4.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

namespace
{

std::string toLowerCopy(std::string in)
{
    for (char &ch : in)
    {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return in;
}

bool readBoolEnv(const char *name, bool fallback)
{
    const char *env = std::getenv(name);
    if (env == nullptr)
    {
        return fallback;
    }
    const std::string v = toLowerCopy(std::string(env));
    if (v == "1" || v == "true" || v == "yes" || v == "on")
    {
        return true;
    }
    if (v == "0" || v == "false" || v == "no" || v == "off")
    {
        return false;
    }
    return fallback;
}

unsigned int readUIntEnv(const char *name, unsigned int fallback)
{
    const char *env = std::getenv(name);
    if (env == nullptr || std::string(env).empty())
    {
        return fallback;
    }
    char *endPtr = nullptr;
    const unsigned long value = std::strtoul(env, &endPtr, 10);
    return (endPtr == env) ? fallback : static_cast<unsigned int>(value);
}

double readDoubleEnv(const char *name, double fallback)
{
    const char *env = std::getenv(name);
    if (env == nullptr || std::string(env).empty())
    {
        return fallback;
    }
    char *endPtr = nullptr;
    const double value = std::strtod(env, &endPtr);
    return (endPtr == env || !std::isfinite(value)) ? fallback : value;
}

bool hasArg(int argc, char **argv, const std::string &arg)
{
    for (int i = 1; i < argc; i++)
    {
        if (argv[i] != nullptr && arg == argv[i])
        {
            return true;
        }
    }
    return false;
}

const std::array<std::string, 22> kV4CommandSourceJointNames = {
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint"};

struct V4OpenloopJointLimit
{
    std::string name;
    double minPos{-3.14};
    double maxPos{3.14};
};

bool loadV4OpenloopJointLimits(std::vector<V4OpenloopJointLimit> &outLimits,
                               std::string &loadedPath,
                               std::string &errMsg)
{
    const std::array<std::string, 3> candidates = {
        "../common/joint_ctrl_config_v4.json",
        "common/joint_ctrl_config_v4.json",
        "joint_ctrl_config_v4.json"};

    Json::Value root;
    std::string parseErr;
    for (const std::string &path : candidates)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in.is_open())
        {
            continue;
        }
        Json::CharReaderBuilder builder;
        builder["allowComments"] = true;
        builder["collectComments"] = false;
        if (!Json::parseFromStream(builder, in, &root, &parseErr))
        {
            errMsg = "failed to parse " + path + ": " + parseErr;
            return false;
        }
        loadedPath = path;
        break;
    }

    if (loadedPath.empty())
    {
        errMsg = "failed to open joint_ctrl_config_v4.json";
        return false;
    }

    outLimits.clear();
    outLimits.reserve(kV4CommandSourceJointNames.size());
    for (const std::string &name : kV4CommandSourceJointNames)
    {
        if (!root.isMember(name))
        {
            errMsg = "joint config missing " + name;
            return false;
        }
        const Json::Value &joint = root[name];
        if (!joint.isMember("minPos") || !joint["minPos"].isNumeric() ||
            !joint.isMember("maxPos") || !joint["maxPos"].isNumeric())
        {
            errMsg = "joint config " + name + " missing numeric minPos/maxPos";
            return false;
        }
        V4OpenloopJointLimit limit;
        limit.name = name;
        limit.minPos = joint["minPos"].asDouble();
        limit.maxPos = joint["maxPos"].asDouble();
        outLimits.push_back(limit);
    }
    errMsg.clear();
    return true;
}

class V4OpenloopSafetyMonitor
{
public:
    explicit V4OpenloopSafetyMonitor(std::vector<V4OpenloopJointLimit> limits)
        : limits_(std::move(limits))
    {
        enabled_ = readBoolEnv("REAL_SAFETY_ENABLE", true);
        cmdJumpLimitRad_ = std::max(0.001, readDoubleEnv("REAL_SAFETY_CMD_JUMP_LIMIT_RAD", cmdJumpLimitRad_));
    }

    void printConfig() const
    {
        std::cout << "[Safety-V4SimOpenLoop] " << (enabled_ ? "enabled" : "disabled")
                  << ", cmd_jump_limit=" << cmdJumpLimitRad_
                  << " rad, checked_joints=" << limits_.size() << std::endl;
    }

    void resetCommandHistory()
    {
        hasLastCommand_ = false;
        lastCommand_.clear();
    }

    bool validateCommandAndRemember(const std::vector<double> &qDes, std::string &reason)
    {
        if (qDes.size() < kV4CommandSourceJointNames.size())
        {
            reason = "position command vector invalid: fewer than 22 values";
            return false;
        }
        for (size_t i = 0; i < kV4CommandSourceJointNames.size(); i++)
        {
            if (!std::isfinite(qDes[i]))
            {
                reason = "position command vector invalid: contains NaN/Inf";
                return false;
            }
        }
        if (!enabled_)
        {
            return true;
        }
        if (limits_.size() != kV4CommandSourceJointNames.size())
        {
            reason = "joint limit table invalid";
            return false;
        }
        for (size_t i = 0; i < limits_.size(); i++)
        {
            if (qDes[i] < limits_[i].minPos || qDes[i] > limits_[i].maxPos)
            {
                std::ostringstream oss;
                oss << "joint command position over limit: " << limits_[i].name
                    << ", q_des=" << qDes[i]
                    << ", range=[" << limits_[i].minPos << ", " << limits_[i].maxPos << "]";
                reason = oss.str();
                return false;
            }
        }
        if (hasLastCommand_)
        {
            double maxJump = 0.0;
            for (size_t i = 0; i < limits_.size(); i++)
            {
                maxJump = std::max(maxJump, std::fabs(qDes[i] - lastCommand_[i]));
            }
            if (maxJump > cmdJumpLimitRad_)
            {
                std::ostringstream oss;
                oss << "command jump over limit: max_jump=" << maxJump;
                reason = oss.str();
                return false;
            }
        }
        lastCommand_.assign(qDes.begin(), qDes.begin() + kV4CommandSourceJointNames.size());
        hasLastCommand_ = true;
        return true;
    }

private:
    bool enabled_{true};
    bool hasLastCommand_{false};
    double cmdJumpLimitRad_{0.25};
    std::vector<V4OpenloopJointLimit> limits_;
    std::vector<double> lastCommand_;
};

int defaultWeldStanceThreadCount()
{
    const unsigned int hw = std::thread::hardware_concurrency();
    const int available = (hw > 2) ? static_cast<int>(hw) - 2 : 1;
    return std::clamp(available, 1, 4);
}

double smoothStep01(double x)
{
    const double u = std::clamp(x, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

double smoothStep01Dot(double x)
{
    const double u = std::clamp(x, 0.0, 1.0);
    return 6.0 * u * (1.0 - u);
}

double smoothStep01Ddot(double x)
{
    if (x <= 0.0 || x >= 1.0)
    {
        return 0.0;
    }
    return 6.0 - 12.0 * x;
}

bool isWeldMotionState(DataBus::MotionState state)
{
    return state == DataBus::WeldPrepare || state == DataBus::Weld ||
           state == DataBus::WeldHold || state == DataBus::WeldRecover;
}

const char *motionStateName(DataBus::MotionState state)
{
    switch (state)
    {
    case DataBus::Stand:
        return "Stand";
    case DataBus::Walk:
        return "Walk";
    case DataBus::Walk2Stand:
        return "Walk2Stand";
    case DataBus::WeldPrepare:
        return "WeldPrepare";
    case DataBus::Weld:
        return "Weld";
    case DataBus::WeldHold:
        return "WeldHold";
    case DataBus::WeldRecover:
        return "WeldRecover";
    default:
        return "Unknown";
    }
}

struct WeldStanceCandidate
{
    Eigen::Vector3d basePos{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double standOff{0.0};
    double sideBias{0.0};
    double yawOffset{0.0};
    double footX{0.0};
    double width{0.0};
    double stagger{0.0};
    double footYCenter{0.0};
    int leftArmIndex{0};
    Eigen::Vector3d footL_W{Eigen::Vector3d::Zero()};
    Eigen::Vector3d footR_W{Eigen::Vector3d::Zero()};
    Eigen::VectorXd leftArmQ;
    Eigen::VectorXd qFixed;
    double score{std::numeric_limits<double>::infinity()};
    long long orderIndex{std::numeric_limits<long long>::max()};
    double maxIkErr{std::numeric_limits<double>::infinity()};
    double rmsIkErr{std::numeric_limits<double>::infinity()};
    double minLimitMargin{0.0};
    double comMargin{0.0};
    double armMotion{0.0};
    double clearanceMargin{0.0};
    double preApproachClearance{0.0};
};

struct WeldStanceResult
{
    bool valid{false};
    bool clearanceFallbackUsed{false};
    bool broadFallbackUsed{false};
    bool fallbackValid{false};
    WeldStanceCandidate best;
    WeldStanceCandidate fallbackBest;
    int candidates{0};
    int bodyCandidates{0};
    int validCandidates{0};
    int clearancePassCandidates{0};
    int leftArmLightEvaluations{0};
    int leftArmLightRejects{0};
    int refineCandidates{0};
    int legIkFail{0};
    int legLimitFail{0};
    int armIkFail{0};
    int armLimitFail{0};
    int comFail{0};
    int clearanceFail{0};
    int preApproachFail{0};
    int baseHeightFail{0};
    int broadFallbackCandidates{0};
    int threadCount{1};
    double totalWallTime{0.0};
    double coarseWallTime{0.0};
    double broadFallbackWallTime{0.0};
    double refineWallTime{0.0};
    double leftArmLightWallTime{0.0};
};

struct WeldStanceGridPoint
{
    Eigen::Vector3d basePos_W{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double standOff{0.0};
    double sideBias{0.0};
    double yawOffset{0.0};
    double footX{0.0};
    double width{0.0};
    double stagger{0.0};
    double footYCenter{0.0};
    int leftArmIndex{0};
    long long orderIndex{0};
};

bool isBetterWeldStanceCandidate(const WeldStanceCandidate &candidate,
                                 const WeldStanceCandidate &current)
{
    constexpr double eps = 1.0e-12;
    if (candidate.score < current.score - eps)
    {
        return true;
    }
    if (std::abs(candidate.score - current.score) <= eps)
    {
        return candidate.orderIndex < current.orderIndex;
    }
    return false;
}

void mergeWeldStanceResult(WeldStanceResult &dst, const WeldStanceResult &src)
{
    dst.candidates += src.candidates;
    dst.bodyCandidates += src.bodyCandidates;
    dst.validCandidates += src.validCandidates;
    dst.clearancePassCandidates += src.clearancePassCandidates;
    dst.leftArmLightEvaluations += src.leftArmLightEvaluations;
    dst.leftArmLightRejects += src.leftArmLightRejects;
    dst.legIkFail += src.legIkFail;
    dst.legLimitFail += src.legLimitFail;
    dst.armIkFail += src.armIkFail;
    dst.armLimitFail += src.armLimitFail;
    dst.comFail += src.comFail;
    dst.clearanceFail += src.clearanceFail;
    dst.preApproachFail += src.preApproachFail;
    dst.baseHeightFail += src.baseHeightFail;
    dst.leftArmLightWallTime += src.leftArmLightWallTime;

    if (src.fallbackValid &&
        (!dst.fallbackValid || isBetterWeldStanceCandidate(src.fallbackBest, dst.fallbackBest)))
    {
        dst.fallbackValid = true;
        dst.fallbackBest = src.fallbackBest;
    }
    if (src.valid &&
        (!dst.valid || isBetterWeldStanceCandidate(src.best, dst.best)))
    {
        dst.valid = true;
        dst.best = src.best;
    }
}

Eigen::Matrix3d yawRot(double yaw)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    R << c, -s, 0.0,
         s,  c, 0.0,
         0.0, 0.0, 1.0;
    return R;
}

double jointLimitMargin(const pinocchio::Model &model, const Eigen::VectorXd &qFixed, int begin, int count)
{
    double margin = std::numeric_limits<double>::infinity();
    for (int i = begin; i < begin + count; ++i)
    {
        if (i >= qFixed.size() || i >= model.lowerPositionLimit.size() || i >= model.upperPositionLimit.size())
        {
            continue;
        }
        const double lower = model.lowerPositionLimit(i);
        const double upper = model.upperPositionLimit(i);
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper)
        {
            continue;
        }
        margin = std::min(margin, std::min(qFixed(i) - lower, upper - qFixed(i)));
    }
    return std::isfinite(margin) ? margin : 1.0;
}

double computeSupportMargin(const Eigen::Vector3d &com_B,
                            const Eigen::Vector3d &footL_B,
                            const Eigen::Vector3d &footR_B,
                            const ControllerConfig &config)
{
    const double xMin = std::min(footL_B.x() - config.mpcDeltaFootRear, footR_B.x() - config.mpcDeltaFootRear);
    const double xMax = std::max(footL_B.x() + config.mpcDeltaFootFront, footR_B.x() + config.mpcDeltaFootFront);
    const double yMin = std::min(footL_B.y() - config.mpcDeltaFootRight, footR_B.y() - config.mpcDeltaFootRight);
    const double yMax = std::max(footL_B.y() + config.mpcDeltaFootLeft, footR_B.y() + config.mpcDeltaFootLeft);
    return std::min({com_B.x() - xMin, xMax - com_B.x(), com_B.y() - yMin, yMax - com_B.y()});
}

constexpr double weldArmRadius = 0.035;
constexpr double weldArmClearanceThreshold = 0.03;
constexpr double weldTcpContactExclusionLength = 0.10;
constexpr int weldClearanceSegmentSamples = 16;
constexpr double weldBaseZStabilityMin = 0.98;
constexpr double weldPreApproachDistDefault = 0.08;
constexpr double weldMinPreApproachClearanceDefault = 0.06;
constexpr double weldHoldDurationDefault = 0.8;
constexpr double weldRecoverDurationDefault = 3.0;
constexpr double weldStanceTransitionDurationDefault = 5.0;
constexpr double weldPrepareHoldTDefault = 0.2;
constexpr double weldPrepareMaxTDefault = 3.0;
constexpr double weldPrepareMinPhaseDefault = 0.95;
constexpr double weldPrepareStartErrDefault = 0.002;
constexpr double weldPrepareFallbackErrDefault = 0.002;
constexpr double weldTcpFinishErrDefault = 0.002;
constexpr double weldFinishTimeoutDefault = 1.0;
constexpr double weldTcpLookaheadDefault = 0.07;
constexpr double weldHandKpDefault = 120.0;
constexpr double weldHandKdDefault = 24.0;
constexpr double weldArmDeltaQLimitDefault = 0.018;
constexpr double weldArmDqLimitDefault = 0.60;
constexpr double weldArmDdqLimitDefault = 10.0;
constexpr double weldLeftArmMomentumGainDefault = 0.02;
constexpr double weldLeftArmVelLimitDefault = 0.10;

struct WeldWorkpieceRuntime
{
    Eigen::Vector3d nominalPlateCenter_W{0.48, 0.0, 1.20};
    Eigen::Vector3d plateHalfExtents{0.012, 0.16, 0.13};
    Eigen::Vector3d nominalPathStart_W{0.465, -0.12, 1.20};
    Eigen::Vector3d nominalPathEnd_W{0.465, 0.12, 1.20};
    Eigen::Vector3d approachDir_W{-1.0, 0.0, 0.0};
    Eigen::Vector3d offset_W{Eigen::Vector3d::Zero()};
    Eigen::Vector3d randomRange_W{0.06, 0.10, 0.03};
    unsigned int seed{20260506U};
    bool randomEnabled{true};

    Eigen::Vector3d plateCenter_W() const { return nominalPlateCenter_W + offset_W; }
    Eigen::Vector3d pathStart_W() const { return nominalPathStart_W + offset_W; }
    Eigen::Vector3d pathEnd_W() const { return nominalPathEnd_W + offset_W; }
    Eigen::Vector3d pathCenter_W() const { return 0.5 * (pathStart_W() + pathEnd_W()); }
    Eigen::Vector3d nominalPathCenter_W() const { return 0.5 * (nominalPathStart_W + nominalPathEnd_W); }
};

WeldWorkpieceRuntime makeWeldWorkpieceRuntime()
{
    WeldWorkpieceRuntime workpiece;
    workpiece.randomEnabled = true;
    workpiece.seed = readUIntEnv("WELD_WORKPIECE_SEED", workpiece.seed);
    if (workpiece.randomEnabled)
    {
        std::mt19937 rng(workpiece.seed);
        std::uniform_real_distribution<double> dx(-workpiece.randomRange_W.x(), workpiece.randomRange_W.x());
        std::uniform_real_distribution<double> dy(-workpiece.randomRange_W.y(), workpiece.randomRange_W.y());
        std::uniform_real_distribution<double> dz(-workpiece.randomRange_W.z(), workpiece.randomRange_W.z());
        workpiece.offset_W << dx(rng), dy(rng), dz(rng);
    }
    return workpiece;
}

void applyWeldWorkpieceToMujoco(mjModel *model,
                                mjData *data,
                                const WeldWorkpieceRuntime &workpiece)
{
    const int bodyId = mj_name2id(model, mjOBJ_BODY, "weld_workpiece");
    if (bodyId < 0)
    {
        std::cerr << "[WeldWorkpiece] warning: body weld_workpiece not found; visual marker not moved." << std::endl;
        return;
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        model->body_pos[3 * bodyId + axis] = workpiece.offset_W(axis);
    }
    mj_forward(model, data);
}

void printWeldWorkpieceRuntime(const WeldWorkpieceRuntime &workpiece)
{
    std::cout << "[WeldWorkpiece] random=" << (workpiece.randomEnabled ? 1 : 0)
              << ", seed=" << workpiece.seed
              << ", range_xyz=" << workpiece.randomRange_W.transpose()
              << ", offset_xyz=" << workpiece.offset_W.transpose() << std::endl;
    std::cout << "[WeldWorkpiece] plate_center=" << workpiece.plateCenter_W().transpose()
              << ", path_start=" << workpiece.pathStart_W().transpose()
              << ", path_end=" << workpiece.pathEnd_W().transpose()
              << ", approach_dir=" << workpiece.approachDir_W.transpose() << std::endl;
}

Eigen::Vector3d normalizedOr(const Eigen::Vector3d &vec, const Eigen::Vector3d &fallback)
{
    const double norm = vec.norm();
    return (norm > 1.0e-9) ? vec / norm : fallback;
}

double pointAabbDistance(const Eigen::Vector3d &point,
                         const Eigen::Vector3d &center,
                         const Eigen::Vector3d &halfExtents)
{
    const Eigen::Vector3d delta = (point - center).cwiseAbs() - halfExtents;
    return delta.cwiseMax(Eigen::Vector3d::Zero()).norm();
}

bool segmentIntersectsAabb(const Eigen::Vector3d &start,
                           const Eigen::Vector3d &end,
                           const Eigen::Vector3d &center,
                           const Eigen::Vector3d &halfExtents)
{
    const Eigen::Vector3d minCorner = center - halfExtents;
    const Eigen::Vector3d maxCorner = center + halfExtents;
    const Eigen::Vector3d direction = end - start;
    double tMin = 0.0;
    double tMax = 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (std::abs(direction(axis)) < 1e-9)
        {
            if (start(axis) < minCorner(axis) || start(axis) > maxCorner(axis))
            {
                return false;
            }
            continue;
        }
        double t1 = (minCorner(axis) - start(axis)) / direction(axis);
        double t2 = (maxCorner(axis) - start(axis)) / direction(axis);
        if (t1 > t2)
        {
            std::swap(t1, t2);
        }
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax)
        {
            return false;
        }
    }
    return true;
}

double segmentCapsuleAabbClearance(const Eigen::Vector3d &start,
                                   const Eigen::Vector3d &end,
                                   const Eigen::Vector3d &center,
                                   const Eigen::Vector3d &halfExtents)
{
    if ((end - start).norm() < 1e-6)
    {
        return pointAabbDistance(start, center, halfExtents) - weldArmRadius;
    }
    if (segmentIntersectsAabb(start, end, center, halfExtents))
    {
        return -weldArmRadius;
    }
    double minDistance = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= weldClearanceSegmentSamples; ++i)
    {
        const double alpha = static_cast<double>(i) / static_cast<double>(weldClearanceSegmentSamples);
        const Eigen::Vector3d point = (1.0 - alpha) * start + alpha * end;
        minDistance = std::min(minDistance, pointAabbDistance(point, center, halfExtents));
    }
    return minDistance - weldArmRadius;
}

double computeRightArmWorkpieceClearance(Pin_KinDyn_V4 &kinDynSolver,
                                         const Eigen::VectorXd &qFixed,
                                         const Eigen::Vector3d &basePos_W,
                                         const Eigen::Matrix3d &R_WB,
                                         const WeldWorkpieceRuntime &workpiece)
{
    if (qFixed.size() != kinDynSolver.model_biped_fixed.nq)
    {
        return 0.0;
    }

    const auto keypoints_B = kinDynSolver.computeRightArmKeypointsFixed(qFixed);
    std::array<Eigen::Vector3d, 5> keypoints_W{};
    for (int i = 0; i < static_cast<int>(keypoints_B.size()); ++i)
    {
        keypoints_W[i] = basePos_W + R_WB * keypoints_B[i];
    }

    const Eigen::Vector3d center = workpiece.plateCenter_W();
    const Eigen::Vector3d halfExtents = workpiece.plateHalfExtents;
    double margin = std::numeric_limits<double>::infinity();

    margin = std::min(margin, segmentCapsuleAabbClearance(keypoints_W[0], keypoints_W[1], center, halfExtents));
    margin = std::min(margin, segmentCapsuleAabbClearance(keypoints_W[1], keypoints_W[2], center, halfExtents));
    margin = std::min(margin, segmentCapsuleAabbClearance(keypoints_W[2], keypoints_W[3], center, halfExtents));

    const Eigen::Vector3d tool = keypoints_W[4] - keypoints_W[3];
    const double toolLength = tool.norm();
    if (toolLength > weldTcpContactExclusionLength + 0.02)
    {
        const Eigen::Vector3d clearanceEnd =
            keypoints_W[3] + tool * ((toolLength - weldTcpContactExclusionLength) / toolLength);
        margin = std::min(margin, segmentCapsuleAabbClearance(keypoints_W[3], clearanceEnd, center, halfExtents));
    }

    return std::isfinite(margin) ? margin : 0.0;
}

void evaluateWeldStanceCandidate(Pin_KinDyn_V4 &kinDynSolver,
                                  const std::array<Eigen::Vector3d, 11> &weldSamples_W,
                                  const WeldWorkpieceRuntime &workpiece,
                                  const ControllerConfig &controllerConfig,
                                  double footHeight,
                                  double preApproachDist,
                                  double minPreApproachClearance,
                                  double bx,
                                  double by,
                                  double bz,
                                  double yaw,
                                  double standOff,
                                  double sideBias,
                                  double yawOffset,
                                  double footX,
                                  double width,
                                  double stagger,
                                  double footYCenter,
                                  int leftArmIndex,
                                  const Eigen::VectorXd &leftArmQ,
                                  const std::vector<Eigen::VectorXd> &leftArmCandidates,
                                  long long orderIndex,
                                  WeldStanceResult &result)
{
    const Eigen::Matrix3d footRot_B = Eigen::Matrix3d::Identity();
    const Eigen::VectorXd neutralLeftArm = (Eigen::VectorXd(5) << 0.3, 1.4, 0.0, -1.4, 0.0).finished();
    const Eigen::VectorXd neutralRightArm = (Eigen::VectorXd(5) << -0.3, -1.4, 0.0, 1.4, 0.0).finished();

    result.candidates++;
    result.bodyCandidates++;
    if (bz < weldBaseZStabilityMin)
    {
        result.baseHeightFail++;
        return;
    }
    const Eigen::Matrix3d R_WB = yawRot(yaw);
    const Eigen::Vector3d basePos_W(bx, by, bz);
    const Eigen::Vector3d footL_B(footX + 0.5 * stagger, footYCenter + 0.5 * width, footHeight - bz);
    const Eigen::Vector3d footR_B(footX - 0.5 * stagger, footYCenter - 0.5 * width, footHeight - bz);
    auto legIk = kinDynSolver.computeInK_Leg(footRot_B, footL_B, footRot_B, footR_B);
    if (legIk.status != 0 || legIk.err.norm() > 2e-3)
    {
        result.legIkFail++;
        return;
    }

    Eigen::VectorXd qFixed = legIk.jointPosRes;
    qFixed.segment<5>(12) = neutralLeftArm;
    qFixed.segment<5>(17) = neutralRightArm;
    const double legMargin = jointLimitMargin(kinDynSolver.model_biped_fixed, qFixed, 0, 12);
    const double leftArmMargin = jointLimitMargin(kinDynSolver.model_biped_fixed, qFixed, 12, 5);
    if (legMargin < 0.08)
    {
        result.legLimitFail++;
        return;
    }
    if (leftArmMargin < 0.12)
    {
        result.armLimitFail++;
        return;
    }

    const Eigen::Vector3d com_B = kinDynSolver.computeFixedCoM(qFixed);
    const double comMargin = computeSupportMargin(com_B, footL_B, footR_B, controllerConfig);
    if (comMargin < 0.035)
    {
        result.comFail++;
        return;
    }

    const Eigen::Vector3d approachDir_W =
        normalizedOr(workpiece.approachDir_W, Eigen::Vector3d(-1.0, 0.0, 0.0));
    const Eigen::Vector3d preApproachTcp_W = weldSamples_W.front() + approachDir_W * preApproachDist;
    auto preApproachIk = kinDynSolver.computeRightHandPosIK(R_WB.transpose() * (preApproachTcp_W - basePos_W), qFixed);
    if (preApproachIk.err.norm() > 0.03)
    {
        result.armIkFail++;
        return;
    }
    const Eigen::VectorXd qPreApproach = preApproachIk.jointPosRes;
    const Eigen::Vector3d actualPreApproachTcp_W =
        basePos_W + R_WB * kinDynSolver.computeRightHandPosFixed(qPreApproach);
    const double preApproachClearance =
        pointAabbDistance(actualPreApproachTcp_W, workpiece.plateCenter_W(), workpiece.plateHalfExtents);
    if (preApproachClearance < minPreApproachClearance)
    {
        result.preApproachFail++;
        return;
    }

    Eigen::VectorXd qSeq = qPreApproach;
    double maxIkErr = 0.0;
    double sumIkErr2 = 0.0;
    double armMargin = std::numeric_limits<double>::infinity();
    double clearanceMargin = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(weldSamples_W.size()); ++i)
    {
        const Eigen::Vector3d target_B = R_WB.transpose() * (weldSamples_W[i] - basePos_W);
        auto armIk = kinDynSolver.computeRightHandPosIK(target_B, qSeq);
        const double errNorm = armIk.err.norm();
        if (errNorm > 0.03)
        {
            result.armIkFail++;
            return;
        }
        qSeq = armIk.jointPosRes;
        maxIkErr = std::max(maxIkErr, errNorm);
        sumIkErr2 += errNorm * errNorm;
        armMargin = std::min(armMargin, jointLimitMargin(kinDynSolver.model_biped_fixed, qSeq, 17, 5));
        clearanceMargin = std::min(clearanceMargin,
                                   computeRightArmWorkpieceClearance(kinDynSolver, qSeq, basePos_W, R_WB, workpiece));
    }

    if (armMargin < 0.12)
    {
        result.armLimitFail++;
        return;
    }

    const double rmsIkErr = std::sqrt(sumIkErr2 / static_cast<double>(weldSamples_W.size()));
    const double armMotion = (qSeq.segment<5>(17) - qPreApproach.segment<5>(17)).norm();
    const double legPostureCost = qPreApproach.segment<12>(0).norm();
    const double boundedClearance = std::clamp(clearanceMargin, -0.05, 0.12);
    const double baseLoweringReward = std::clamp(1.02 - bz, 0.0, 0.12);
    const double standOffCost = std::abs(standOff - 0.38);
    const double sideBiasCost = std::abs(sideBias - 0.20);
    const double baseScore = 1000.0 * maxIkErr + 300.0 * rmsIkErr + 8.0 * armMotion
                             - 40.0 * boundedClearance - 55.0 * baseLoweringReward
                             - 10.0 * std::clamp(preApproachClearance, 0.0, 0.12)
                             + 8.0 * standOffCost + 6.0 * sideBiasCost
                             + 4.0 * std::abs(width - 0.30)
                             + 4.0 * std::abs(footX) + 4.0 * std::abs(stagger)
                             + 5.0 * std::abs(footYCenter)
                             + 2.0 * legPostureCost + 3.0 * std::abs(yawOffset);

    const auto leftArmStart = std::chrono::steady_clock::now();
    const std::vector<Eigen::VectorXd> fallbackLeftArmCandidates = {
        (leftArmQ.size() == 5 ? leftArmQ : neutralLeftArm)};
    const auto &leftCandidates = leftArmCandidates.empty() ? fallbackLeftArmCandidates : leftArmCandidates;
    bool anyLightValid = false;
    for (int candidateIndex = 0; candidateIndex < static_cast<int>(leftCandidates.size()); ++candidateIndex)
    {
        result.leftArmLightEvaluations++;
        const Eigen::VectorXd &candidateLeftArmQ =
            (leftCandidates[candidateIndex].size() == 5) ? leftCandidates[candidateIndex] : neutralLeftArm;
        Eigen::VectorXd qCandidate = qPreApproach;
        qCandidate.segment<5>(12) = candidateLeftArmQ;

        const double candidateLeftArmMargin =
            jointLimitMargin(kinDynSolver.model_biped_fixed, qCandidate, 12, 5);
        if (candidateLeftArmMargin < 0.12)
        {
            result.leftArmLightRejects++;
            continue;
        }

        const Eigen::Vector3d candidateCom_B = kinDynSolver.computeFixedCoM(qCandidate);
        const double candidateComMargin =
            computeSupportMargin(candidateCom_B, footL_B, footR_B, controllerConfig);
        if (candidateComMargin < 0.035)
        {
            result.leftArmLightRejects++;
            continue;
        }

        const double leftArmPostureCost = (candidateLeftArmQ - neutralLeftArm).norm();
        const double leftArmBalanceReward = std::clamp(leftArmPostureCost, 0.0, 0.45);
        const double minLimitMargin = std::min({legMargin, armMargin, candidateLeftArmMargin});
        const double score = baseScore - 120.0 * minLimitMargin - 70.0 * candidateComMargin
                             - 4.0 * leftArmBalanceReward + 0.4 * leftArmPostureCost;

        result.validCandidates++;
        anyLightValid = true;
        WeldStanceCandidate candidate;
        candidate.basePos = basePos_W;
        candidate.yaw = yaw;
        candidate.standOff = standOff;
        candidate.sideBias = sideBias;
        candidate.yawOffset = yawOffset;
        candidate.footX = footX;
        candidate.width = width;
        candidate.stagger = stagger;
        candidate.footYCenter = footYCenter;
        candidate.leftArmIndex = candidateIndex;
        candidate.footL_W = basePos_W + R_WB * footL_B;
        candidate.footR_W = basePos_W + R_WB * footR_B;
        candidate.leftArmQ = candidateLeftArmQ;
        candidate.qFixed = qCandidate;
        candidate.score = score;
        candidate.orderIndex = orderIndex * 1000 + candidateIndex;
        candidate.maxIkErr = maxIkErr;
        candidate.rmsIkErr = rmsIkErr;
        candidate.minLimitMargin = minLimitMargin;
        candidate.comMargin = candidateComMargin;
        candidate.armMotion = armMotion;
        candidate.clearanceMargin = clearanceMargin;
        candidate.preApproachClearance = preApproachClearance;

        if (!result.fallbackValid || isBetterWeldStanceCandidate(candidate, result.fallbackBest))
        {
            result.fallbackValid = true;
            result.fallbackBest = candidate;
        }
        if (clearanceMargin < weldArmClearanceThreshold)
        {
            result.clearanceFail++;
            continue;
        }
        result.clearancePassCandidates++;
        if (!result.valid || isBetterWeldStanceCandidate(candidate, result.best))
        {
            result.valid = true;
            result.best = candidate;
        }
    }
    if (!anyLightValid)
    {
        result.comFail++;
    }
    result.leftArmLightWallTime +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - leftArmStart).count();
}

WeldStanceResult selectWeldStance(Pin_KinDyn_V4 &kinDynSolver,
                                  const WeldTrajectory &trajectory,
                                  const WeldWorkpieceRuntime &workpiece,
                                  const ControllerConfig &controllerConfig,
                                  double footHeight,
                                  double preApproachDist,
                                  double minPreApproachClearance,
                                  int requestedThreadCount)
{
    WeldStanceResult result;
    result.threadCount = std::max(1, requestedThreadCount);
    const auto totalStart = std::chrono::steady_clock::now();
    if (!trajectory.valid())
    {
        return result;
    }

    std::array<Eigen::Vector3d, 11> weldSamples_W;
    for (int i = 0; i < static_cast<int>(weldSamples_W.size()); ++i)
    {
        const double t = trajectory.totalDuration() * static_cast<double>(i) /
                         static_cast<double>(weldSamples_W.size() - 1);
        weldSamples_W[i] = trajectory.sample(t).pose.pos;
    }

    const std::vector<Eigen::VectorXd> leftArmCandidates = []()
    {
        std::vector<Eigen::VectorXd> candidates;
        auto addCandidate = [&](double shoulderPitch, double shoulderRoll, double shoulderYaw,
                                double elbow, double wristRoll)
        {
            candidates.push_back((Eigen::VectorXd(5) << shoulderPitch, shoulderRoll, shoulderYaw,
                                  elbow, wristRoll)
                                     .finished());
        };
        const double elbowNeutral = -1.4;
        const double wristNeutral = 0.0;
        addCandidate(0.30, 1.40, 0.00, elbowNeutral, wristNeutral);
        for (double shoulderPitch : {-0.25, 0.85})
        {
            addCandidate(shoulderPitch, 1.40, 0.00, elbowNeutral, wristNeutral);
        }
        for (double shoulderRoll : {0.95, 1.85})
        {
            addCandidate(0.30, shoulderRoll, 0.00, elbowNeutral, wristNeutral);
        }
        for (double shoulderYaw : {-0.60, 0.60})
        {
            addCandidate(0.30, 1.40, shoulderYaw, elbowNeutral, wristNeutral);
        }
        for (double shoulderPitch : {-0.25, 0.85})
            for (double shoulderRoll : {0.95, 1.85})
                for (double shoulderYaw : {-0.60, 0.60})
                {
                    const double elbow = (shoulderRoll > 1.4) ? -1.25 : -1.55;
                    const double wrist = (shoulderYaw > 0.0) ? -0.15 : 0.15;
                    addCandidate(shoulderPitch, shoulderRoll, shoulderYaw, elbow, wrist);
        }
        return candidates;
    }();
    const std::vector<int> bodySearchLeftArmIndices = {0};

    Eigen::Vector3d weldCenter_W = Eigen::Vector3d::Zero();
    for (const auto &sample : weldSamples_W)
    {
        weldCenter_W += sample;
    }
    weldCenter_W /= static_cast<double>(weldSamples_W.size());
    const Eigen::Vector3d approachNormal_W =
        normalizedOr(workpiece.approachDir_W, Eigen::Vector3d(-1.0, 0.0, 0.0));
    const double nominalYaw = std::atan2(-approachNormal_W.y(), -approachNormal_W.x());
    const double weldHeightDelta = weldCenter_W.z() - workpiece.nominalPathCenter_W().z();
    const double nominalBaseZ =
        std::clamp(0.99 + weldHeightDelta, weldBaseZStabilityMin, 1.03);
    const std::vector<double> coarseBaseZs = {
        std::max(weldBaseZStabilityMin, nominalBaseZ - 0.01),
        nominalBaseZ + 0.01};
    const std::vector<double> broadBaseZs = {
        std::max(weldBaseZStabilityMin, nominalBaseZ - 0.03),
        nominalBaseZ,
        std::min(1.04, nominalBaseZ + 0.03)};
    std::cout << "[WeldStance] search envelope: weld_center=" << weldCenter_W.transpose()
              << ", nominal_base_z=" << nominalBaseZ
              << ", coarse_base_z=" << coarseBaseZs.front() << "/" << coarseBaseZs.back()
              << ", standOff={0.38,0.46,0.54}, sideBias={0.24,0.32,0.40}" << std::endl;
    long long nextOrderIndex = 0;

    auto elapsedSeconds = [](const std::chrono::steady_clock::time_point &start)
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    };

    auto buildRelativeGrid = [&](const std::vector<double> &standOffs,
                                 const std::vector<double> &sideBiases,
                                 const std::vector<double> &baseZs,
                                 const std::vector<double> &yawOffsets,
                                 const std::vector<double> &footXs,
                                 const std::vector<double> &widths,
                                 const std::vector<double> &staggers,
                                 const std::vector<double> &footYCenters,
                                 const std::vector<int> &leftArmIndices)
    {
        std::vector<WeldStanceGridPoint> grid;
        const size_t expectedCount = standOffs.size() * sideBiases.size() * baseZs.size() *
                                     yawOffsets.size() * footXs.size() * widths.size() *
                                     staggers.size() * footYCenters.size() * leftArmIndices.size();
        grid.reserve(expectedCount);
        for (double standOff : standOffs)
            for (double sideBias : sideBiases)
                for (double baseZ : baseZs)
                    for (double yawOffset : yawOffsets)
                        for (double footX : footXs)
                            for (double width : widths)
                                for (double stagger : staggers)
                                    for (double footYCenter : footYCenters)
                                        for (int leftArmIndex : leftArmIndices)
                                        {
                                            const double yaw = nominalYaw + yawOffset;
                                            const Eigen::Vector3d bodyY_W = yawRot(yaw).col(1);
                                            WeldStanceGridPoint point;
                                            point.basePos_W =
                                                weldCenter_W + approachNormal_W * standOff + bodyY_W * sideBias;
                                            point.basePos_W.z() = baseZ;
                                            point.yaw = yaw;
                                            point.standOff = standOff;
                                            point.sideBias = sideBias;
                                            point.yawOffset = yawOffset;
                                            point.footX = footX;
                                            point.width = width;
                                            point.stagger = stagger;
                                            point.footYCenter = footYCenter;
                                            point.leftArmIndex =
                                                std::clamp(leftArmIndex, 0,
                                                           static_cast<int>(leftArmCandidates.size()) - 1);
                                            point.orderIndex = nextOrderIndex++;
                                            grid.push_back(point);
                                        }
        return grid;
    };

    auto evaluateGridRange = [&](Pin_KinDyn_V4 &solver,
                                 const std::vector<WeldStanceGridPoint> &grid,
                                 size_t begin,
                                 size_t end)
    {
        WeldStanceResult localResult;
        for (size_t gridIndex = begin; gridIndex < end; ++gridIndex)
        {
            const WeldStanceGridPoint &point = grid[gridIndex];
            evaluateWeldStanceCandidate(solver, weldSamples_W, workpiece, controllerConfig, footHeight,
                                        preApproachDist, minPreApproachClearance,
                                        point.basePos_W.x(), point.basePos_W.y(), point.basePos_W.z(), point.yaw,
                                        point.standOff, point.sideBias, point.yawOffset,
                                        point.footX, point.width, point.stagger,
                                        point.footYCenter, point.leftArmIndex,
                                        leftArmCandidates[point.leftArmIndex],
                                        leftArmCandidates,
                                        point.orderIndex, localResult);
        }
        return localResult;
    };

    auto evaluateGrid = [&](const std::vector<WeldStanceGridPoint> &grid)
    {
        WeldStanceResult phaseResult;
        if (grid.empty())
        {
            return phaseResult;
        }
        const int workerCount = std::min(result.threadCount, static_cast<int>(grid.size()));
        if (workerCount <= 1)
        {
            return evaluateGridRange(kinDynSolver, grid, 0, grid.size());
        }

        std::vector<std::future<WeldStanceResult>> futures;
        futures.reserve(workerCount);
        for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
        {
            const size_t begin = grid.size() * static_cast<size_t>(workerIndex) /
                                 static_cast<size_t>(workerCount);
            const size_t end = grid.size() * static_cast<size_t>(workerIndex + 1) /
                               static_cast<size_t>(workerCount);
            futures.emplace_back(std::async(std::launch::async,
                                            [&, begin, end]()
                                            {
                                                Pin_KinDyn_V4 workerSolver("../models/speedbot_v4/speedbot_v4.urdf");
                                                return evaluateGridRange(workerSolver, grid, begin, end);
                                            }));
        }
        for (auto &future : futures)
        {
            mergeWeldStanceResult(phaseResult, future.get());
        }
        return phaseResult;
    };

    auto runRelativeGrid = [&](const std::vector<double> &standOffs,
                               const std::vector<double> &sideBiases,
                               const std::vector<double> &baseZs,
                               const std::vector<double> &yawOffsets,
                               const std::vector<double> &footXs,
                               const std::vector<double> &widths,
                               const std::vector<double> &staggers,
                               const std::vector<double> &footYCenters,
                               const std::vector<int> &leftArmIndices)
    {
        const std::vector<WeldStanceGridPoint> grid =
            buildRelativeGrid(standOffs, sideBiases, baseZs, yawOffsets,
                              footXs, widths, staggers, footYCenters, leftArmIndices);
        return evaluateGrid(grid);
    };

    auto phaseStart = std::chrono::steady_clock::now();
    WeldStanceResult coarseResult =
        runRelativeGrid({0.38, 0.46, 0.54},
                        {0.24, 0.32, 0.40},
                        coarseBaseZs,
                        {-0.08, 0.0, 0.08},
                        {0.03, 0.09},
                        {0.36, 0.42},
                        {-0.12, -0.06},
                        {0.0},
                        bodySearchLeftArmIndices);
    result.coarseWallTime = elapsedSeconds(phaseStart);
    mergeWeldStanceResult(result, coarseResult);

    if (!result.valid)
    {
        result.broadFallbackUsed = true;
        phaseStart = std::chrono::steady_clock::now();
        WeldStanceResult broadResult =
            runRelativeGrid({0.30, 0.62},
                            {0.12, 0.48},
                            broadBaseZs,
                            {-0.18, 0.18},
                            {0.03, 0.09},
                            {0.36, 0.42},
                            {-0.12, -0.06},
                            {0.0},
                            bodySearchLeftArmIndices);
        result.broadFallbackWallTime = elapsedSeconds(phaseStart);
        result.broadFallbackCandidates = broadResult.candidates;
        mergeWeldStanceResult(result, broadResult);
    }

    if (result.fallbackValid)
    {
        const WeldStanceCandidate refineSeed = result.valid ? result.best : result.fallbackBest;
        phaseStart = std::chrono::steady_clock::now();
        WeldStanceResult refineResult =
            runRelativeGrid({std::max(0.16, refineSeed.standOff - 0.04), refineSeed.standOff, refineSeed.standOff + 0.04},
                            {std::max(0.0, refineSeed.sideBias - 0.04), refineSeed.sideBias, refineSeed.sideBias + 0.04},
                            {std::max(weldBaseZStabilityMin, refineSeed.basePos.z() - 0.01),
                             refineSeed.basePos.z() + 0.01},
                            {refineSeed.yawOffset - 0.04, refineSeed.yawOffset, refineSeed.yawOffset + 0.04},
                            {refineSeed.footX - 0.03, refineSeed.footX + 0.03},
                            {std::max(0.22, refineSeed.width - 0.02), refineSeed.width, std::min(0.42, refineSeed.width + 0.02)},
                            {refineSeed.stagger - 0.02, refineSeed.stagger, refineSeed.stagger + 0.02},
                            {refineSeed.footYCenter},
                            bodySearchLeftArmIndices);
        result.refineWallTime = elapsedSeconds(phaseStart);
        result.refineCandidates = refineResult.candidates;
        mergeWeldStanceResult(result, refineResult);
    }

    if (!result.valid && result.fallbackValid)
    {
        result.valid = true;
        result.clearanceFallbackUsed = true;
        result.best = result.fallbackBest;
    }

    result.totalWallTime = elapsedSeconds(totalStart);
    return result;
}

void applyInitialPoseToMujoco(mjModel *model,
                              mjData *data,
                              const Eigen::Vector3d &basePos,
                              double yaw,
                              const Eigen::VectorXd &qFixed)
{
    data->qpos[0] = basePos.x();
    data->qpos[1] = basePos.y();
    data->qpos[2] = basePos.z();
    data->qpos[3] = std::cos(0.5 * yaw);
    data->qpos[4] = 0.0;
    data->qpos[5] = 0.0;
    data->qpos[6] = std::sin(0.5 * yaw);
    const int nJoint = std::min(static_cast<int>(qFixed.size()), model->nq - 7);
    for (int i = 0; i < nJoint; ++i)
    {
        data->qpos[7 + i] = qFixed(i);
    }
    for (int i = 0; i < model->nv; ++i)
    {
        data->qvel[i] = 0.0;
    }
    for (int i = 0; i < model->nu; ++i)
    {
        data->ctrl[i] = 0.0;
    }
    mj_forward(model, data);
}

double computeFootCopMargin(const Eigen::Ref<const Eigen::VectorXd> &wrench_W,
                            const Eigen::Matrix3d &footRot_W,
                            const ControllerConfig &config)
{
    if (wrench_W.size() < 6)
    {
        return -1.0;
    }

    Eigen::Matrix<double, 6, 1> wrench_L;
    wrench_L.head<3>() = footRot_W.transpose() * wrench_W.head<3>();
    wrench_L.tail<3>() = footRot_W.transpose() * wrench_W.tail<3>();
    const double fz = wrench_L(2);
    if (fz <= 1e-6)
    {
        return -1.0;
    }

    const double copX = -wrench_L(4) / fz;
    const double copY = wrench_L(3) / fz;
    return std::min({config.mpcDeltaFootFront - copX,
                     copX + config.mpcDeltaFootRear,
                     config.mpcDeltaFootLeft - copY,
                     copY + config.mpcDeltaFootRight});
}

double computeDoubleSupportCopMargin(const DataBus &robotState,
                                     const ControllerConfig &config)
{
    if (robotState.wbc_FrRes.size() < 12)
    {
        return 0.0;
    }
    const double leftMargin = computeFootCopMargin(robotState.wbc_FrRes.segment(0, 6),
                                                   robotState.fe_l_rot_W, config);
    const double rightMargin = computeFootCopMargin(robotState.wbc_FrRes.segment(6, 6),
                                                    robotState.fe_r_rot_W, config);
    return std::min(leftMargin, rightMargin);
}

double computeJointTorqueMargin(const Eigen::VectorXd &tauCmd,
                                const Eigen::VectorXd &tauMaxAbs)
{
    if (tauCmd.size() == 0 || tauCmd.size() != tauMaxAbs.size())
    {
        return 0.0;
    }
    double margin = std::numeric_limits<double>::infinity();
    for (int i = 0; i < tauCmd.size(); ++i)
    {
        margin = std::min(margin, std::abs(tauMaxAbs(i)) - std::abs(tauCmd(i)));
    }
    return std::isfinite(margin) ? margin : 0.0;
}

Eigen::VectorXd makeMujocoQpos(int nq,
                               const Eigen::Vector3d &basePos,
                               double yaw,
                               const Eigen::VectorXd &qFixed)
{
    Eigen::VectorXd qpos = Eigen::VectorXd::Zero(nq);
    qpos(0) = basePos.x();
    qpos(1) = basePos.y();
    qpos(2) = basePos.z();
    qpos(3) = std::cos(0.5 * yaw);
    qpos(4) = 0.0;
    qpos(5) = 0.0;
    qpos(6) = std::sin(0.5 * yaw);
    const int nJoint = std::min(static_cast<int>(qFixed.size()), nq - 7);
    for (int i = 0; i < nJoint; ++i)
    {
        qpos(7 + i) = qFixed(i);
    }
    return qpos;
}

Eigen::VectorXd readMujocoQpos(const mjModel *model, const mjData *data)
{
    Eigen::VectorXd qpos(model->nq);
    for (int i = 0; i < model->nq; ++i)
    {
        qpos(i) = data->qpos[i];
    }
    return qpos;
}

void writeMujocoQpos(mjModel *model, mjData *data, Eigen::VectorXd qpos)
{
    if (qpos.size() != model->nq)
    {
        return;
    }
    if (qpos.size() >= 7)
    {
        Eigen::Vector4d quat = qpos.segment<4>(3);
        const double quatNorm = quat.norm();
        if (quatNorm > 1.0e-9 && std::isfinite(quatNorm))
        {
            quat /= quatNorm;
        }
        else
        {
            quat << 1.0, 0.0, 0.0, 0.0;
        }
        qpos.segment<4>(3) = quat;
    }
    for (int i = 0; i < model->nq; ++i)
    {
        data->qpos[i] = qpos(i);
    }
    for (int i = 0; i < model->nv; ++i)
    {
        data->qvel[i] = 0.0;
    }
    for (int i = 0; i < model->nu; ++i)
    {
        data->ctrl[i] = 0.0;
    }
    mj_forward(model, data);
}

}

int main(int argc, char **argv)
{
    // initialize classes
    UIctr uiController(mj_model, mj_data);
    MJ_Interface_V4 mj_interface(mj_model, mj_data);
    Pin_KinDyn_V4 kinDynSolver("../models/speedbot_v4/speedbot_v4.urdf");

    ControllerConfig controllerConfig;
    controllerConfig.forwardSpeedDefault = 0.4;
    controllerConfig.speedStep = 0.1;
    controllerConfig.speedMax = 1.2;
    controllerConfig.speedMin = 0.0;
    controllerConfig.turnRateCmd = 0.2;
    const char *cfgEnv = std::getenv("CONTROLLER_CONFIG");
    const std::string cfgPath = (cfgEnv != nullptr && std::string(cfgEnv).size() > 0)
                                    ? std::string(cfgEnv)
                                    : std::string("../common/controller_config_v4.json");
    std::cout << "[ControllerConfig] loading: " << cfgPath << std::endl;
    std::string controllerConfigErr;
    if (!loadControllerConfig(cfgPath, controllerConfig, &controllerConfigErr))
    {
        std::cerr << "[ControllerConfig] fallback to built-in defaults: " << controllerConfigErr << std::endl;
    }
    const bool simRealOpenloopRequested = hasArg(argc, argv, "--sim-real-openloop");
    const double simDt = mj_model->opt.timestep;
    const int mainCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mainControlDt / simDt)));
    const double mainCtrlDt = simDt * mainCtrlDecimation;
    const int mpcCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mpcControlDt / mainCtrlDt)));
    const double mpcCtrlDt = mainCtrlDt * mpcCtrlDecimation;
    std::cout << "[LoopRate] sim=" << 1.0 / simDt << " Hz, main=" << 1.0 / mainCtrlDt
              << " Hz, mpc=" << 1.0 / mpcCtrlDt << " Hz" << std::endl;

    DataBus RobotState(kinDynSolver.model_nv);
    WBC_priority_V4 WBC_solv(kinDynSolver.model_nv, 18, 44, 0.7, mainCtrlDt);
    WBC_solv.setJointTorqueLimits(-kinDynSolver.motorMaxTorque, kinDynSolver.motorMaxTorque);
    MPC MPC_solv(mpcCtrlDt);
    GaitScheduler gaitScheduler(0.4, mainCtrlDt);
    PVT_Ctr_V4 pvtCtr(mainCtrlDt, "../common/joint_ctrl_config_v4.json");
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(mainCtrlDt);
    DataLogger logger("../record/datalog.log");
    StateEst StateModule(mainCtrlDt);
    ROS2_Interface_V4_Leg simRealCommandPub;
    std::unique_ptr<V4OpenloopSafetyMonitor> simRealCommandSafety;
    bool simRealOpenloopEnabled = false;
    bool simRealCommandPublishEnabled = false;
    bool simRealCommandSafetyStopped = false;
    int simRealCommandPubCount = 0;
    const int simRealCommandPubDecimation =
        std::max(1, static_cast<int>(std::lround(controllerConfig.simRosPublishDt / mainCtrlDt)));
    const double realCommandInitialRampTimeSec = 5.0;
    const double realWeldStanceRampTimeSec = weldStanceTransitionDurationDefault;
    std::vector<double> simRealLastPublishedPos;
    std::vector<double> simRealRampStartPos(kV4CommandSourceJointNames.size(), 0.0);
    double simRealRampElapsed = 0.0;
    bool realWeldStanceRampActive = false;
    double realWeldStanceRampElapsed = 0.0;
    std::vector<double> realWeldStanceRampStart(kV4CommandSourceJointNames.size(), 0.0);
    std::vector<double> realWeldStanceRampTarget(kV4CommandSourceJointNames.size(), 0.0);
    const std::vector<double> simRealZeroVel(kV4CommandSourceJointNames.size(), 0.0);
    const std::vector<double> simRealZeroTau(kV4CommandSourceJointNames.size(), 0.0);
    Eigen::Matrix3d mpcInertiaCfg;
    mpcInertiaCfg << controllerConfig.mpcInertiaXx, controllerConfig.mpcInertiaXy, controllerConfig.mpcInertiaXz,
                     controllerConfig.mpcInertiaXy, controllerConfig.mpcInertiaYy, controllerConfig.mpcInertiaYz,
                     controllerConfig.mpcInertiaXz, controllerConfig.mpcInertiaYz, controllerConfig.mpcInertiaZz;
    MPC_solv.setRobotMass(controllerConfig.mpcMass);
    MPC_solv.setFrictionCoeff(controllerConfig.contactMiu);
    if (!MPC_solv.setBodyInertia(mpcInertiaCfg))
    {
        std::cerr << "[MPC] invalid inertia config, keeping previous inertia matrix." << std::endl;
    }
    MPC_solv.setUseDataBusInertia(controllerConfig.mpcUseDataBusInertia);
    MPC_solv.setFootSupportPolygon(controllerConfig.mpcDeltaFootFront, controllerConfig.mpcDeltaFootRear,
                                   controllerConfig.mpcDeltaFootLeft, controllerConfig.mpcDeltaFootRight);
    MPC_solv.setWrenchLimits(controllerConfig.mpcForceMaxXY, controllerConfig.mpcFzMaxScale,
                             controllerConfig.mpcTorqueMaxX, controllerConfig.mpcTorqueMaxY, controllerConfig.mpcTorqueMaxZ);
    MPC_solv.setHorizon(controllerConfig.mpcPredictionHorizon, controllerConfig.mpcControlHorizon);
    WBC_solv.setContactMiu(controllerConfig.contactMiu);

    if (simRealOpenloopRequested)
    {
        std::string simRealRosErr;
        if (!simRealCommandPub.initializeCommandPublisherOnly(controllerConfig, &simRealRosErr))
        {
            std::cerr << "[ROS2-OpenLoop-V4] command publisher initialization failed: "
                      << simRealRosErr << std::endl;
            return 1;
        }

        std::vector<V4OpenloopJointLimit> jointLimits;
        std::string jointLimitPath;
        std::string jointLimitErr;
        if (!loadV4OpenloopJointLimits(jointLimits, jointLimitPath, jointLimitErr))
        {
            std::cerr << "[Safety-V4SimOpenLoop] failed to load joint limits: "
                      << jointLimitErr << std::endl;
            return 1;
        }
        simRealCommandSafety = std::make_unique<V4OpenloopSafetyMonitor>(jointLimits);
        simRealCommandSafety->printConfig();
        simRealOpenloopEnabled = true;
        simRealCommandPubCount = simRealCommandPubDecimation - 1;
        std::cout << "[ROS2-OpenLoop-V4] ready, publish_dt="
                  << simRealCommandPubDecimation * mainCtrlDt
                  << " s, action_topic=" << controllerConfig.rosTopicActionCmd << std::endl;
        std::cout << "[Command-V4SimOpenLoop] publishes Float64MultiArray[69]=[pos23][zero_vel23][zero_torque23]; "
                  << "V4 source [leg12][left_arm5][right_arm5], command waist_yaw=0." << std::endl;
        std::cout << "[PublishGate-V4SimOpenLoop] startup publishes nothing. Press P to start/stop real command publishing." << std::endl;
    }

    WeldWorkpieceRuntime weldWorkpiece;
    bool weldWorkpieceActivated = false;

    WeldTrajectory nominalWeldTrajectory;
    WeldTrajectory weldTrajectory;
    const char *weldEnv = std::getenv("WELD_TRAJECTORY");
    const std::string weldPath = (weldEnv != nullptr && std::string(weldEnv).size() > 0)
                                     ? std::string(weldEnv)
                                     : std::string("../common/weld_trajectory_v4.csv");
    std::string weldLoadErr;
    const bool nominalWeldTrajectoryReady = nominalWeldTrajectory.loadCsv(weldPath, &weldLoadErr);
    bool weldTrajectoryReady = false;
    RobotState.weld_trajectory_valid = weldTrajectoryReady;
    if (nominalWeldTrajectoryReady)
    {
        const auto weldStartSample = nominalWeldTrajectory.sample(0.0);
        const auto weldEndSample = nominalWeldTrajectory.sample(nominalWeldTrajectory.totalDuration());
        std::cout << "[WeldTrajectory] nominal loaded: " << nominalWeldTrajectory.path()
                  << ", segments=" << nominalWeldTrajectory.segmentCount()
                  << ", length=" << nominalWeldTrajectory.totalLength()
                  << " m, duration=" << nominalWeldTrajectory.totalDuration() << " s" << std::endl;
        std::cout << "[WeldTrajectory] nominal path start/end: "
                  << weldStartSample.pose.pos.transpose() << " / "
                  << weldEndSample.pose.pos.transpose() << std::endl;
    }
    else
    {
        std::cerr << "[WeldTrajectory] disabled: " << weldLoadErr << std::endl;
    }
    const double weldPreApproachDist = weldPreApproachDistDefault;
    const double weldMinPreApproachClearance = weldMinPreApproachClearanceDefault;
    const double weldHoldDuration = weldHoldDurationDefault;
    const double weldRecoverDuration = weldRecoverDurationDefault;
    Eigen::Vector3d weldPreApproachTcp_W = Eigen::Vector3d::Zero();
    std::cout << "[WeldWorkpiece] startup: nominal workpiece; interactive weld preparation waits for G."
              << std::endl;
    std::cout << "[Weld] preapproach=" << weldPreApproachDist
              << " m, min_clearance=" << weldMinPreApproachClearance
              << " m, hold=" << weldHoldDuration
              << " s, recover=" << weldRecoverDuration << " s" << std::endl;

    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking();
    uiController.createWindow("Demo_V4", false);
    uiController.setGeomGroupVisible(0, false);
    UIctr::ButtonState buttonState;
    std::cout << "[Mode] startup: open-loop stand. F=closed-loop Stand, Space=Walk, "
              << "G=prepare/start weld in Stand." << std::endl;
    if (simRealOpenloopEnabled)
    {
        std::cout << "[Key-V4SimOpenLoop] P(publish real command on/off) G(weld prepare/start) "
                  << "F(closed-loop) Space(stand/walk) W/S/A/D(move) Q/E(speed) J(stop) H(reset yaw)"
                  << std::endl;
    }

    // initialize variables
    // speedbot_v4: leg length ~0.983m, use 0.95 for slight bend; foot height ~0.053m     
    double stand_legLength = 0.95;  // desired baselink height
    double foot_height = 0.053;     // distance between the foot ankel joint and the bottom
    double xv_des = controllerConfig.forwardSpeedDefault; // desired velocity in x direction
    double xv_step = controllerConfig.speedStep;          // speed increment per key press
    double xv_max = controllerConfig.speedMax;            // speed magnitude upper bound
    double xv_min = controllerConfig.speedMin;            // speed magnitude lower bound
    const double turnRateCmd = controllerConfig.turnRateCmd;

    gaitScheduler.tSwing = controllerConfig.tSwing;
    gaitScheduler.phiSwitchMin = controllerConfig.phiSwitchMin;
    gaitScheduler.phiSwitchAutoDesign = controllerConfig.phiSwitchAutoDesign;
    gaitScheduler.phiSwitchDesignRefTSwing = controllerConfig.phiSwitchDesignRefTSwing;
    gaitScheduler.phiSwitchDesignRef = controllerConfig.phiSwitchDesignRef;
    gaitScheduler.phiSwitchDesignPower = controllerConfig.phiSwitchDesignPower;
    gaitScheduler.phiSwitchDesignMin = controllerConfig.phiSwitchDesignMin;
    gaitScheduler.phiSwitchDesignMax = controllerConfig.phiSwitchDesignMax;
    gaitScheduler.fzSwitchThreshold = controllerConfig.fzSwitchThreshold;
    gaitScheduler.fzStopThreshold = controllerConfig.fzStopThreshold;

    const int robot_nq = kinDynSolver.model_nv + 1;
    const int robot_nv = robot_nq - 1;

    RobotState.width_hips = 0.245;
    footPlacement.kp_vx = controllerConfig.kpVx;
    footPlacement.kp_vy = controllerConfig.kpVy;
    footPlacement.kp_wz = controllerConfig.kpWz;
    footPlacement.stepHeight = controllerConfig.stepHeight;
    footPlacement.legLength = stand_legLength;
    footPlacement.xOffsetL = controllerConfig.xOffsetL;
    footPlacement.yOffsetL = controllerConfig.yOffsetL;
    footPlacement.zOffsetW = controllerConfig.zOffsetW;
    footPlacement.swingTrajectoryPhase = controllerConfig.swingTrajectoryPhase;
    footPlacement.swingTrajectoryWindow = controllerConfig.swingTrajectoryWindow;
    footPlacement.zStretchStartPhi = controllerConfig.zStretchStartPhi;
    footPlacement.zStretchStep = controllerConfig.zStretchStep;
    footPlacement.zStretchMin = controllerConfig.zStretchMin;

    WBC_solv.cfg_pos_err_clamp_xy = controllerConfig.posErrClampXY;
    WBC_solv.cfg_pos_err_clamp_z = controllerConfig.posErrClampZ;
    WBC_solv.cfg_posrot_kp = controllerConfig.posRotKp;
    WBC_solv.cfg_posrot_kd = controllerConfig.posRotKd;
    WBC_solv.cfg_posrot_kp_x = controllerConfig.posRotKpX;
    WBC_solv.cfg_posrot_kp_pitch = controllerConfig.posRotKpPitch;
    WBC_solv.cfg_posrot_kd_pitch = controllerConfig.posRotKdPitch;
    WBC_solv.cfg_swing_kp = controllerConfig.swingLegKp;
    WBC_solv.cfg_swing_kd = controllerConfig.swingLegKd;
    WBC_solv.cfg_weld_hand_kp = weldHandKpDefault;
    WBC_solv.cfg_weld_hand_kd = weldHandKdDefault;
    WBC_solv.cfg_weld_arm_delta_q_limit = weldArmDeltaQLimitDefault;
    WBC_solv.cfg_weld_arm_dq_limit = weldArmDqLimitDefault;
    WBC_solv.cfg_weld_arm_ddq_limit = weldArmDdqLimitDefault;
    WBC_solv.cfg_weld_left_arm_momentum_gain = weldLeftArmMomentumGainDefault;
    WBC_solv.cfg_weld_left_arm_vel_limit = weldLeftArmVelLimitDefault;
    std::cout << "[WeldWBC] hand_kp/kd=" << WBC_solv.cfg_weld_hand_kp
              << "/" << WBC_solv.cfg_weld_hand_kd
              << ", arm_limits(delta_q,dq,ddq)=" << WBC_solv.cfg_weld_arm_delta_q_limit
              << ", " << WBC_solv.cfg_weld_arm_dq_limit
              << ", " << WBC_solv.cfg_weld_arm_ddq_limit
              << ", left_gain/vel_limit=" << WBC_solv.cfg_weld_left_arm_momentum_gain
              << "/" << WBC_solv.cfg_weld_left_arm_vel_limit << std::endl;

    std::vector<double> motors_pos_des(robot_nv - 6, 0);
    std::vector<double> motors_pos_cur(robot_nv - 6, 0);
    std::vector<double> motors_vel_des(robot_nv - 6, 0);
    std::vector<double> motors_vel_cur(robot_nv - 6, 0);
    std::vector<double> motors_tau_des(robot_nv - 6, 0);
    std::vector<double> motors_tau_cur(robot_nv - 6, 0);

    // ini position for foot-end and hand
    // speedbot_v4: hip y offset ≈ ±0.1225, x offset ≈ 0
    Eigen::Vector3d fe_l_pos_L_des = {0, 0.1225, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {0, -0.1225, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {0, 0, 0};
    Eigen::Vector3d fe_r_eul_L_des = {0, 0, 0};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    // arm initial pose: 5 DoF per arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
    Eigen::VectorXd hd_l_des, hd_r_des;
    hd_l_des.resize(5);
    hd_r_des.resize(5);
    hd_l_des << 0.3, 1.4, 0, -1.4, 0;
    hd_r_des << -0.3, -1.4, 0, 1.4, 0;

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes(0) = 0.0;
    qIniDes(1) = 0.0;
    qIniDes(2) = stand_legLength + foot_height;
    qIniDes(3) = 0.0;
    qIniDes(4) = 0.0;
    qIniDes(5) = 0.0;
    qIniDes(6) = 1.0;
    qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes;
    // Overwrite arm joints in q-space:
    // arm_l: fixed indices 12-16 → q[19..23]
    // arm_r: fixed indices 17-21 → q[24..28]
    qIniDes.block(19, 0, 5, 1) = hd_l_des;
    qIniDes.block(24, 0, 5, 1) = hd_r_des;
    Eigen::VectorXd qIniFixedDes = qIniDes.block(7, 0, robot_nv - 6, 1);
    writeMujocoQpos(mj_model, mj_data,
                    makeMujocoQpos(mj_model->nq,
                                   Eigen::Vector3d(qIniDes(0), qIniDes(1), qIniDes(2)),
                                   0.0, qIniFixedDes));
    std::cout << "[WeldWorkpiece] initial visualization pose: nominal robot stand, nominal workpiece."
              << std::endl;

    const char *weldStanceModeEnv = std::getenv("WELD_STANCE_MODE");
    const std::string weldStanceMode = (weldStanceModeEnv != nullptr && std::string(weldStanceModeEnv).size() > 0)
                                           ? toLowerCopy(std::string(weldStanceModeEnv))
                                           : std::string("auto");
    const bool weldStanceAuto = (weldStanceMode != "nominal");
    const bool weldReapplyOnCloseLoop = true;
    WeldStanceResult weldStanceResult;
    bool weldStanceReady = !weldStanceAuto;
    Eigen::Vector4d weldStanceBaseXyzYaw;
    weldStanceBaseXyzYaw << qIniDes(0), qIniDes(1), qIniDes(2), 0.0;
    double weldStanceScore = 0.0;
    double weldStanceMaxIkErr = 0.0;
    double weldStanceMinLimitMargin = 0.0;
    double weldStanceComMargin = 0.0;
    double weldStanceClearanceMargin = 0.0;
    double weldPreApproachClearance = weldTrajectoryReady
                                          ? pointAabbDistance(weldPreApproachTcp_W,
                                                              weldWorkpiece.plateCenter_W(),
                                                              weldWorkpiece.plateHalfExtents)
                                          : 0.0;
    Eigen::Vector3d weldStanceFootL_W =
        Eigen::Vector3d(qIniDes(0), qIniDes(1), qIniDes(2)) + fe_l_pos_L_des;
    Eigen::Vector3d weldStanceFootR_W =
        Eigen::Vector3d(qIniDes(0), qIniDes(1), qIniDes(2)) + fe_r_pos_L_des;
    Eigen::VectorXd weldStanceLeftArmQ = qIniFixedDes.segment<5>(12);
    Eigen::Vector3d weldStanceCoM_W =
        Eigen::Vector3d(qIniDes(0), qIniDes(1), qIniDes(2)) + kinDynSolver.computeFixedCoM(qIniFixedDes);
    const double weldStanceTransitionDuration = weldStanceTransitionDurationDefault;
    const int weldStanceThreadCount = defaultWeldStanceThreadCount();
    bool weldStanceTransitionActive = false;
    bool weldStanceTransitionJustFinished = false;
    double weldStanceTransitionStartTime = 0.0;
    Eigen::VectorXd weldStanceTransitionStartQpos = Eigen::VectorXd::Zero(mj_model->nq);
    Eigen::VectorXd weldStanceTransitionTargetQpos = Eigen::VectorXd::Zero(mj_model->nq);

    auto printWeldStanceResult = [&](const WeldStanceResult &result)
    {
        std::cout << "[WeldStance] timing: total=" << result.totalWallTime
                  << " s, body_search=" << (result.coarseWallTime + result.broadFallbackWallTime + result.refineWallTime)
                  << " s, left_arm_search=" << result.leftArmLightWallTime
                  << " s, coarse=" << result.coarseWallTime
                  << " s, fallback=" << result.broadFallbackWallTime
                  << " s, refine=" << result.refineWallTime
                  << " s, threads=" << result.threadCount << std::endl;
        std::cout << "[WeldStance] candidates=" << result.candidates
                  << ", body_candidates=" << result.bodyCandidates
                  << ", left_arm_light_eval=" << result.leftArmLightEvaluations
                  << ", left_arm_light_reject=" << result.leftArmLightRejects
                  << ", valid=" << result.validCandidates
                  << ", leg_ik_fail=" << result.legIkFail
                  << ", leg_limit_fail=" << result.legLimitFail
                  << ", com_fail=" << result.comFail
                  << ", arm_ik_fail=" << result.armIkFail
                  << ", arm_limit_fail=" << result.armLimitFail
                  << ", preapproach_fail=" << result.preApproachFail
                  << ", base_height_fail=" << result.baseHeightFail
                  << ", clearance_pass=" << result.clearancePassCandidates
                  << ", clearance_fail=" << result.clearanceFail
                  << ", broad_fallback=" << (result.broadFallbackUsed ? 1 : 0)
                  << ", broad_candidates=" << result.broadFallbackCandidates
                  << ", refine=" << result.refineCandidates << std::endl;
        if (result.broadFallbackUsed)
        {
            std::cerr << "[WeldStance] warning: main relative coarse search found no valid stance; "
                      << "wide fallback candidates=" << result.broadFallbackCandidates << std::endl;
        }
        if (result.clearanceFallbackUsed)
        {
            std::cerr << "[WeldStance] warning: no candidate met arm/workpiece clearance >= "
                      << weldArmClearanceThreshold << " m; fallback to best IK/COM stance." << std::endl;
        }
    };

    auto commitWeldStanceResult = [&](const WeldStanceResult &result, double now)
    {
        weldStanceResult = result;
        printWeldStanceResult(weldStanceResult);
        if (!weldStanceResult.valid)
        {
            std::cerr << "[WeldStance] no valid auto stance; G will be disabled for welding." << std::endl;
            return;
        }

        weldStanceReady = true;
        const auto &best = weldStanceResult.best;
        weldStanceBaseXyzYaw << best.basePos.x(), best.basePos.y(), best.basePos.z(), best.yaw;
        weldStanceScore = best.score;
        weldStanceMaxIkErr = best.maxIkErr;
        weldStanceMinLimitMargin = best.minLimitMargin;
        weldStanceComMargin = best.comMargin;
        weldStanceClearanceMargin = best.clearanceMargin;
        weldPreApproachClearance = best.preApproachClearance;
        weldStanceFootL_W = best.footL_W;
        weldStanceFootR_W = best.footR_W;
        weldStanceLeftArmQ = best.leftArmQ;
        qIniFixedDes = best.qFixed;
        qIniDes.block(7, 0, robot_nv - 6, 1) = qIniFixedDes;
        qIniDes(0) = best.basePos.x();
        qIniDes(1) = best.basePos.y();
        qIniDes(2) = best.basePos.z();
        qIniDes(3) = 0.0;
        qIniDes(4) = 0.0;
        qIniDes(5) = std::sin(0.5 * best.yaw);
        qIniDes(6) = std::cos(0.5 * best.yaw);
        stand_legLength = best.basePos.z() - foot_height;
        footPlacement.legLength = stand_legLength;
        weldStanceCoM_W = best.basePos + yawRot(best.yaw) * kinDynSolver.computeFixedCoM(qIniFixedDes);
        hd_l_des = qIniFixedDes.segment<5>(12);
        hd_r_des = qIniFixedDes.segment<5>(17);
        if (simRealOpenloopEnabled && simRealCommandPublishEnabled)
        {
            bool targetValid = qIniFixedDes.size() >= static_cast<int>(kV4CommandSourceJointNames.size());
            for (size_t i = 0; targetValid && i < kV4CommandSourceJointNames.size(); i++)
            {
                targetValid = std::isfinite(qIniFixedDes(static_cast<int>(i)));
            }

            if (simRealLastPublishedPos.size() == kV4CommandSourceJointNames.size() && targetValid)
            {
                realWeldStanceRampStart = simRealLastPublishedPos;
                for (size_t i = 0; i < kV4CommandSourceJointNames.size(); i++)
                {
                    realWeldStanceRampTarget[i] = qIniFixedDes(static_cast<int>(i));
                }
                double maxStartDelta = 0.0;
                for (size_t i = 0; i < kV4CommandSourceJointNames.size(); i++)
                {
                    maxStartDelta = std::max(maxStartDelta,
                                             std::fabs(realWeldStanceRampTarget[i] - realWeldStanceRampStart[i]));
                }
                realWeldStanceRampElapsed = 0.0;
                realWeldStanceRampActive = true;
                simRealRampElapsed = realCommandInitialRampTimeSec;
                std::cout << "[Publish-V4SimOpenLoop] weld stance real command ramp started: "
                          << "from last published command to searched stance, duration="
                          << realWeldStanceRampTimeSec
                          << " s, max_delta=" << maxStartDelta << " rad" << std::endl;
            }
            else
            {
                simRealCommandPublishEnabled = false;
                realWeldStanceRampActive = false;
                if (simRealCommandSafety)
                {
                    simRealCommandSafety->resetCommandHistory();
                }
                std::cerr << "[Publish-V4SimOpenLoop] stopped real command publishing: "
                          << "weld stance target needs a valid last published command and finite 22-DoF target."
                          << " Press P again after checking the robot side." << std::endl;
            }
        }
        if (weldReapplyOnCloseLoop)
        {
            weldStanceTransitionStartQpos = readMujocoQpos(mj_model, mj_data);
            weldStanceTransitionTargetQpos = makeMujocoQpos(mj_model->nq, best.basePos, best.yaw, qIniFixedDes);
            weldStanceTransitionStartTime = now;
            weldStanceTransitionJustFinished = false;
            if (weldStanceTransitionDuration <= 1.0e-6)
            {
                writeMujocoQpos(mj_model, mj_data, weldStanceTransitionTargetQpos);
                weldStanceTransitionActive = false;
                weldStanceTransitionJustFinished = true;
                std::cout << "[WeldStance] optimized qpos applied instantly "
                          << "(weld stance transition duration is 0)." << std::endl;
            }
            else
            {
                weldStanceTransitionActive = true;
                std::cout << "[WeldStance] visual transition default->optimized started, duration="
                          << weldStanceTransitionDuration << " s" << std::endl;
            }
        }
        else
        {
            std::cout << "[WeldStance] optimized qpos selected but not applied "
                      << "(weld stance reapply disabled in code)." << std::endl;
        }
        std::cout << "[WeldStance] selected base xyz/yaw: "
                  << best.basePos.transpose() << ", " << best.yaw << std::endl;
        std::cout << "[WeldStance] relative params standOff/sideBias/yawOffset: "
                  << best.standOff << ", " << best.sideBias << ", " << best.yawOffset << std::endl;
        std::cout << "[WeldStance] foot L/R world: "
                  << best.footL_W.transpose() << " / " << best.footR_W.transpose() << std::endl;
        std::cout << "[WeldStance] foot params x/width/stagger/y_center: "
                  << best.footX << ", " << best.width << ", " << best.stagger
                  << ", " << best.footYCenter << std::endl;
        std::cout << "[WeldStance] left arm preset #" << best.leftArmIndex
                  << " q0: " << weldStanceLeftArmQ.transpose() << std::endl;
        std::cout << "[WeldStance] right arm q0: "
                  << qIniFixedDes.segment<5>(17).transpose() << std::endl;
        std::cout << "[WeldStance] score=" << best.score
                  << ", max_ik_err=" << best.maxIkErr
                  << ", rms_ik_err=" << best.rmsIkErr
                  << ", min_limit_margin=" << best.minLimitMargin
                  << ", com_margin=" << best.comMargin
                  << ", clearance_margin=" << best.clearanceMargin
                  << ", preapproach_clearance=" << best.preApproachClearance << std::endl;
        std::cout << "[WeldStance] weld_stance_valid=1" << std::endl;
    };

    std::future<WeldStanceResult> weldStanceFuture;
    bool weldStanceSearchRunning = false;
    bool weldStanceSearchDone = !weldStanceAuto || !nominalWeldTrajectoryReady;
    auto startWeldStanceSearch = [&](double now)
    {
        if (!weldStanceAuto || weldStanceSearchRunning || weldStanceSearchDone)
        {
            return;
        }
        if (!weldTrajectoryReady)
        {
            std::cerr << "[WeldStance] auto search skipped: workpiece trajectory is not active."
                      << std::endl;
            weldStanceSearchDone = true;
            return;
        }
        const WeldTrajectory trajectoryForSearch = weldTrajectory;
        const WeldWorkpieceRuntime workpieceForSearch = weldWorkpiece;
        const ControllerConfig controllerConfigForSearch = controllerConfig;
        std::cout << "[WeldStance] auto search enabled at t=" << now
                  << " s in background; MuJoCo remains interactive, threads="
                  << weldStanceThreadCount << "." << std::endl;
        weldStanceFuture = std::async(std::launch::async,
                                      [trajectoryForSearch, workpieceForSearch, controllerConfigForSearch, foot_height,
                                       weldPreApproachDist, weldMinPreApproachClearance, weldStanceThreadCount]()
                                      {
                                          Pin_KinDyn_V4 searchKinDyn("../models/speedbot_v4/speedbot_v4.urdf");
                                          return selectWeldStance(searchKinDyn, trajectoryForSearch, workpieceForSearch,
                                                                  controllerConfigForSearch, foot_height,
                                                                  weldPreApproachDist,
                                                                  weldMinPreApproachClearance,
                                                                  weldStanceThreadCount);
                                      });
        weldStanceSearchRunning = true;
    };

    auto activateWeldWorkpiece = [&](double now)
    {
        if (weldWorkpieceActivated)
        {
            return;
        }
        weldWorkpiece = makeWeldWorkpieceRuntime();
        applyWeldWorkpieceToMujoco(mj_model, mj_data, weldWorkpiece);
        weldWorkpieceActivated = true;
        std::cout << "[WeldWorkpiece] randomize at t=" << now << " s" << std::endl;
        printWeldWorkpieceRuntime(weldWorkpiece);

        if (nominalWeldTrajectoryReady)
        {
            weldTrajectory = nominalWeldTrajectory;
            weldTrajectory.applyTranslation(weldWorkpiece.offset_W);
            weldTrajectoryReady = true;
            RobotState.weld_trajectory_valid = true;
            const auto weldStartSample = weldTrajectory.sample(0.0);
            const auto weldEndSample = weldTrajectory.sample(weldTrajectory.totalDuration());
            const Eigen::Vector3d approachDir_W =
                normalizedOr(weldWorkpiece.approachDir_W, Eigen::Vector3d(-1.0, 0.0, 0.0));
            weldPreApproachTcp_W = weldStartSample.pose.pos + approachDir_W * weldPreApproachDist;
            weldPreApproachClearance =
                pointAabbDistance(weldPreApproachTcp_W,
                                  weldWorkpiece.plateCenter_W(),
                                  weldWorkpiece.plateHalfExtents);
            std::cout << "[WeldTrajectory] runtime path start/end: "
                      << weldStartSample.pose.pos.transpose() << " / "
                      << weldEndSample.pose.pos.transpose() << std::endl;
            std::cout << "[Weld] runtime preapproach_tcp="
                      << weldPreApproachTcp_W.transpose()
                      << ", clearance=" << weldPreApproachClearance << " m" << std::endl;
        }
        else
        {
            weldTrajectoryReady = false;
            RobotState.weld_trajectory_valid = false;
            std::cerr << "[WeldTrajectory] runtime disabled after workpiece activation: "
                      << weldLoadErr << std::endl;
        }

        if (weldStanceAuto)
        {
            startWeldStanceSearch(now);
        }
        else
        {
            weldStanceSearchDone = true;
            std::cout << "[WeldStance] nominal mode: using legacy initial stance after workpiece activation."
                      << std::endl;
        }
    };

    jsInterp.setIniPos(qIniDes(0), qIniDes(1), weldStanceBaseXyzYaw(3));
    RobotState.js_pos_des << qIniDes(0), qIniDes(1), qIniDes(2);
    RobotState.js_eul_des << 0.0, 0.0, weldStanceBaseXyzYaw(3);
    WBC_solv.pCoMDes = weldStanceCoM_W;
    WBC_solv.setQini(qIniDes, RobotState.q);

    // register data logger items
    logger.addIterm("dyn_time", 1);
    logger.addIterm("motors_pos_cur", robot_nv - 6);
    logger.addIterm("motors_pos_des", robot_nv - 6);
    logger.addIterm("motors_tor_cur", robot_nv - 6);
    logger.addIterm("motors_tor_des", robot_nv - 6);
    logger.addIterm("motors_tor_out", robot_nv - 6);
    logger.addIterm("motors_vel_des", robot_nv - 6);
    logger.addIterm("motors_vel_cur", robot_nv - 6);
    logger.addIterm("FL_est", 3);
    logger.addIterm("FR_est", 3);
    logger.addIterm("wbc_FrRes", 12);
    logger.addIterm("base_pos_des", 3);
    logger.addIterm("base_pos", 3);
    logger.addIterm("base_pos_est", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("base_vel_est", 3);
    logger.addIterm("base_rpy", 3);
    logger.addIterm("eul_est", 3);
    logger.addIterm("Ufe", 13);
    logger.addIterm("phi", 1);
    logger.addIterm("phiSwitchMinRuntime", 1);
    logger.addIterm("tSwing", 1);
    logger.addIterm("mainControlDt", 1);
    logger.addIterm("mpcControlDt", 1);
    logger.addIterm("mpcPredictionHorizon", 1);
    logger.addIterm("mpcControlHorizon", 1);
    logger.addIterm("js_vel_des", 3);
    logger.addIterm("js_omega_des", 3);
    logger.addIterm("swingDesPosCur_W", 3);
    logger.addIterm("swingDesPosFinal_W", 3);
    logger.addIterm("qpStatus_MPC", 1);
    logger.addIterm("legState", 1);
    logger.addIterm("motionState", 1);
    logger.addIterm("weld_active", 1);
    logger.addIterm("weld_segment_index", 1);
    logger.addIterm("weld_phase", 1);
    logger.addIterm("weld_tcp_pos_des", 3);
    logger.addIterm("weld_tcp_pos_cur", 3);
    logger.addIterm("weld_tcp_pos_err", 3);
    logger.addIterm("weld_tcp_rot_err", 3);
    logger.addIterm("wbc_qp_status", 1);
    logger.addIterm("weld_stance_valid", 1);
    logger.addIterm("weld_stance_base_xyz_yaw", 4);
    logger.addIterm("weld_stance_score", 1);
    logger.addIterm("weld_stance_max_ik_err", 1);
    logger.addIterm("weld_stance_min_limit_margin", 1);
    logger.addIterm("weld_stance_com_margin", 1);
    logger.addIterm("weld_stance_foot_l", 3);
    logger.addIterm("weld_stance_foot_r", 3);
    logger.addIterm("weld_stance_left_arm_q", 5);
    logger.addIterm("weld_preapproach_clearance", 1);
    logger.addIterm("weld_prepare_phase", 1);
    logger.addIterm("weld_recover_phase", 1);
    logger.addIterm("weld_ang_momentum", 3);
    logger.addIterm("weld_h_ang_norm", 1);
    logger.addIterm("weld_left_arm_q", 5);
    logger.addIterm("weld_left_arm_balance_vel", 5);
    logger.addIterm("weld_left_arm_vel_norm", 1);
    logger.addIterm("weld_base_ref", 3);
    logger.addIterm("weld_base_delta", 3);
    logger.addIterm("weld_base_delta_norm", 1);
    logger.addIterm("weld_com_ref", 3);
    logger.addIterm("weld_com_delta", 3);
    logger.addIterm("weld_com_delta_norm", 1);
    logger.addIterm("weld_base_rpy_ref", 3);
    logger.addIterm("weld_base_rpy_delta", 3);
    logger.addIterm("weld_cop_margin", 1);
    logger.addIterm("weld_tau_margin", 1);
    logger.addIterm("weld_clearance_margin", 1);

    logger.finishItermAdding();

    //// main loop
    int mainCtrlCount = mainCtrlDecimation - 1;
    int mpcCtrlCount = mpcCtrlDecimation - 1;
    int logCtrlCount = 0;
    constexpr int logDecimation = 5;

    bool openLoopPhaseActive = true;
    bool weldTaskRequested = false;
    bool weldStartPromptPrinted = false;
    double simEndTime = 200.0;
    const bool headlessMode = readBoolEnv("HEADLESS", false);
    const char *autoWalkEnv = std::getenv("AUTOWALK");
    const bool autoWalk = (autoWalkEnv != nullptr) && (std::string(autoWalkEnv) == "1");
    bool autoWalkStarted = false;
    bool autoStopTriggered = false;
    bool stopToStandPending = false;
    bool weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
    bool weldActive = false;
    bool weldPreparing = false;
    bool weldHolding = false;
    bool weldRecovering = false;
    bool finishWeldAfterControl = false;
    double weldStartTime = 0.0;
    double weldPrepareStartTime = 0.0;
    double weldPrepareReadySince = -1.0;
    double weldHoldStartTime = 0.0;
    double weldRecoverStartTime = 0.0;
    Eigen::Vector3d weldCoMDes = Eigen::Vector3d::Zero();
    Eigen::Vector3d weldPrepareStartTcpPos_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d weldFinalTcpPos_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d weldRecoverStartTcpPos_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d weldRecoverTargetTcp_W = weldPreApproachTcp_W;
    Eigen::Vector3d weldBaseRef_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d weldCoMRef_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d weldBaseRpyRef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d weldFinalTcpRot_W = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d weldRecoverTargetRot_W = Eigen::Matrix3d::Identity();
    const double weldPrepareHoldT = weldPrepareHoldTDefault;
    const double weldPrepareMaxT = weldPrepareMaxTDefault;
    const double weldPrepareMinPhase = weldPrepareMinPhaseDefault;
    const double weldPrepareStartErr = weldPrepareStartErrDefault;
    const double weldPrepareFallbackErr = std::max(weldPrepareStartErr, weldPrepareFallbackErrDefault);
    const double weldTcpFinishErr = weldTcpFinishErrDefault;
    const double weldFinishTimeout = weldFinishTimeoutDefault;
    const double weldTcpLookahead = weldTcpLookaheadDefault;
    bool weldFinishTimedOut = false;
    bool weldStabilityRefValid = false;
    double weldMaxTcpErr = 0.0;
    double weldMaxBaseDelta = 0.0;
    double weldMaxComDelta = 0.0;
    double weldMaxRollPitchDelta = 0.0;
    double weldMinCopMargin = std::numeric_limits<double>::infinity();
    int weldBadQpCount = 0;
    const double stopTransitionVxThresh = 0.05;
    const double stopTransitionWzThresh = 0.08;
    double autoStopTime = -1.0;
    const char *autoStopEnv = std::getenv("AUTOSTOP_TIME");
    if (autoStopEnv != nullptr)
    {
        autoStopTime = std::atof(autoStopEnv);
    }
    const bool finiteSimDuration = headlessMode;
    if (finiteSimDuration)
    {
        std::cout << "[MuJoCo] finite run enabled, sim_end_t=" << simEndTime
                  << " s, reason=headless" << std::endl;
    }
    else
    {
        std::cout << "[MuJoCo] interactive run has no fixed simulation time limit." << std::endl;
    }
    std::cout << "[WeldPrepare] hold=" << weldPrepareHoldT
              << " s, max=" << weldPrepareMaxT
              << " s, start_err=" << weldPrepareStartErr
              << " m, fallback_err=" << weldPrepareFallbackErr
              << " m, finish_err=" << weldTcpFinishErr
              << " m, finish_timeout=" << weldFinishTimeout
              << " s, tcp_lookahead=" << weldTcpLookahead << " s" << std::endl;

    auto resetWeldStabilityMetrics = [&]()
    {
        weldBaseRef_W = RobotState.base_pos;
        weldCoMRef_W = RobotState.pCoM_W;
        weldBaseRpyRef = RobotState.base_rpy;
        weldMaxTcpErr = 0.0;
        weldMaxBaseDelta = 0.0;
        weldMaxComDelta = 0.0;
        weldMaxRollPitchDelta = 0.0;
        weldMinCopMargin = std::numeric_limits<double>::infinity();
        weldBadQpCount = 0;
        weldFinishTimedOut = false;
        weldStabilityRefValid = true;
    };

    auto writeWeldStabilityLogFields = [&]()
    {
        if (weldStabilityRefValid && isWeldMotionState(RobotState.motionState))
        {
            RobotState.weld_base_ref_W = weldBaseRef_W;
            RobotState.weld_base_delta_W = RobotState.base_pos - weldBaseRef_W;
            RobotState.weld_com_ref_W = weldCoMRef_W;
            RobotState.weld_com_delta_W = RobotState.pCoM_W - weldCoMRef_W;
            RobotState.weld_base_rpy_ref = weldBaseRpyRef;
            RobotState.weld_base_rpy_delta = RobotState.base_rpy - weldBaseRpyRef;
            RobotState.weld_base_delta_norm = RobotState.weld_base_delta_W.norm();
            RobotState.weld_com_delta_norm = RobotState.weld_com_delta_W.norm();
        }
        else
        {
            RobotState.weld_base_ref_W.setZero();
            RobotState.weld_base_delta_W.setZero();
            RobotState.weld_com_ref_W.setZero();
            RobotState.weld_com_delta_W.setZero();
            RobotState.weld_base_rpy_ref.setZero();
            RobotState.weld_base_rpy_delta.setZero();
            RobotState.weld_base_delta_norm = 0.0;
            RobotState.weld_com_delta_norm = 0.0;
        }
    };

    auto maybePrintWeldReadyPrompt = [&]()
    {
        if (weldTaskRequested && !weldStartPromptPrinted &&
            weldTrajectoryReady && weldStanceReady &&
            !weldStanceSearchRunning && !weldStanceTransitionActive &&
            !realWeldStanceRampActive)
        {
            std::cout << "[Weld] preparation ready; press G again in Stand to start WeldPrepare."
                      << std::endl;
            weldStartPromptPrinted = true;
        }
    };

    auto stopSimRealPublishingForSafety = [&](const std::string &reason)
    {
        if (!simRealCommandSafetyStopped)
        {
            std::cerr << "[Safety-V4SimOpenLoop] stopped real command publishing: "
                      << reason << std::endl;
            std::cerr << "[Safety-V4SimOpenLoop] restart the demo after checking the robot side."
                      << std::endl;
        }
        simRealCommandSafetyStopped = true;
        simRealCommandPublishEnabled = false;
        realWeldStanceRampActive = false;
        if (simRealCommandSafety)
        {
            simRealCommandSafety->resetCommandHistory();
        }
    };

    auto publishV4SimRealCommand = [&](double now)
    {
        if (!simRealOpenloopEnabled || !simRealCommandPublishEnabled)
        {
            return;
        }
        if (RobotState.motors_pos_des.size() < kV4CommandSourceJointNames.size())
        {
            stopSimRealPublishingForSafety("motors_pos_des invalid: fewer than 22 values");
            return;
        }

        std::vector<double> targetPos(RobotState.motors_pos_des.begin(),
                                      RobotState.motors_pos_des.begin() + kV4CommandSourceJointNames.size());
        for (double value : targetPos)
        {
            if (!std::isfinite(value))
            {
                stopSimRealPublishingForSafety("motors_pos_des invalid: contains NaN/Inf");
                return;
            }
        }

        std::vector<double> publishPos = targetPos;
        if (realWeldStanceRampActive)
        {
            const double rampTime = std::max(realWeldStanceRampTimeSec, mainCtrlDt);
            const double ratio = std::clamp(realWeldStanceRampElapsed / rampTime, 0.0, 1.0);
            const double alpha = ratio * ratio * (3.0 - 2.0 * ratio);
            for (size_t i = 0; i < publishPos.size(); i++)
            {
                publishPos[i] = (1.0 - alpha) * realWeldStanceRampStart[i] + alpha * realWeldStanceRampTarget[i];
            }
            realWeldStanceRampElapsed += mainCtrlDt;
            if (realWeldStanceRampElapsed >= rampTime)
            {
                publishPos = realWeldStanceRampTarget;
                realWeldStanceRampActive = false;
                std::cout << "[Publish-V4SimOpenLoop] weld stance real command ramp complete at t="
                          << now << " s" << std::endl;
                maybePrintWeldReadyPrompt();
            }
        }
        else
        {
            const double rampTime = std::max(realCommandInitialRampTimeSec, mainCtrlDt);
            if (simRealRampElapsed < rampTime)
            {
                const double ratio = std::clamp(simRealRampElapsed / rampTime, 0.0, 1.0);
                const double alpha = ratio * ratio * (3.0 - 2.0 * ratio);
                for (size_t i = 0; i < publishPos.size(); i++)
                {
                    publishPos[i] = (1.0 - alpha) * simRealRampStartPos[i] + alpha * targetPos[i];
                }
                simRealRampElapsed += mainCtrlDt;
            }
        }

        simRealCommandPubCount++;
        if (simRealCommandPubCount < simRealCommandPubDecimation)
        {
            return;
        }
        simRealCommandPubCount = 0;

        std::string safetyReason;
        if (simRealCommandSafety == nullptr)
        {
            stopSimRealPublishingForSafety("safety monitor unavailable");
            return;
        }
        if (!simRealCommandSafety->validateCommandAndRemember(publishPos, safetyReason))
        {
            stopSimRealPublishingForSafety(safetyReason);
            return;
        }

        simRealCommandPub.setMotorsCommand(publishPos, simRealZeroVel, simRealZeroTau);
        simRealCommandPub.spinSome();
        simRealLastPublishedPos = publishPos;
        static int publishLogCount = 0;
        if (publishLogCount == 0 || publishLogCount % 500 == 0)
        {
            std::cout << "[Publish-V4SimOpenLoop] sent command at t=" << now
                      << " s, motionState=" << motionStateName(RobotState.motionState)
                      << ", topic=" << controllerConfig.rosTopicActionCmd << std::endl;
        }
        publishLogCount++;
    };

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    std::string mujocoExitReason = "window closed";

    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim)
        {
            mj_step(mj_model, mj_data);
            simTime = mj_data->time;
            if (weldStanceTransitionActive)
            {
                const double alphaRaw = weldStanceTransitionDuration > 1.0e-9
                                            ? (simTime - weldStanceTransitionStartTime) / weldStanceTransitionDuration
                                            : 1.0;
                const double alpha = smoothStep01(alphaRaw);
                const Eigen::VectorXd qpos =
                    (1.0 - alpha) * weldStanceTransitionStartQpos + alpha * weldStanceTransitionTargetQpos;
                writeMujocoQpos(mj_model, mj_data, qpos);
                if (alphaRaw >= 1.0)
                {
                    writeMujocoQpos(mj_model, mj_data, weldStanceTransitionTargetQpos);
                    weldStanceTransitionActive = false;
                    weldStanceTransitionJustFinished = true;
                    std::cout << "[WeldStance] visual transition completed at t="
                              << simTime << " s" << std::endl;
                }
            }
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);
            mainCtrlCount++;
            if (mainCtrlCount < mainCtrlDecimation)
            {
                continue;
            }
            mainCtrlCount = 0;
            finishWeldAfterControl = false;

            if (simTime > 1 && StateModule.flag_init)
            {
                std::cout << "init state module" << std::endl;
                StateModule.init(RobotState);
            }

            if (weldStanceSearchRunning &&
                weldStanceFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                weldStanceSearchRunning = false;
                weldStanceSearchDone = true;
                commitWeldStanceResult(weldStanceFuture.get(), simTime);
                weldHoldOptimizedStance = weldStanceReady && weldStanceAuto &&
                                          weldReapplyOnCloseLoop && !weldStanceTransitionActive;
                maybePrintWeldReadyPrompt();
            }
            if (weldStanceTransitionJustFinished)
            {
                mj_interface.updateSensorValues();
                mj_interface.dataBusWrite(RobotState);
                kinDynSolver.dataBusRead(RobotState);
                kinDynSolver.computeJ_dJ();
                kinDynSolver.computeDyn();
                kinDynSolver.dataBusWrite(RobotState);
                StateModule.init(RobotState);
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                RobotState.js_pos_des << qIniDes(0), qIniDes(1), qIniDes(2);
                RobotState.js_eul_des << 0.0, 0.0, weldStanceBaseXyzYaw(3);
                WBC_solv.pCoMDes = weldStanceCoM_W;
                WBC_solv.setQini(qIniDes, RobotState.q);
                weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                weldStanceTransitionJustFinished = false;
                maybePrintWeldReadyPrompt();
            }

            buttonState = uiController.getButtonState();
            if (simRealOpenloopEnabled && buttonState.key_p)
            {
                if (simRealCommandSafetyStopped)
                {
                    std::cerr << "[PublishGate-V4SimOpenLoop] P ignored after safety stop. Please restart manually." << std::endl;
                }
                else if (simRealCommandPublishEnabled)
                {
                    simRealCommandPublishEnabled = false;
                    realWeldStanceRampActive = false;
                    if (simRealCommandSafety)
                    {
                        simRealCommandSafety->resetCommandHistory();
                    }
                    std::cout << "[PublishGate-V4SimOpenLoop] real command publishing stopped by P, topic="
                              << controllerConfig.rosTopicActionCmd << std::endl;
                }
                else
                {
                    realWeldStanceRampActive = false;
                    simRealRampStartPos.assign(kV4CommandSourceJointNames.size(), 0.0);
                    const bool hasLastCommand =
                        simRealLastPublishedPos.size() == kV4CommandSourceJointNames.size();
                    if (hasLastCommand)
                    {
                        simRealRampStartPos = simRealLastPublishedPos;
                    }
                    simRealRampElapsed = 0.0;
                    simRealCommandPubCount = simRealCommandPubDecimation - 1;
                    simRealCommandPublishEnabled = true;
                    if (simRealCommandSafety)
                    {
                        simRealCommandSafety->resetCommandHistory();
                    }
                    std::cout << "[PublishGate-V4SimOpenLoop] real command publishing enabled by P, topic="
                              << controllerConfig.rosTopicActionCmd
                              << ", motionState=" << motionStateName(RobotState.motionState)
                              << ", ramp_start=" << (hasLastCommand ? "last command" : "zero command")
                              << ", real command initial ramp_time=" << realCommandInitialRampTimeSec
                              << " s" << std::endl;
                }
            }
            if (buttonState.key_g && openLoopPhaseActive)
            {
                std::cout << "[Weld] G ignored in open-loop: press F first to enter closed-loop Stand."
                          << std::endl;
            }
            if (buttonState.key_f && openLoopPhaseActive)
            {
                WBC_solv.fe_l_pos_des_W = RobotState.fe_l_pos_W;
                WBC_solv.fe_r_pos_des_W = RobotState.fe_r_pos_W;
                WBC_solv.fe_l_rot_des_W = RobotState.fe_l_rot_W;
                WBC_solv.fe_r_rot_des_W = RobotState.fe_r_rot_W;
                WBC_solv.pCoMDes = weldHoldOptimizedStance ? weldStanceCoM_W : RobotState.pCoM_W;
                openLoopPhaseActive = false;
                stopToStandPending = false;
                RobotState.motionState = DataBus::Stand;
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setVyDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                std::cout << "[Mode] F: OpenLoop -> closed-loop Stand at t="
                          << simTime << " s" << std::endl;
            }
            if (!openLoopPhaseActive)
            {
                if (buttonState.key_space && realWeldStanceRampActive)
                {
                    std::cout << "[Mode] Space ignored: wait for weld stance real command ramp to finish."
                              << std::endl;
                }
                else if (buttonState.key_space && RobotState.motionState == DataBus::Stand)
                {
                    gaitScheduler.start();
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    weldHoldOptimizedStance = false;
                    RobotState.motionState = DataBus::Walk;
                    std::cout << "[Mode] Space: Stand -> Walk at t=" << simTime << " s" << std::endl;
                }
                else if (buttonState.key_space && RobotState.motionState == DataBus::Walk)
                {
                    // Graceful stop: first ramp speed/yaw-rate to zero, then switch to Walk2Stand.
                    jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                    jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                    stopToStandPending = true;
                    std::cout << "[Mode] Space: Walk stop ramp requested at t="
                              << simTime << " s" << std::endl;
                }

                if (buttonState.key_a &&
                    (RobotState.motionState == DataBus::Walk || RobotState.motionState == DataBus::Walk2Stand))
                {
                    if (jsInterp.wzLGen.yDes < 0)
                        jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
                    else
                        jsInterp.setWzDesLPara(turnRateCmd, controllerConfig.wzRampTime);
                }
                if (buttonState.key_d &&
                    (RobotState.motionState == DataBus::Walk || RobotState.motionState == DataBus::Walk2Stand))
                {
                    if (jsInterp.wzLGen.yDes > 0)
                        jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
                    else
                        jsInterp.setWzDesLPara(-turnRateCmd, controllerConfig.wzRampTime);
                }

                // W: forward at current xv_des
                if (buttonState.key_w && RobotState.motionState == DataBus::Walk)
                    jsInterp.setVxDesLPara(xv_des, controllerConfig.vxRampTime);

                // S: backward at current xv_des (negative)
                if (buttonState.key_s && RobotState.motionState == DataBus::Walk)
                    jsInterp.setVxDesLPara(-fabs(xv_des), controllerConfig.vxRampTime);

                // J: emergency stop to stand
                if (buttonState.key_j && isWeldMotionState(RobotState.motionState))
                {
                    weldActive = false;
                    weldPreparing = false;
                    weldHolding = false;
                    weldRecovering = false;
                    finishWeldAfterControl = false;
                    RobotState.motionState = DataBus::Stand;
                    RobotState.weld_active = false;
                    RobotState.weld_recover_phase = 0.0;
                    weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                    std::cout << "[Weld] J: abort and return to stand" << std::endl;
                }
                else if (buttonState.key_j && RobotState.motionState != DataBus::Stand)
                {
                    jsInterp.setVxDesLPara(0, controllerConfig.vxStopRampTime);
                    jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
                    stopToStandPending = true;
                    std::cout << "[Joystick] J: stop and stand" << std::endl;
                }

                // E: increase speed
                if (buttonState.key_e)
                {
                    xv_des = std::min(std::round((xv_des + xv_step) * 10.0) / 10.0, xv_max);
                    if (RobotState.motionState == DataBus::Walk && std::fabs(jsInterp.vxLGen.yDes) > 1e-3)
                    {
                        const double dir = (jsInterp.vxLGen.yDes >= 0.0) ? 1.0 : -1.0;
                        jsInterp.setVxDesLPara(dir * xv_des, controllerConfig.speedUpdateRampTime);
                    }
                    std::cout << "[Speed] xv_des=" << xv_des << " m/s" << std::endl;
                }

                // Q: decrease speed
                if (buttonState.key_q)
                {
                    xv_des = std::max(std::round((xv_des - xv_step) * 10.0) / 10.0, xv_min);
                    if (RobotState.motionState == DataBus::Walk && std::fabs(jsInterp.vxLGen.yDes) > 1e-3)
                    {
                        const double dir = (jsInterp.vxLGen.yDes >= 0.0) ? 1.0 : -1.0;
                        jsInterp.setVxDesLPara(dir * xv_des, controllerConfig.speedUpdateRampTime);
                    }
                    std::cout << "[Speed] xv_des=" << xv_des << " m/s" << std::endl;
                }

                if (buttonState.key_h)
                {
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    jsInterp.setWzDesLPara(0, controllerConfig.headingResetRampTime);
                    std::cout << "[Joystick] H: reset heading reference" << std::endl;
                }

                if (buttonState.key_g)
                {
                    if (isWeldMotionState(RobotState.motionState))
                    {
                        weldActive = false;
                        weldPreparing = false;
                        weldHolding = false;
                        weldRecovering = false;
                        finishWeldAfterControl = false;
                        RobotState.motionState = DataBus::Stand;
                        RobotState.weld_active = false;
                        RobotState.weld_recover_phase = 0.0;
                        weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                        std::cout << "[Weld] stopped by G at t=" << simTime << " s" << std::endl;
                    }
                    else if (RobotState.motionState == DataBus::Stand)
                    {
                        if (!weldWorkpieceActivated)
                        {
                            weldTaskRequested = true;
                            weldStartPromptPrinted = false;
                            std::cout << "[Weld] G: preparation requested at t=" << simTime
                                      << " s; randomizing workpiece and preparing stance." << std::endl;
                            activateWeldWorkpiece(simTime);
                            if (!weldTrajectoryReady)
                            {
                                std::cerr << "[Weld] preparation failed: " << weldLoadErr << std::endl;
                            }
                            else if (weldStanceSearchRunning || weldStanceTransitionActive || !weldStanceReady)
                            {
                                std::cout << "[Weld] preparing: waiting for weld stance search/visual transition."
                                          << std::endl;
                            }
                            else
                            {
                                maybePrintWeldReadyPrompt();
                            }
                        }
                        else if (!weldTrajectoryReady)
                        {
                            std::cerr << "[Weld] cannot start: " << weldLoadErr << std::endl;
                        }
                        else if (!weldStanceReady || weldStanceSearchRunning ||
                                 weldStanceTransitionActive || realWeldStanceRampActive)
                        {
                            weldTaskRequested = true;
                            if (weldStanceSearchRunning || weldStanceTransitionActive)
                            {
                                std::cout << "[Weld] preparing: weld stance search/visual transition is still running."
                                          << std::endl;
                            }
                            else if (realWeldStanceRampActive)
                            {
                                std::cout << "[Weld] preparing: real command weld stance ramp is still running."
                                          << std::endl;
                            }
                            else
                            {
                                std::cerr << "[Weld] cannot start: no valid auto stance. "
                                          << "Use WELD_STANCE_MODE=nominal to debug the legacy pose." << std::endl;
                            }
                        }
                        else
                        {
                            weldTaskRequested = true;
                            weldStartPromptPrinted = false;
                            weldActive = false;
                            weldPreparing = true;
                            weldHolding = false;
                            weldRecovering = false;
                            finishWeldAfterControl = false;
                            weldPrepareStartTime = simTime;
                            weldPrepareReadySince = -1.0;
                            weldPrepareStartTcpPos_W = RobotState.hd_r_pos_W;
                            weldRecoverTargetTcp_W = weldPreApproachTcp_W;
                            weldRecoverTargetRot_W = weldTrajectory.sample(0.0).pose.quat.toRotationMatrix();
                            weldCoMDes = (weldStanceAuto && weldReapplyOnCloseLoop) ? weldStanceCoM_W : RobotState.pCoM_W;
                            weldHoldOptimizedStance = weldStanceAuto && weldReapplyOnCloseLoop;
                            RobotState.motionState = DataBus::WeldPrepare;
                            RobotState.weld_active = false;
                            RobotState.weld_recover_phase = 0.0;
                            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                            std::cout << "[WeldPrepare] started by G at t=" << simTime
                                      << " s using " << weldTrajectory.path() << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "[Weld] G ignored: return to Stand before welding." << std::endl;
                    }
                }

                if (autoWalk && !autoWalkStarted && RobotState.motionState == DataBus::Stand)
                {
                    gaitScheduler.start();
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    weldHoldOptimizedStance = false;
                    RobotState.motionState = DataBus::Walk;
                    jsInterp.setVxDesLPara(xv_des, controllerConfig.autoStartRampTime);
                    autoWalkStarted = true;
                    std::cout << "[AutoWalk] started with vx_des=" << xv_des << " m/s" << std::endl;
                }
                if (autoWalk && autoWalkStarted && !autoStopTriggered && autoStopTime > 0.0 &&
                    simTime >= autoStopTime && RobotState.motionState == DataBus::Walk)
                {
                    jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                    jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                    stopToStandPending = true;
                    autoStopTriggered = true;
                    std::cout << "[AutoWalk] auto stop triggered at t=" << simTime << " s" << std::endl;
                }

                if (stopToStandPending && RobotState.motionState == DataBus::Walk)
                {
                    if (std::fabs(jsInterp.vxLGen.y) < stopTransitionVxThresh &&
                        std::fabs(jsInterp.wzLGen.y) < stopTransitionWzThresh)
                    {
                        RobotState.motionState = DataBus::Walk2Stand;
                        jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                        stopToStandPending = false;
                        std::cout << "[Stop2Stand] switch to Walk2Stand at t=" << simTime
                                  << " s, vxGen=" << jsInterp.vxLGen.y
                                  << ", wzGen=" << jsInterp.wzLGen.y << std::endl;
                    }
                }
            }

            StateModule.set(RobotState);
            StateModule.update();
            StateModule.get(RobotState);

            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            StateModule.setF(RobotState);
            StateModule.updateF();
            StateModule.getF(RobotState);

            if (openLoopPhaseActive)
            {
                RobotState.motionState = DataBus::Stand;
                stopToStandPending = false;
                weldActive = false;
                weldPreparing = false;
                weldHolding = false;
                weldRecovering = false;
                RobotState.weld_active = false;
                RobotState.weld_recover_phase = 0.0;
                jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setVyDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                RobotState.motors_pos_des = eigen2std(qIniFixedDes);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
                publishV4SimRealCommand(simTime);
                pvtCtr.dataBusRead(RobotState);
                pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
                pvtCtr.dataBusWrite(RobotState);
                mj_interface.setMotorsTorque(RobotState.motors_tor_out);
                continue;
            }

            RobotState.weld_trajectory_valid = weldTrajectoryReady;
            RobotState.weld_stance_valid = weldStanceReady && weldStanceAuto;
            RobotState.weld_stance_base_xyz_yaw = weldStanceBaseXyzYaw;
            RobotState.weld_stance_score = weldStanceScore;
            RobotState.weld_stance_max_ik_err = weldStanceMaxIkErr;
            RobotState.weld_stance_min_limit_margin = weldStanceMinLimitMargin;
            RobotState.weld_stance_com_margin = weldStanceComMargin;
            RobotState.weld_stance_foot_l_W = weldStanceFootL_W;
            RobotState.weld_stance_foot_r_W = weldStanceFootR_W;
            RobotState.weld_stance_left_arm_q = weldStanceLeftArmQ;
            RobotState.weld_preapproach_clearance = weldPreApproachClearance;
            RobotState.weld_clearance_margin = weldStanceClearanceMargin;
            if (RobotState.q.size() >= 7 + robot_nv - 6)
            {
                const Eigen::VectorXd qFixedCur = RobotState.q.segment(7, robot_nv - 6);
                RobotState.weld_clearance_margin =
                    computeRightArmWorkpieceClearance(kinDynSolver, qFixedCur, RobotState.base_pos,
                                                      RobotState.base_rot, weldWorkpiece);
            }

            if (RobotState.motionState == DataBus::WeldPrepare && weldPreparing && weldTrajectoryReady)
            {
                const auto weldSample = weldTrajectory.sample(0.0);
                const double prepareAlpha = smoothStep01((simTime - weldPrepareStartTime) / weldPrepareMaxT);
                RobotState.weld_active = false;
                RobotState.weld_tcp_pos_des_W =
                    (1.0 - prepareAlpha) * weldPrepareStartTcpPos_W + prepareAlpha * weldSample.pose.pos;
                RobotState.weld_tcp_rot_des_W = weldSample.pose.quat.toRotationMatrix();
                RobotState.weld_tcp_linear_vel_des_W.setZero();
                RobotState.weld_tcp_angular_vel_des_W.setZero();
                RobotState.weld_tcp_linear_acc_des_W.setZero();
                RobotState.weld_tcp_angular_acc_des_W.setZero();
                RobotState.weld_phase = prepareAlpha;
                RobotState.weld_segment_index = static_cast<double>(weldSample.segmentIndex);
                RobotState.weld_prepare_phase = prepareAlpha;
                RobotState.weld_recover_phase = 0.0;
                WBC_solv.pCoMDes = weldCoMDes;
                finishWeldAfterControl = false;
            }
            else if (RobotState.motionState == DataBus::Weld && weldActive && weldTrajectoryReady)
            {
                const double weldElapsed = simTime - weldStartTime;
                const auto weldSample = weldTrajectory.sample(weldElapsed);
                const auto weldControlSample = weldTrajectory.sample(weldElapsed + weldTcpLookahead);
                RobotState.weld_active = true;
                RobotState.weld_tcp_pos_des_W = weldControlSample.pose.pos;
                RobotState.weld_tcp_rot_des_W = weldControlSample.pose.quat.toRotationMatrix();
                RobotState.weld_tcp_linear_vel_des_W = weldControlSample.linearVel;
                RobotState.weld_tcp_angular_vel_des_W = weldControlSample.angularVel;
                RobotState.weld_tcp_linear_acc_des_W = weldControlSample.linearAcc;
                RobotState.weld_tcp_angular_acc_des_W = weldControlSample.angularAcc;
                RobotState.weld_phase = weldSample.phase;
                RobotState.weld_segment_index = static_cast<double>(weldSample.segmentIndex);
                RobotState.weld_prepare_phase = 1.0;
                RobotState.weld_recover_phase = 0.0;
                WBC_solv.pCoMDes = weldCoMDes;
                const double weldEndOverrun =
                    (simTime - weldStartTime) - weldTrajectory.totalDuration();
                const double weldTcpErr = RobotState.weld_tcp_pos_err_W.norm();
                const bool finishByPrecision = weldSample.done && (weldTcpErr < weldTcpFinishErr);
                weldFinishTimedOut = weldSample.done && !finishByPrecision &&
                                     (weldEndOverrun >= weldFinishTimeout);
                finishWeldAfterControl = finishByPrecision || weldFinishTimedOut;
            }
            else if (RobotState.motionState == DataBus::WeldHold && weldHolding && weldTrajectoryReady)
            {
                const auto weldSample = weldTrajectory.sample(weldTrajectory.totalDuration());
                RobotState.weld_active = false;
                RobotState.weld_tcp_pos_des_W = weldFinalTcpPos_W;
                RobotState.weld_tcp_rot_des_W = weldFinalTcpRot_W;
                RobotState.weld_tcp_linear_vel_des_W.setZero();
                RobotState.weld_tcp_angular_vel_des_W.setZero();
                RobotState.weld_tcp_linear_acc_des_W.setZero();
                RobotState.weld_tcp_angular_acc_des_W.setZero();
                RobotState.weld_phase = 1.0;
                RobotState.weld_segment_index = static_cast<double>(weldSample.segmentIndex);
                RobotState.weld_prepare_phase = 1.0;
                RobotState.weld_recover_phase = 0.0;
                WBC_solv.pCoMDes = weldCoMDes;
                finishWeldAfterControl = false;
            }
            else if (RobotState.motionState == DataBus::WeldRecover && weldRecovering && weldTrajectoryReady)
            {
                const double rawPhase = (simTime - weldRecoverStartTime) / weldRecoverDuration;
                const double recoverAlpha = smoothStep01(rawPhase);
                const double recoverAlphaDot = smoothStep01Dot(rawPhase) / weldRecoverDuration;
                const double recoverAlphaDdot = smoothStep01Ddot(rawPhase) /
                                                (weldRecoverDuration * weldRecoverDuration);
                const Eigen::Vector3d recoverDelta = weldRecoverTargetTcp_W - weldRecoverStartTcpPos_W;
                RobotState.weld_active = false;
                RobotState.weld_tcp_pos_des_W = weldRecoverStartTcpPos_W + recoverAlpha * recoverDelta;
                RobotState.weld_tcp_rot_des_W = weldRecoverTargetRot_W;
                RobotState.weld_tcp_linear_vel_des_W = recoverAlphaDot * recoverDelta;
                RobotState.weld_tcp_angular_vel_des_W.setZero();
                RobotState.weld_tcp_linear_acc_des_W = recoverAlphaDdot * recoverDelta;
                RobotState.weld_tcp_angular_acc_des_W.setZero();
                RobotState.weld_phase = 1.0;
                RobotState.weld_segment_index = -2.0;
                RobotState.weld_prepare_phase = 1.0;
                RobotState.weld_recover_phase = recoverAlpha;
                WBC_solv.pCoMDes = weldCoMDes;
                finishWeldAfterControl = false;
            }
            else
            {
                RobotState.weld_active = false;
                RobotState.weld_phase = 0.0;
                RobotState.weld_segment_index = -1.0;
                RobotState.weld_prepare_phase = 0.0;
                RobotState.weld_recover_phase = 0.0;
                RobotState.weld_tcp_pos_des_W = RobotState.hd_r_pos_W;
                RobotState.weld_tcp_rot_des_W = RobotState.hd_r_rot_W;
                RobotState.weld_tcp_linear_vel_des_W.setZero();
                RobotState.weld_tcp_angular_vel_des_W.setZero();
                RobotState.weld_tcp_linear_acc_des_W.setZero();
                RobotState.weld_tcp_angular_acc_des_W.setZero();
            }

            if (RobotState.motionState == DataBus::Walk2Stand || openLoopPhaseActive)
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

            if (RobotState.motionState == DataBus::Walk || RobotState.motionState == DataBus::Walk2Stand)
            {
                jsInterp.step();
                RobotState.js_pos_des(2) = stand_legLength + foot_height;
                jsInterp.dataBusWrite(RobotState);

                MPC_solv.enable();

                gaitScheduler.dataBusRead(RobotState);
                gaitScheduler.step();
                gaitScheduler.dataBusWrite(RobotState);

                footPlacement.dataBusRead(RobotState);
                footPlacement.getSwingPos();
                footPlacement.dataBusWrite(RobotState);
            }
            else{
                MPC_solv.disable();
            }

            if (openLoopPhaseActive || RobotState.motionState == DataBus::Walk2Stand)
            {
                WBC_solv.setQini(qIniDes, RobotState.q);
                WBC_solv.fe_l_pos_des_W = RobotState.fe_l_pos_W;
                WBC_solv.fe_r_pos_des_W = RobotState.fe_r_pos_W;
                WBC_solv.fe_l_rot_des_W = RobotState.fe_l_rot_W;
                WBC_solv.fe_r_rot_des_W = RobotState.fe_r_rot_W;
                WBC_solv.pCoMDes = RobotState.pCoM_W;
            }

            // MPC
            mpcCtrlCount = mpcCtrlCount + 1;
            if (mpcCtrlCount >= mpcCtrlDecimation) {
                MPC_solv.dataBusRead(RobotState);
                MPC_solv.cal();
                mpcCtrlCount = 0;
            }

            if (RobotState.motionState==DataBus::Walk || RobotState.motionState==DataBus::Walk2Stand) {
                MPC_solv.dataBusWrite(RobotState);
            }
            else {
                RobotState.Fr_ff = Eigen::VectorXd::Zero(12);
                RobotState.des_ddq = Eigen::VectorXd::Zero(RobotState.model_nv);
                RobotState.des_dq = Eigen::VectorXd::Zero(RobotState.model_nv);
                RobotState.des_delta_q = Eigen::VectorXd::Zero(RobotState.model_nv);
                if (weldHoldOptimizedStance)
                {
                    RobotState.base_rpy_des << 0.0, 0.0, weldStanceBaseXyzYaw(3);
                    RobotState.base_pos_des = weldStanceBaseXyzYaw.head<3>();
                    RobotState.js_pos_des = RobotState.base_pos_des;
                    WBC_solv.pCoMDes = isWeldMotionState(RobotState.motionState)
                                           ? weldCoMDes
                                           : weldStanceCoM_W;
                }
                else
                {
                    RobotState.base_rpy_des << 0.0, 0.0, jsInterp.thetaZ;
                    RobotState.base_pos_des = RobotState.js_pos_des;
                    RobotState.base_pos_des(2) = stand_legLength + foot_height;
                }
                // ~59 kg robot, ~290 N per foot
                RobotState.Fr_ff<<0,0,290,0,0,0,
                        0,0,290,0,0,0;
            }

            // WBC
            WBC_solv.dataBusRead(RobotState);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(RobotState);
            RobotState.weld_cop_margin = computeDoubleSupportCopMargin(RobotState, controllerConfig);
            RobotState.weld_tau_margin = computeJointTorqueMargin(RobotState.wbc_tauJointRes, kinDynSolver.motorMaxTorque);
            if (RobotState.motionState == DataBus::Weld && weldActive && weldTrajectoryReady)
            {
                const auto nominalWeldSample = weldTrajectory.sample(simTime - weldStartTime);
                RobotState.weld_tcp_pos_des_W = nominalWeldSample.pose.pos;
                RobotState.weld_tcp_rot_des_W = nominalWeldSample.pose.quat.toRotationMatrix();
                RobotState.weld_tcp_linear_vel_des_W = nominalWeldSample.linearVel;
                RobotState.weld_tcp_angular_vel_des_W = nominalWeldSample.angularVel;
                RobotState.weld_tcp_linear_acc_des_W = nominalWeldSample.linearAcc;
                RobotState.weld_tcp_angular_acc_des_W = nominalWeldSample.angularAcc;
                RobotState.weld_tcp_pos_err_W =
                    nominalWeldSample.pose.pos - RobotState.weld_tcp_pos_cur_W;
            }
            writeWeldStabilityLogFields();
            if (RobotState.motionState == DataBus::Weld && weldStabilityRefValid)
            {
                const double tcpErr = RobotState.weld_tcp_pos_err_W.norm();
                const double rollPitchDelta =
                    std::hypot(RobotState.weld_base_rpy_delta.x(), RobotState.weld_base_rpy_delta.y());
                weldMaxTcpErr = std::max(weldMaxTcpErr, tcpErr);
                weldMaxBaseDelta = std::max(weldMaxBaseDelta, RobotState.weld_base_delta_norm);
                weldMaxComDelta = std::max(weldMaxComDelta, RobotState.weld_com_delta_norm);
                weldMaxRollPitchDelta = std::max(weldMaxRollPitchDelta, rollPitchDelta);
                weldMinCopMargin = std::min(weldMinCopMargin, RobotState.weld_cop_margin);
                if (RobotState.qp_status != 0)
                {
                    weldBadQpCount++;
                }
            }

            if (isWeldMotionState(RobotState.motionState) && RobotState.qp_status != 0)
            {
                weldActive = false;
                weldPreparing = false;
                weldHolding = false;
                weldRecovering = false;
                finishWeldAfterControl = false;
                RobotState.motionState = DataBus::Stand;
                RobotState.weld_active = false;
                RobotState.weld_recover_phase = 0.0;
                weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                std::cerr << "[Weld] aborted: WBC QP status=" << RobotState.qp_status << std::endl;
            }
            else if (RobotState.motionState == DataBus::WeldPrepare && weldPreparing)
            {
                const double startErr = (weldTrajectory.sample(0.0).pose.pos - RobotState.weld_tcp_pos_cur_W).norm();
                const double prepareElapsed = simTime - weldPrepareStartTime;
                const double preparePhase = smoothStep01(prepareElapsed / weldPrepareMaxT);
                if (preparePhase >= weldPrepareMinPhase && startErr < weldPrepareStartErr)
                {
                    if (weldPrepareReadySince < 0.0)
                    {
                        weldPrepareReadySince = simTime;
                    }
                    if (simTime - weldPrepareReadySince >= weldPrepareHoldT)
                    {
                        weldPreparing = false;
                        weldActive = true;
                        weldHolding = false;
                        weldRecovering = false;
                        weldStartTime = simTime;
                        RobotState.motionState = DataBus::Weld;
                        RobotState.weld_active = true;
                        RobotState.weld_prepare_phase = 1.0;
                        resetWeldStabilityMetrics();
                        std::cout << "[Weld] started at t=" << simTime
                                  << " s after prepare, start_err=" << startErr << " m" << std::endl;
                    }
                }
                else
                {
                    weldPrepareReadySince = -1.0;
                    if (prepareElapsed >= weldPrepareMaxT)
                    {
                        if (startErr < weldPrepareFallbackErr)
                        {
                            weldPreparing = false;
                            weldActive = true;
                            weldHolding = false;
                            weldRecovering = false;
                            weldStartTime = simTime;
                            RobotState.motionState = DataBus::Weld;
                            RobotState.weld_active = true;
                            RobotState.weld_prepare_phase = 1.0;
                            resetWeldStabilityMetrics();
                            std::cout << "[Weld] started with prepare fallback at t=" << simTime
                                      << " s, start_err=" << startErr << " m" << std::endl;
                        }
                        else
                        {
                            weldPreparing = false;
                            weldActive = false;
                            weldHolding = false;
                            weldRecovering = false;
                            finishWeldAfterControl = false;
                            RobotState.motionState = DataBus::Stand;
                            RobotState.weld_active = false;
                            RobotState.weld_recover_phase = 0.0;
                            weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                            std::cerr << "[WeldPrepare] aborted: start_err=" << startErr
                                      << " m after " << prepareElapsed << " s" << std::endl;
                        }
                    }
                }
            }
            else if (RobotState.motionState == DataBus::WeldHold && weldHolding)
            {
                if (simTime - weldHoldStartTime >= weldHoldDuration)
                {
                    weldHolding = false;
                    weldRecovering = true;
                    weldRecoverStartTime = simTime;
                    weldRecoverStartTcpPos_W = weldFinalTcpPos_W;
                    weldRecoverTargetTcp_W = weldPreApproachTcp_W;
                    weldRecoverTargetRot_W = weldTrajectoryReady
                                                  ? weldTrajectory.sample(0.0).pose.quat.toRotationMatrix()
                                                  : weldFinalTcpRot_W;
                    RobotState.motionState = DataBus::WeldRecover;
                    RobotState.weld_recover_phase = 0.0;
                    WBC_solv.setQini(qIniDes, RobotState.q);
                    std::cout << "[WeldRecover] started at t=" << simTime
                              << " s, target_tcp=" << weldRecoverTargetTcp_W.transpose() << std::endl;
                }
            }
            else if (RobotState.motionState == DataBus::WeldRecover && weldRecovering)
            {
                const double recoverElapsed = simTime - weldRecoverStartTime;
                const double recoverErr = (weldRecoverTargetTcp_W - RobotState.weld_tcp_pos_cur_W).norm();
                if (recoverElapsed >= weldRecoverDuration &&
                    (recoverErr < 0.015 || recoverElapsed >= weldRecoverDuration + 0.8))
                {
                    if (recoverErr >= 0.015)
                    {
                        std::cerr << "[WeldRecover] warning: finish with tcp_err="
                                  << recoverErr << " m" << std::endl;
                    }
                    weldActive = false;
                    weldPreparing = false;
                    weldHolding = false;
                    weldRecovering = false;
                    finishWeldAfterControl = false;
                    RobotState.motionState = DataBus::Stand;
                    RobotState.weld_active = false;
                    RobotState.weld_recover_phase = 1.0;
                    weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                    WBC_solv.setQini(qIniDes, RobotState.q);
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    std::cout << "[WeldRecover] completed at t=" << simTime
                              << " s, tcp_err=" << recoverErr << " m" << std::endl;
                }
            }

            // joint command
            if (openLoopPhaseActive)
            {
                RobotState.motors_pos_des = eigen2std(qIniFixedDes);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
            }
            else
            {
                Eigen::Matrix<double, 1, nx> L_diag;
                Eigen::Matrix<double, 1, nu> K_diag;
                L_diag << 1.0, 1.0, 1.0,
                        1e-3, 100.0, 1.0,
                        1e-3, 1e-3, 1e-3,
                        1, 100.0, 1.0;
                K_diag << 1.0, 1.0, 1.0,
                        1.0, 1, 1.0,
                        1.0, 1.0, 1.0,
                        1.0, 1, 1.0,
                        1.0;
                MPC_solv.set_weight(1e-6, L_diag, K_diag);

                Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
                RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, robot_nv - 6, 1));
                RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            }

            publishV4SimRealCommand(simTime);

            // joint PVT controller
            pvtCtr.dataBusRead(RobotState);
            if (openLoopPhaseActive)
            {
                pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
            }
            else
            {
                double kp = 1.;
                double kd = 1.;

                pvtCtr.setJointPD(400 * kp, 15 * kd, "left_hip_roll_joint");
                pvtCtr.setJointPD(200 * kp, 10 * kd, "left_hip_yaw_joint");
                pvtCtr.setJointPD(300 * kp, 10 * kd, "left_hip_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 14 * kd, "left_knee_joint");
                pvtCtr.setJointPD(300 * kp, 18 * kd, "left_ankle_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 16 * kd, "left_ankle_roll_joint");

                pvtCtr.setJointPD(400 * kp, 15 * kd, "right_hip_roll_joint");
                pvtCtr.setJointPD(200 * kp, 10 * kd, "right_hip_yaw_joint");
                pvtCtr.setJointPD(300 * kp, 10 * kd, "right_hip_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 14 * kd, "right_knee_joint");
                pvtCtr.setJointPD(300 * kp, 18 * kd, "right_ankle_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 16 * kd, "right_ankle_roll_joint");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            logCtrlCount++;
            if (logCtrlCount >= logDecimation)
            {
                logCtrlCount = 0;
                logger.startNewLine();
                logger.recItermData("dyn_time", simTime);
            logger.recItermData("motors_pos_cur", RobotState.motors_pos_cur);
            logger.recItermData("motors_pos_des", RobotState.motors_pos_des);
            logger.recItermData("motors_tor_cur", RobotState.motors_tor_cur);
            logger.recItermData("motors_tor_des", RobotState.motors_tor_des);
            logger.recItermData("motors_tor_out", RobotState.motors_tor_out);
            logger.recItermData("motors_vel_cur", RobotState.motors_vel_cur);
            logger.recItermData("motors_vel_des", RobotState.motors_vel_des);
            logger.recItermData("FL_est", RobotState.FL_est);
            logger.recItermData("FR_est", RobotState.FR_est);
            logger.recItermData("wbc_FrRes", RobotState.wbc_FrRes);
            logger.recItermData("base_pos_des", RobotState.base_pos_des);
            logger.recItermData("base_pos", RobotState.base_pos);
            logger.recItermData("base_pos_est", RobotState.base_pos_est);
            logger.recItermData("baseLinVel", RobotState.baseLinVel);
            logger.recItermData("base_vel_est", RobotState.base_vel_est);
            logger.recItermData("base_rpy", RobotState.base_rpy);
            logger.recItermData("eul_est", RobotState.eul_est);
            logger.recItermData("Ufe", RobotState.fe_react_tau_cmd);
            logger.recItermData("phi", RobotState.phi);
            logger.recItermData("phiSwitchMinRuntime", RobotState.phiSwitchMinRuntime);
            logger.recItermData("tSwing", RobotState.tSwing);
            logger.recItermData("mainControlDt", mainCtrlDt);
            logger.recItermData("mpcControlDt", mpcCtrlDt);
            logger.recItermData("mpcPredictionHorizon", static_cast<double>(RobotState.mpcPredictionHorizon));
            logger.recItermData("mpcControlHorizon", static_cast<double>(RobotState.mpcControlHorizon));
            logger.recItermData("js_vel_des", RobotState.js_vel_des);
            logger.recItermData("js_omega_des", RobotState.js_omega_des);
            logger.recItermData("swingDesPosCur_W", RobotState.swingDesPosCur_W);
            logger.recItermData("swingDesPosFinal_W", RobotState.swingDesPosFinal_W);
            logger.recItermData("qpStatus_MPC", static_cast<double>(RobotState.qpStatus_MPC));
            logger.recItermData("legState", RobotState.legState);
            logger.recItermData("motionState", RobotState.motionState);
            logger.recItermData("weld_active", RobotState.weld_active ? 1.0 : 0.0);
            logger.recItermData("weld_segment_index", RobotState.weld_segment_index);
            logger.recItermData("weld_phase", RobotState.weld_phase);
            logger.recItermData("weld_tcp_pos_des", RobotState.weld_tcp_pos_des_W);
            logger.recItermData("weld_tcp_pos_cur", RobotState.weld_tcp_pos_cur_W);
            logger.recItermData("weld_tcp_pos_err", RobotState.weld_tcp_pos_err_W);
            logger.recItermData("weld_tcp_rot_err", RobotState.weld_tcp_rot_err_W);
            logger.recItermData("wbc_qp_status", static_cast<double>(RobotState.qp_status));
            logger.recItermData("weld_stance_valid", RobotState.weld_stance_valid ? 1.0 : 0.0);
            logger.recItermData("weld_stance_base_xyz_yaw", RobotState.weld_stance_base_xyz_yaw);
            logger.recItermData("weld_stance_score", RobotState.weld_stance_score);
            logger.recItermData("weld_stance_max_ik_err", RobotState.weld_stance_max_ik_err);
            logger.recItermData("weld_stance_min_limit_margin", RobotState.weld_stance_min_limit_margin);
            logger.recItermData("weld_stance_com_margin", RobotState.weld_stance_com_margin);
            logger.recItermData("weld_stance_foot_l", RobotState.weld_stance_foot_l_W);
            logger.recItermData("weld_stance_foot_r", RobotState.weld_stance_foot_r_W);
            logger.recItermData("weld_stance_left_arm_q", RobotState.weld_stance_left_arm_q);
            logger.recItermData("weld_preapproach_clearance", RobotState.weld_preapproach_clearance);
            logger.recItermData("weld_prepare_phase", RobotState.weld_prepare_phase);
            logger.recItermData("weld_recover_phase", RobotState.weld_recover_phase);
            logger.recItermData("weld_ang_momentum", RobotState.weld_ang_momentum);
            logger.recItermData("weld_h_ang_norm", RobotState.weld_h_ang_norm);
            logger.recItermData("weld_left_arm_q", RobotState.weld_left_arm_q);
            logger.recItermData("weld_left_arm_balance_vel", RobotState.weld_left_arm_balance_vel);
            logger.recItermData("weld_left_arm_vel_norm", RobotState.weld_left_arm_vel_norm);
            logger.recItermData("weld_base_ref", RobotState.weld_base_ref_W);
            logger.recItermData("weld_base_delta", RobotState.weld_base_delta_W);
            logger.recItermData("weld_base_delta_norm", RobotState.weld_base_delta_norm);
            logger.recItermData("weld_com_ref", RobotState.weld_com_ref_W);
            logger.recItermData("weld_com_delta", RobotState.weld_com_delta_W);
            logger.recItermData("weld_com_delta_norm", RobotState.weld_com_delta_norm);
            logger.recItermData("weld_base_rpy_ref", RobotState.weld_base_rpy_ref);
            logger.recItermData("weld_base_rpy_delta", RobotState.weld_base_rpy_delta);
            logger.recItermData("weld_cop_margin", RobotState.weld_cop_margin);
            logger.recItermData("weld_tau_margin", RobotState.weld_tau_margin);
                logger.recItermData("weld_clearance_margin", RobotState.weld_clearance_margin);
                logger.finishLine();
            }

            if (finishWeldAfterControl && RobotState.motionState == DataBus::Weld)
            {
                weldActive = false;
                weldPreparing = false;
                weldHolding = true;
                weldRecovering = false;
                weldHoldStartTime = simTime;
                const auto finalSample = weldTrajectory.sample(weldTrajectory.totalDuration());
                weldFinalTcpPos_W = finalSample.pose.pos;
                weldFinalTcpRot_W = finalSample.pose.quat.toRotationMatrix();
                RobotState.motionState = DataBus::WeldHold;
                RobotState.weld_active = false;
                RobotState.weld_recover_phase = 0.0;
                weldHoldOptimizedStance = weldStanceReady && weldStanceAuto && weldReapplyOnCloseLoop;
                std::cout << "[Weld] completed at t=" << simTime
                          << " s; holding final TCP for " << weldHoldDuration << " s" << std::endl;
                if (weldFinishTimedOut)
                {
                    std::cerr << "[Weld] warning: finish timeout reached before TCP error < "
                              << weldTcpFinishErr << " m." << std::endl;
                }
                std::cout << "[WeldSummary] max_tcp_err=" << weldMaxTcpErr
                          << " m, max_base_delta=" << weldMaxBaseDelta
                          << " m, max_com_delta=" << weldMaxComDelta
                          << " m, max_roll_pitch_delta=" << weldMaxRollPitchDelta
                          << " rad, min_cop_margin="
                          << (std::isfinite(weldMinCopMargin) ? weldMinCopMargin : 0.0)
                          << " m, bad_qp=" << weldBadQpCount << std::endl;
            }
        }

        if (finiteSimDuration && mj_data->time >= simEndTime)
        {
            mujocoExitReason = "headless sim_end reached";
            break;
        }

        uiController.updateScene();
    };
    std::cout << "[MuJoCo] exit: " << mujocoExitReason << " at t=" << simTime << " s" << std::endl;
    uiController.Close();

    return 0;
}
