#include "weld_trajectory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace
{
std::string trim(const std::string &in)
{
    const auto first = in.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return "";
    }
    const auto last = in.find_last_not_of(" \t\r\n");
    return in.substr(first, last - first + 1);
}

bool parseQuaternion(double qx, double qy, double qz, double qw, Eigen::Quaterniond &quat)
{
    quat = Eigen::Quaterniond(qw, qx, qy, qz);
    const double n = quat.norm();
    if (n < 1e-8 || !std::isfinite(n))
    {
        return false;
    }
    quat.normalize();
    return true;
}
} // namespace

bool WeldTrajectory::loadCsv(const std::string &path, std::string *errMsg)
{
    std::ifstream in(path);
    if (!in.is_open())
    {
        if (errMsg != nullptr)
        {
            *errMsg = "failed to open weld trajectory: " + path;
        }
        return false;
    }

    std::vector<Segment> loaded;
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line))
    {
        lineNo++;
        const auto comment = line.find('#');
        if (comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        line = trim(line);
        if (line.empty())
        {
            continue;
        }

        std::stringstream ss(line);
        std::vector<double> values;
        double value = 0.0;
        while (ss >> value)
        {
            values.push_back(value);
        }
        if (values.size() != 15)
        {
            if (errMsg != nullptr)
            {
                *errMsg = "line " + std::to_string(lineNo) +
                          " must contain 15 numeric fields";
            }
            return false;
        }

        Segment seg;
        seg.start.pos << values[0], values[1], values[2];
        if (!parseQuaternion(values[3], values[4], values[5], values[6], seg.start.quat))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "line " + std::to_string(lineNo) + " has invalid start quaternion";
            }
            return false;
        }
        seg.end.pos << values[7], values[8], values[9];
        if (!parseQuaternion(values[10], values[11], values[12], values[13], seg.end.quat))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "line " + std::to_string(lineNo) + " has invalid end quaternion";
            }
            return false;
        }
        seg.speed = values[14];
        seg.length = (seg.end.pos - seg.start.pos).norm();
        if (seg.speed <= 1e-6 || !std::isfinite(seg.speed))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "line " + std::to_string(lineNo) + " has non-positive speed";
            }
            return false;
        }
        if (seg.length <= 1e-8 || !std::isfinite(seg.length))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "line " + std::to_string(lineNo) + " has zero-length weld segment";
            }
            return false;
        }
        seg.duration = seg.length / seg.speed;
        loaded.push_back(seg);
    }

    if (loaded.empty())
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory has no segments: " + path;
        }
        return false;
    }

    segments_ = std::move(loaded);
    recomputeTiming();
    path_ = path;

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}

bool WeldTrajectory::setSingleSegment(const Pose &start,
                                      const Pose &end,
                                      double speed,
                                      std::string *errMsg)
{
    if (speed <= 1.0e-8 || !std::isfinite(speed))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory speed must be positive";
        }
        return false;
    }
    const double length = (end.pos - start.pos).norm();
    if (length <= 1.0e-8 || !std::isfinite(length))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory segment length must be positive";
        }
        return false;
    }
    if (start.quat.norm() <= 1.0e-8 || end.quat.norm() <= 1.0e-8 ||
        !std::isfinite(start.quat.norm()) || !std::isfinite(end.quat.norm()))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory orientation must be a valid quaternion";
        }
        return false;
    }

    Segment seg;
    seg.start = start;
    seg.start.quat.normalize();
    seg.end = end;
    seg.end.quat.normalize();
    seg.speed = speed;
    segments_.assign(1, seg);
    path_ = "generated_right_arm_tcp_line";
    recomputeTiming();

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}

bool WeldTrajectory::setPolyline(const std::vector<Pose> &points,
                                 double speed,
                                 std::string *errMsg)
{
    if (points.size() < 2)
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory polyline needs at least 2 points";
        }
        return false;
    }
    if (speed <= 1.0e-8 || !std::isfinite(speed))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory speed must be positive";
        }
        return false;
    }

    std::vector<Segment> loaded;
    loaded.reserve(points.size() - 1);
    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        if (points[i].quat.norm() <= 1.0e-8 || points[i + 1].quat.norm() <= 1.0e-8 ||
            !std::isfinite(points[i].quat.norm()) || !std::isfinite(points[i + 1].quat.norm()))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "weld trajectory polyline has invalid orientation";
            }
            return false;
        }

        Segment seg;
        seg.start = points[i];
        seg.end = points[i + 1];
        seg.start.quat.normalize();
        seg.end.quat.normalize();
        seg.speed = speed;
        seg.length = (seg.end.pos - seg.start.pos).norm();
        if (seg.length <= 1.0e-8 || !std::isfinite(seg.length))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "weld trajectory polyline has zero-length segment";
            }
            return false;
        }
        loaded.push_back(seg);
    }

    segments_ = std::move(loaded);
    path_ = "generated_right_arm_tcp_polyline";
    recomputeTiming();

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}

bool WeldTrajectory::scalePathLength(double targetLength, std::string *errMsg)
{
    if (segments_.empty())
    {
        if (errMsg != nullptr)
        {
            *errMsg = "cannot scale empty weld trajectory";
        }
        return false;
    }
    if (targetLength <= 1.0e-8 || !std::isfinite(targetLength))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "target weld length must be positive";
        }
        return false;
    }
    if (totalLength_ <= 1.0e-8 || !std::isfinite(totalLength_))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "loaded weld trajectory has invalid length";
        }
        return false;
    }

    const double scale = targetLength / totalLength_;
    const Eigen::Vector3d center = 0.5 * (segments_.front().start.pos + segments_.back().end.pos);
    for (auto &seg : segments_)
    {
        seg.start.pos = center + scale * (seg.start.pos - center);
        seg.end.pos = center + scale * (seg.end.pos - center);
    }
    recomputeTiming();

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}

bool WeldTrajectory::setSpeed(double speed, std::string *errMsg)
{
    if (segments_.empty())
    {
        if (errMsg != nullptr)
        {
            *errMsg = "cannot set speed on empty weld trajectory";
        }
        return false;
    }
    if (speed <= 1.0e-8 || !std::isfinite(speed))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory speed must be positive";
        }
        return false;
    }
    for (auto &seg : segments_)
    {
        seg.speed = speed;
    }
    recomputeTiming();

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}

bool WeldTrajectory::setFixedOrientation(const Eigen::Quaterniond &quat, std::string *errMsg)
{
    if (segments_.empty())
    {
        if (errMsg != nullptr)
        {
            *errMsg = "cannot set orientation on empty weld trajectory";
        }
        return false;
    }

    const double norm = quat.norm();
    if (norm <= 1.0e-8 || !std::isfinite(norm))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "weld trajectory orientation must be a valid quaternion";
        }
        return false;
    }

    const Eigen::Quaterniond fixedQuat = quat.normalized();
    for (auto &seg : segments_)
    {
        seg.start.quat = fixedQuat;
        seg.end.quat = fixedQuat;
    }

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}

void WeldTrajectory::recomputeTiming()
{
    double startTime = 0.0;
    totalLength_ = 0.0;
    for (auto &seg : segments_)
    {
        seg.length = (seg.end.pos - seg.start.pos).norm();
        seg.duration = seg.length / seg.speed;
        seg.startTime = startTime;
        startTime += seg.duration;
        totalLength_ += seg.length;
    }
    totalDuration_ = startTime;
}

void WeldTrajectory::applyTranslation(const Eigen::Vector3d &offset)
{
    if (offset.squaredNorm() <= 1.0e-18)
    {
        return;
    }
    for (auto &seg : segments_)
    {
        seg.start.pos += offset;
        seg.end.pos += offset;
    }
}

WeldTrajectory::Sample WeldTrajectory::sample(double elapsedSec) const
{
    Sample out;
    if (segments_.empty())
    {
        return out;
    }

    const double t = std::max(0.0, elapsedSec);
    int segId = static_cast<int>(segments_.size()) - 1;
    for (int i = 0; i < static_cast<int>(segments_.size()); i++)
    {
        if (t <= segments_[i].startTime + segments_[i].duration)
        {
            segId = i;
            break;
        }
    }

    const Segment &seg = segments_[segId];
    const double localT = std::clamp(t - seg.startTime, 0.0, seg.duration);
    const double u = std::clamp(localT / seg.duration, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double dsDu = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
    const double d2sDu2 = 60.0 * u - 180.0 * u2 + 120.0 * u3;
    const Eigen::Vector3d dp = seg.end.pos - seg.start.pos;

    out.pose.pos = seg.start.pos + s * dp;
    out.pose.quat = seg.start.quat.slerp(s, seg.end.quat).normalized();
    out.linearVel = (dsDu / seg.duration) * dp;
    out.linearAcc = (d2sDu2 / (seg.duration * seg.duration)) * dp;
    out.segmentIndex = segId;
    out.phase = totalDuration_ > 1e-8 ? std::clamp(t / totalDuration_, 0.0, 1.0) : 1.0;
    out.done = t >= totalDuration_;
    out.valid = true;
    if (out.done)
    {
        out.pose.pos = segments_.back().end.pos;
        out.pose.quat = segments_.back().end.quat;
        out.linearVel.setZero();
        out.angularVel.setZero();
        out.linearAcc.setZero();
        out.angularAcc.setZero();
    }
    return out;
}
