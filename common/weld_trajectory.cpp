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

    double startTime = 0.0;
    totalLength_ = 0.0;
    for (auto &seg : loaded)
    {
        seg.startTime = startTime;
        startTime += seg.duration;
        totalLength_ += seg.length;
    }
    totalDuration_ = startTime;
    segments_ = std::move(loaded);
    path_ = path;

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
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
    const Eigen::Vector3d dp = seg.end.pos - seg.start.pos;

    out.pose.pos = seg.start.pos + u * dp;
    out.pose.quat = seg.start.quat.slerp(u, seg.end.quat).normalized();
    out.linearVel = dp / seg.duration;
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
    }
    return out;
}
