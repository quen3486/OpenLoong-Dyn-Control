#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

class WeldTrajectory
{
public:
    struct Pose
    {
        Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond quat{Eigen::Quaterniond::Identity()};
    };

    struct Sample
    {
        Pose pose;
        Eigen::Vector3d linearVel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d angularVel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d linearAcc{Eigen::Vector3d::Zero()};
        Eigen::Vector3d angularAcc{Eigen::Vector3d::Zero()};
        double phase{0.0};
        int segmentIndex{-1};
        bool done{false};
        bool valid{false};
    };

    bool loadCsv(const std::string &path, std::string *errMsg = nullptr);
    bool scalePathLength(double targetLength, std::string *errMsg = nullptr);
    void applyTranslation(const Eigen::Vector3d &offset);
    Sample sample(double elapsedSec) const;

    bool valid() const { return !segments_.empty(); }
    const std::string &path() const { return path_; }
    int segmentCount() const { return static_cast<int>(segments_.size()); }
    double totalLength() const { return totalLength_; }
    double totalDuration() const { return totalDuration_; }

private:
    void recomputeTiming();

    struct Segment
    {
        Pose start;
        Pose end;
        double speed{0.0};
        double length{0.0};
        double duration{0.0};
        double startTime{0.0};
    };

    std::vector<Segment> segments_;
    std::string path_;
    double totalLength_{0.0};
    double totalDuration_{0.0};
};
