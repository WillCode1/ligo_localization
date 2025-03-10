#pragma once
#include <Eigen/Dense>

class evo_tool
{
public:
    evo_tool(const std::string& trajectory_path)
    {
        pose_trajectory = fopen(trajectory_path.c_str(), "w");
        fprintf(pose_trajectory, "# target trajectory\n# timestamp tx ty tz qx qy qz qw\n");
        fflush(pose_trajectory);
    }
    ~evo_tool()
    {
        fclose(pose_trajectory);
    }

    void save_trajectory(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, const double &time)
    {
        fprintf(pose_trajectory, "%0.4lf %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", time,
                pos.x(), pos.y(), pos.z(), quat.x(), quat.y(), quat.z(), quat.w());
        fflush(pose_trajectory);
    }

    void save_trajectory(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot, const double &time)
    {
        Eigen::Quaterniond quat(rot);
        save_trajectory(pos, quat, time);
    }

    FILE *pose_trajectory;
};
