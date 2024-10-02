#pragma once

#include <vector>
#include <list>

#include <parametric_trajectories_common/trajectory_common.h>

namespace nmpc_nav_control {

using TPathList = std::list<parametric_trajectories_common::TPath>;

class PathDiscretizer {
  public:
    struct Pose {
      double x;
      double y;
      double theta;
    };
    struct Vel {
      double vx;
      double vy;
    };

    PathDiscretizer(double sample_period, int num_poses, bool is_holonomic = false);
    ~PathDiscretizer() = default;

    void getNextNPoses(const TPathList path_list, const double &nearest_sample_u, std::vector<Pose> &next_poses);
  
  private:
    double sample_period_;
    int num_poses_;
    bool is_holonomic_;
    int num_points_per_cycle_;
    double percent_error_dist_treshold_;
    std::vector<parametric_trajectories_common::TPath> path_vector;
    
    Pose getPoseSample(const double &sample_u);
    Vel getVelSample(const double &sample_u);
};

} // namespace nmpc_nav_control