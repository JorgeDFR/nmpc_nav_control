#include "nmpc_nav_control/PathDiscretizer.h"

namespace nmpc_nav_control {

PathDiscretizer::PathDiscretizer(double sample_period, int num_poses, bool is_holonomic) : 
    sample_period_(sample_period), num_poses_(num_poses), is_holonomic_(is_holonomic)
{
    percent_error_dist_treshold_ = 1e-2; // 1.0 %

    if (sample_period >= 1.0) { num_points_per_cycle_ = 20; }
    else { num_points_per_cycle_ = 10; }
}

void PathDiscretizer::getNextNPoses(const TPathList path_list, const double &nearest_sample_u, 
                                    std::vector<Pose> &next_poses)
{
    path_vector.clear();
    for (TPathList::const_iterator it = path_list.begin(); it != path_list.end(); it++) {
        path_vector.push_back(*it);
    }

    double N = path_vector.size(); // Number of parametric curves in the Path
    
    double vel = std::fabs(path_vector[(int)std::floor(nearest_sample_u)].GetVelocity()); // Velocity in the current parametric curve
    double goal_dist = vel * sample_period_;
    double rel = goal_dist / num_points_per_cycle_;  

    double u = nearest_sample_u;
    Pose old_point = getPoseSample(nearest_sample_u);
    Vel old_point_vel = getVelSample(nearest_sample_u);
    double step = rel / std::sqrt(std::pow(old_point_vel.vx, 2) + std::pow(old_point_vel.vy, 2));

    double curr_dist = 0.0;
    while (u < N) {
        u += step;
        u = std::min(u, N);

        Pose new_point = getPoseSample(u);
        curr_dist += std::sqrt(std::pow(new_point.x - old_point.x, 2) + std::pow(new_point.y - old_point.y, 2));

        if ((goal_dist - curr_dist) <= percent_error_dist_treshold_*goal_dist) {
            next_poses.push_back(new_point);
            vel = std::fabs(path_vector[(int)std::min(std::floor(u), N-1)].GetVelocity());
            goal_dist = vel*sample_period_;
            rel = goal_dist / num_points_per_cycle_;
            curr_dist = 0.0;        
        }

        if (num_poses_ == (int)next_poses.size()) { break; } // All points were found

        Vel new_point_vel = getVelSample(u);
        step = rel / std::sqrt(std::pow(new_point_vel.vx, 2) + std::pow(new_point_vel.vy, 2));

        old_point = new_point;
    }

    if (num_poses_ > (int)next_poses.size()) {
        Pose last_point = getPoseSample(N);
        while (num_poses_ > (int)next_poses.size()) {
            next_poses.push_back(last_point);
        }
    }
}
  
PathDiscretizer::Pose PathDiscretizer::getPoseSample(const double &sample_u)
{
    int path_num = std::floor(sample_u);
    double u = sample_u - (double)path_num;
    if (path_num >= (int)path_vector.size()) {
        path_num = path_vector.size() - 1;
        u = 1.0;
    } else if (path_num < 0) {
        path_num = 0;
        u = 0.0;
    }
    parametric_trajectories_common::TPath path = path_vector[path_num];
    Pose result;
    result.x = path.GetX(u);
    result.y = path.GetY(u);
    if (!is_holonomic_) {
        if (path.GetVelocity() >= 0) { result.theta = path.GetTheta(u); }
        else { result.theta = path.GetTheta(u) + M_PI; }
    } else { result.theta = path.GetThetaHolomonic(u); }       
    return result;
}

PathDiscretizer::Vel PathDiscretizer::getVelSample(const double &sample_u)
{
    int path_num = std::floor(sample_u);
    double u = sample_u - (double)path_num;
    if (path_num >= (int)path_vector.size()) {
        path_num = path_vector.size() - 1;
        u = 1.0;
    } else if (path_num < 0) {
        path_num = 0;
        u = 0.0;
    }
    parametric_trajectories_common::TPath path = path_vector[path_num];
    Vel result;
    result.vx = path.GetDX(u);
    result.vy = path.GetDY(u);
    return result;
}

} // namespace nmpc_nav_control