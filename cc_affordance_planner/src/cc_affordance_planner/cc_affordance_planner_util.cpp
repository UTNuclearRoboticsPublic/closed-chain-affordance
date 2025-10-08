#include <cc_affordance_planner/cc_affordance_planner_util.hpp>

namespace cc_affordance_planner {

std::vector<cc_affordance_planner::TaskDescription> get_se3_screw_tasks(const std::vector<Eigen::Matrix4d> &se3_screw_path, bool parameterize_linearly) {
  std::vector<cc_affordance_planner::TaskDescription> task_list;  // Output of the function 
  const int segment_density = 2;           // Points per small trajectory segment

  if (parameterize_linearly)
  {
    // --------------------------------------------------------------------------
    // Linear parameterization: translation between successive poses
    // --------------------------------------------------------------------------
    Eigen::Vector3d prev_position = se3_screw_path.front().block<3, 1>(0, 3);

    for (size_t i = 1; i < se3_screw_path.size(); ++i)
    {
      // Compute unit vector and distance between consecutive points
      const Eigen::Vector3d current_position = se3_screw_path[i].block<3, 1>(0, 3);
      const Eigen::Vector3d delta_vec = prev_position - current_position;
      const double distance = delta_vec.norm();
      const Eigen::Vector3d direction = delta_vec.normalized();

      // Fill out task description
      cc_affordance_planner::TaskDescription task;
      task.trajectory_density = segment_density;
      task.vir_screw_order = affordance_util::VirtualScrewOrder::NONE;

      task.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
      task.affordance_info.axis = direction;
      task.affordance_info.location = Eigen::Vector3d::Zero();  // Irrelevant for translation
      task.goal.affordance = distance;

      task_list.push_back(std::move(task));

      // Update previous position
      prev_position = current_position;
    }
  }
  else
  {
    // --------------------------------------------------------------------------
    // Screw parameterization: full SE(3) motion between successive poses
    // --------------------------------------------------------------------------
    for (size_t i = 1; i < se3_screw_path.size(); ++i)
    {
      // Compute screw vector between consecutive se3 matrices along the screw path
      const Eigen::Matrix4d& T_prev = se3_screw_path[i - 1];
      const Eigen::Matrix4d& T_curr = se3_screw_path[i];

      const Eigen::Matrix4d T_rel = affordance_util::TransInv(T_prev) * T_curr;
      const Eigen::Matrix<double, 6, 1> body_twist =
          affordance_util::se3ToVec(affordance_util::MatrixLog6(T_rel));
      const Eigen::Matrix<double, 6, 1> space_twist =
          affordance_util::Adjoint(T_prev) * body_twist;

      const double theta = space_twist.norm();

      const Eigen::Matrix<double, 6, 1> screw = space_twist / theta;

      // Fill out task description
      cc_affordance_planner::TaskDescription task;
      task.trajectory_density = segment_density;
      task.vir_screw_order = affordance_util::VirtualScrewOrder::NONE;

      task.affordance_info.type = affordance_util::ScrewType::SCREW;
      task.affordance_info.screw = screw;
      task.goal.affordance = theta;

      task_list.push_back(std::move(task));
    }
  }

  return task_list;
}
} // namespace cc_affordance_planner
