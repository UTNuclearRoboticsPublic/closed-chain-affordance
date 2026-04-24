///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_util.hpp
//      Project   : cc_affordance_planner
//      Created   : Fall 2023
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#ifndef CC_AFFORDANCE_PLANNER_UTIL
#define CC_AFFORDANCE_PLANNER_UTIL

#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <vector>

namespace cc_affordance_planner
{

/**
 * @brief Converts a discretized SE(3) screw path into task descriptions to cause travel along that path
 *
 * Each pair of consecutive poses in @p se3_screw_path defines a segment task.
 * If @p preserve_orientation is true, segments are treated as pure translations and the original orientation is preserved throughout.
 * Otherwise, full screw motion (rotation + translation) is considered.
 *
 * @param[in] se3_screw_path Discretized SE(3) trajectory along the screw motion.
 * @param[in] preserve_orientation Whether to linearize motion and preserve orientation or use full screw parameterization.
 * @return Vector of TaskDescription objects representing each trajectory segment.
 */
std::vector<cc_affordance_planner::TaskDescription> get_se3_screw_tasks(const std::vector<Eigen::Matrix4d>& se3_screw_path, bool preserve_orientation = false);


} // namespace cc_affordance_planner
#endif // CC_AFFORDANCE_PLANNER_UTIL
