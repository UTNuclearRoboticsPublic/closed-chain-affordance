///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_util.hpp
//      Project   : cc_affordance_planner
//      Created   : Fall 2023
//      Author    : Janak Panthi (Crasun Jans)
//      Copyright : Copyright© The University of Texas at Austin, 2014-2026. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
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
 * If @p parameterize_linearly is true, segments are treated as pure translations, i,e. the original orientation is preserved throughout.
 * Otherwise, full screw motion (rotation + translation) is parameterized.
 *
 * @param[in] se3_screw_path Discretized SE(3) trajectory along the screw motion.
 * @param[in] parameterize_linearly Whether to linearize motion (translation-only) or use full screw parameterization.
 * @return Vector of TaskDescription objects representing each trajectory segment.
 */
std::vector<cc_affordance_planner::TaskDescription> get_se3_screw_tasks(const std::vector<Eigen::Matrix4d>& se3_screw_path, bool parameterize_linearly = false);


} // namespace cc_affordance_planner
#endif // CC_AFFORDANCE_PLANNER_UTIL
