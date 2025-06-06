/*
 *  Copyright 2025, Mascor Institute
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  author: Mascor Institute
 *
 */

#ifndef MBF_OCTO_CORE__OCTO_PLANNER_H
#define MBF_OCTO_CORE__OCTO_PLANNER_H

#include <string>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mbf_abstract_core/abstract_planner.h>
#include <rclcpp/rclcpp.hpp>


namespace mbf_octo_core
{
class OctoPlanner : public mbf_abstract_core::AbstractPlanner
{
public:
  typedef std::shared_ptr<mbf_octo_core::OctoPlanner> Ptr;

  /**
   * @brief Destructor
   */
  virtual ~OctoPlanner() {};

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can
   * relax the constraint in x and y before failing
   * @param plan The plan... filled by the planner
   * @param cost The cost for the the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result. (see GetPath.action)
   */
  virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                            std::string& message) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not
   * implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Initializes the planner plugin with a user configured name and a shared pointer to the mesh map
   * @param name The user configured name, which is used as namespace for parameters, etc.
   * @param mesh_map_ptr A shared pointer to the mesh map instance to access attributes and helper functions, etc.
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& name, const rclcpp::Node::SharedPtr& node) = 0;

protected:
  OctoPlanner() {};
};

} /* namespace mbf_octo_core */

#endif /* MBF_OCTO_CORE__OCTO_PLANNER_H */
