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
 */

#ifndef VECTOR_FIELD_MESH_PLANNER__VECTOR_FIELD_MESH_PLANNER_H
#define VECTOR_FIELD_MESH_PLANNER__VECTOR_FIELD_MESH_PLANNER_H

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mbf_octo_core/mesh_planner.h>
#include <mbf_octo_core/mesh_mapper.h>
#include <mbf_octo_core/mesh_graph_data.h>

namespace vector_field_mesh_planner
{

/**
 * @brief Cost-weighted shortest-path planner searching the vertex graph Gv
 *        built by MeshMappingServer (mbf_octo_core::MeshMapper::
 *        getReadyMeshGraph()).
 *
 * Responsibilities (path search only — mirrors how AstarOctoPlanner relates
 * to OctoMappingServer):
 *  - Snap start/goal to the nearest mesh vertex.
 *  - Run Dijkstra on Gv with edge_weight(u,v) = length(u,v) * (1 +
 *    cost_weight_ * max(vertex_cost[u], vertex_cost[v])), treating vertices
 *    at or above lethal_cost_threshold_ as impassable. Since MeshMappingServer
 *    already folds wall/edge/corner/stair/slope/border classification and
 *    hazard inflation into that single per-vertex cost, this reuses all of
 *    that work directly instead of re-deriving separate roughness/height-diff
 *    terms.
 *  - Backtrack the shortest-path parent chain into a plan.
 *
 * Everything else (meshing, cost classification, cost inflation, Gv/Gt graph
 * construction) lives in MeshMappingServer.
 */
class VectorFieldMeshPlanner : public mbf_octo_core::MeshPlanner
{
public:
  typedef std::shared_ptr<vector_field_mesh_planner::VectorFieldMeshPlanner> Ptr;

  VectorFieldMeshPlanner();
  virtual ~VectorFieldMeshPlanner();

  // ---- MeshPlanner interface -------------------------------------------------

  virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped& start,
                            const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance,
                            std::vector<geometry_msgs::msg::PoseStamped>& plan,
                            double& cost,
                            std::string& message) override;

  virtual bool cancel() override;

  /**
   * @brief Initialise the planner plugin.
   * @param plugin_name  Parameter namespace prefix.
   * @param node         Shared ROS node.
   * @param mapper       Mesh mapping server — owns the mesh, cost layer, and
   *                     Gv/Gt graphs. The planner reads them through this
   *                     interface only.
   */
  virtual bool initialize(const std::string& plugin_name,
                          const rclcpp::Node::SharedPtr& node,
                          const mbf_octo_core::MeshMapper::Ptr& mapper) override;

private:
  // Nearest mesh vertex to world point p (3D Euclidean). Returns false if
  // none within max_vertex_search_dist_.
  bool findNearestVertex(const mbf_octo_core::MeshGraphData& graph,
                         const geometry_msgs::msg::Point& p,
                         uint32_t& out_idx) const;

  // Converts a vertex-index chain (start..target, in order) into a pose
  // sequence, with orientation from each segment's direction. Shared by the
  // full plan on success and the partial plan published for debugging on
  // NO_PATH_FOUND.
  std::vector<geometry_msgs::msg::PoseStamped> chainToPoses(
    const mbf_octo_core::MeshGraphData& graph,
    const std::vector<uint32_t>& chain,
    const std::string& frame_id,
    const rclcpp::Time& stamp) const;

  // ---- ROS handles -----------------------------------------------------------
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // ---- Mapper (injected at initialize) ---------------------------------------
  mbf_octo_core::MeshMapper::Ptr mapper_;

  // ---- Cancel flag -------------------------------------------------------------
  std::atomic_bool cancel_planning_{false};

  // ---- Config -------------------------------------------------------------------
  // Weight applied to vertex cost when computing edge weight.
  double cost_weight_ = 5.0;
  // Vertices at or above this cost are treated as impassable. Costs top out at
  // 1.0 (only a true 3-axis corner reaches it), so the default here is
  // deliberately above that range: primary obstacle avoidance is the weighted
  // edge cost (cost_weight_ above), not a hard cutoff. A doorway/narrow gap is
  // made of wall-edge/corner vertices (cost 0.95-1.0) that are still the only
  // route through — hard-blocking them at a threshold like 0.9 makes an
  // existing path unreachable instead of just discouraged. Lower this (e.g.
  // to 0.95 or 1.0) only if you have genuinely forbidden zones the robot must
  // never enter regardless of whether it's the only route.
  double lethal_cost_threshold_ = 1.5;
  // Max distance (metres) allowed when snapping start/goal to a mesh vertex.
  double max_vertex_search_dist_ = 1.0;

  // ---- Robot footprint (hard collision constraints, not cost-weighted) -----
  // A vertex whose clearance_above/clearance_lateral (from
  // MeshMapper::getReadyMeshGraph()) can't fit the robot is excluded from the
  // search entirely — unlike lethal_cost_threshold_ this isn't a "make it
  // more attractive to avoid" knob, the robot's body physically does not fit.
  //
  // This models a rigid box footprint (uniform clearance needed in every
  // direction at every height band), which is a simplification even for a
  // legged robot like Spot. MeshMappingServer's clearance scan now starts
  // its reference point collision_check_height_offset_ above the raw floor
  // (see its doc comment) specifically so ground-level texture/leg-height
  // bumps don't trip this — if it's still too trigger-happy for your
  // environment, set this false and rely on the cost layer + inflation
  // alone, which degrades gracefully instead of hard-excluding a vertex.
  bool enable_footprint_check_ = true;
  double robot_height_ = 0.9;
  double robot_width_  = 0.3;
  // Added to robot_width_/2 before comparing to clearance_lateral_, since the
  // lateral scan is isotropic (see MeshGraphData::clearance_lateral) and
  // therefore already a bit optimistic through tight turns.
  double collision_margin_ = 0.05;

  // ---- Headroom cost (soft, not a hard exclusion) ----------------------------
  // fitsFootprint() is pass/fail: a vertex under a railing's top rail with
  // 1.5m of clearance passes exactly the same as one with 20m of open sky
  // above, so the planner has no reason to prefer the open route over
  // squeezing under the railing. This turns clearance_above into a smooth
  // cost too: a vertex with less than robot_height_ + headroom_margin_ of
  // headroom gets extra cost that ramps up to lethal-adjacent as it
  // approaches the bare minimum, folded into the same max(...) that produces
  // vcost — so it's discouraged, not blocked (the railing gap may be the
  // only route in some maps). Set to 0 to disable.
  double headroom_margin_ = 0.5;
};

}  // namespace vector_field_mesh_planner

#endif  // VECTOR_FIELD_MESH_PLANNER__VECTOR_FIELD_MESH_PLANNER_H
