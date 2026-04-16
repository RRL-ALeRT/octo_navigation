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

#ifndef MBF_OCTO_CORE__OCTO_MAPPER_H
#define MBF_OCTO_CORE__OCTO_MAPPER_H

#include <memory>
#include <string>
#include <array>

#include <mbf_octo_core/octo_graph_data.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mbf_octo_core
{

/**
 * @brief Abstract interface for the octomap-based navigation graph and costmap.
 *
 * Implemented by OctoMappingServer in mbf_octo_nav.  Passed to planner plugins
 * via OctoPlanner::initialize() so they can access the live graph and octree
 * without owning any mapping state themselves.
 *
 * Responsibility split:
 *  - OctoMapper (impl: OctoMappingServer): subscribe to octomap, build graph,
 *    classify voxels (floor / wall / stair), compute costmap penalties.
 *  - OctoPlanner (impl: AstarOctoPlanner): path search only — no map ownership.
 */
class OctoMapper
{
public:
  using Ptr = std::shared_ptr<OctoMapper>;

  virtual ~OctoMapper() = default;

  /**
   * @brief Get a planning-ready graph snapshot.
   *
   * Internally runs walkability revalidation and incremental penalty
   * computation for any nodes added since the last call, then returns a
   * deep-copy snapshot of the active graph that is safe to use from the
   * planner thread without holding any lock.
   *
   * The returned GraphData has up-to-date node_penalty entries for all
   * walkable nodes so A* can consume them directly.
   */
  virtual std::shared_ptr<GraphData> getReadyGraph() = 0;

  /**
   * @brief Force a full penalty recomputation on the next getReadyGraph() call.
   *
   * Called by the planner when penalty-relevant parameters change at runtime
   * (e.g. robot_height affects which nodes pass revalidation).
   */
  virtual void markPenaltiesDirty() = 0;

  /**
   * @brief Publish additional visualization markers on the mapper's topic.
   *
   * Used by the planner to publish plan-specific debug markers (e.g. expanded
   * dead-end nodes, start/goal markers) on the same RViz topic as the graph.
   */
  virtual void publishAdditionalMarkers(
    const visualization_msgs::msg::MarkerArray & ma) = 0;

  /**
   * @brief Read-only access to the live octree for A* collision checks.
   *
   * The returned pointer is valid as long as the OctoMapper is alive.
   * No lock is held — callers must not write to the octree.
   */
  virtual const octomap::OcTree * getOctree() const = 0;

  /**
   * @brief Current octree resolution (metres per voxel side).
   */
  virtual double getVoxelSize() const = 0;

  /**
   * @brief frame_id from the most recently received octomap message.
   */
  virtual std::string getMapFrame() const = 0;

  /**
   * @brief Spatial bounds of the current map (never-shrink envelope).
   * @param min_b  Set to the minimum corner (x,y,z) in metres.
   * @param max_b  Set to the maximum corner (x,y,z) in metres.
   */
  virtual void getBounds(std::array<double, 3> & min_b,
                         std::array<double, 3> & max_b) const = 0;

  /**
   * @brief Maximum step/climb height in metres.
   *
   * Declared and owned by the mapper (it drives graph-building heuristics).
   * Exposed here so the planner's live collision checks use the same value.
   */
  virtual double getMaxStepHeight() const = 0;

protected:
  OctoMapper() = default;
};

}  // namespace mbf_octo_core

#endif  // MBF_OCTO_CORE__OCTO_MAPPER_H
