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

#ifndef MBF_OCTO_CORE__MESH_MAPPER_H
#define MBF_OCTO_CORE__MESH_MAPPER_H

#include <memory>
#include <string>

#include <mbf_octo_core/mesh_graph_data.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mbf_octo_core
{

/**
 * @brief Abstract interface for the mesh-based navigation graph.
 *
 * Implemented by MeshMappingServer in mbf_octo_nav. Passed to mesh planner
 * plugins via MeshPlanner::initialize() so they can search the live mesh
 * graph without owning any mapping state themselves.
 *
 * Responsibility split (mirrors OctoMapper / OctoPlanner):
 *  - MeshMapper (impl: MeshMappingServer): greedy-mesh the octomap voxels,
 *    classify vertices (wall / floor incline / stair / edge / corner /
 *    border), compute + inflate the cost layer, build the vertex/triangle
 *    graphs (Gv/Gt).
 *  - MeshPlanner (impl: a planner plugin): potential-field / vector-field
 *    search on the graph only — no mesh ownership.
 */
class MeshMapper
{
public:
  using Ptr = std::shared_ptr<MeshMapper>;

  virtual ~MeshMapper() = default;

  /**
   * @brief Get a planning-ready, self-consistent snapshot of the mesh graph.
   *
   * Returns a deep copy — the mesh, its cost layer, and both graphs (Gv, Gt)
   * are all from the exact same generation, so a planner can search it
   * without any further synchronization with the mapper (which keeps
   * mutating its live state, including full vertex-index reshuffles on every
   * optimize pass, in the background).
   */
  virtual std::shared_ptr<MeshGraphData> getReadyMeshGraph() = 0;

  /**
   * @brief frame_id the mesh is expressed in.
   */
  virtual std::string getMapFrame() const = 0;

  /**
   * @brief Publish additional visualization markers on the mapper's topic.
   *
   * Used by a planner to publish plan-specific debug markers (e.g. the
   * computed vector field) on the same RViz namespace as the mesh.
   */
  virtual void publishAdditionalMarkers(
    const visualization_msgs::msg::MarkerArray & ma) = 0;

protected:
  MeshMapper() = default;
};

}  // namespace mbf_octo_core

#endif  // MBF_OCTO_CORE__MESH_MAPPER_H
