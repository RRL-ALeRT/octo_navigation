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

#ifndef MBF_OCTO_CORE__MESH_GRAPH_DATA_H
#define MBF_OCTO_CORE__MESH_GRAPH_DATA_H

#include <array>
#include <cstdint>
#include <vector>

#include <mesh_msgs/msg/mesh_geometry_stamped.hpp>
#include <mesh_msgs/msg/mesh_vertex_costs_stamped.hpp>

namespace mbf_octo_core
{

/**
 * @brief One self-consistent snapshot of the mesh navigation graph.
 *
 * Bundles the mesh geometry, its per-vertex cost layer, and the two graphs
 * from Wiemann et al.'s Graph Half Edge Mesh: the vertex graph Gv
 * (vertex_adjacency — Dijkstra/potential-field search) and the triangle
 * graph Gt (face_adjacency + face_normals — reconstructing a per-triangle
 * gradient/vector field from a scalar potential field).
 *
 * All arrays are index-aligned: vertex_adjacency[i] and
 * vertex_costs.mesh_vertex_costs.costs[i] both describe
 * mesh.mesh_geometry.vertices[i]; face_adjacency[f] and face_normals[f] both
 * describe mesh.mesh_geometry.faces[f].
 *
 * Deep-copied under a single lock by MeshMapper::getReadyMeshGraph(), so a
 * planner can search it without any further synchronization with the mapper
 * (which keeps mutating its live state — including full vertex-index
 * reshuffles on every optimize pass — in the background).
 */
struct MeshGraphData
{
  mesh_msgs::msg::MeshGeometryStamped mesh;
  mesh_msgs::msg::MeshVertexCostsStamped vertex_costs;

  struct VertexEdge
  {
    uint32_t to;
    float length;  // metres
  };
  //! Gv: per-vertex list of (neighbor vertex index, edge length).
  std::vector<std::vector<VertexEdge>> vertex_adjacency;

  //! Gt: per-face up to 3 neighbor face indices, one per triangle edge
  //! (v0-v1, v1-v2, v2-v0); -1 marks a boundary edge (no neighbor).
  std::vector<std::array<int32_t, 3>> face_adjacency;
  //! Per-face unit normal (cross product of the triangle's edges).
  std::vector<std::array<float, 3>> face_normals;

  // ---- Robot-footprint collision data (for planners that can't treat the
  // robot as a point) ---------------------------------------------------------
  //! Free vertical space (metres) above each vertex before hitting occupied
  //! space, capped at the mapper's clearance_scan_max_height. A planner
  //! rejects a vertex if this is below the robot's height.
  std::vector<float> clearance_above;
  //! Isotropic minimum distance (metres) to the nearest occupied space at
  //! each vertex's height, capped at the mapper's clearance_scan_max_radius.
  //! A planner rejects a vertex if this is below half the robot's width (plus
  //! margin). Direction-independent: a vertex right at a tight corner can
  //! read as blocked even if the robot could pass through heading straight —
  //! a deliberate simplicity/precision tradeoff, not a bug.
  std::vector<float> clearance_lateral;

  size_t vertexCount() const { return mesh.mesh_geometry.vertices.size(); }
  size_t faceCount() const { return mesh.mesh_geometry.faces.size(); }
};

}  // namespace mbf_octo_core

#endif  // MBF_OCTO_CORE__MESH_GRAPH_DATA_H
