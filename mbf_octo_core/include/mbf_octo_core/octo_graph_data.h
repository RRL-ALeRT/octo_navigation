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

#ifndef MBF_OCTO_CORE__OCTO_GRAPH_DATA_H
#define MBF_OCTO_CORE__OCTO_GRAPH_DATA_H

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>

#include <octomap/OcTreeKey.h>
#include <octomap/octomap_types.h>

namespace mbf_octo_core
{

/**
 * @brief A single node in the 3D walkability graph built from the octomap.
 *
 * Each node corresponds to an occupied leaf voxel in the octree that has been
 * classified as a floor surface (walkable) or wall/obstacle (non-walkable).
 */
struct GraphNode
{
  octomap::OcTreeKey key;
  unsigned int depth = 0;
  octomap::point3d center;
  double size = 0.0;
  //! true = floor surface with clearance, false = wall/obstacle
  bool is_walkable = true;
  //! true = node was rescued by stair detection (needs wider adjacency)
  bool is_stair_step = false;

  std::string id() const
  {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%u_%u_%u_%u",
                  key.k[0], key.k[1], key.k[2], depth);
    return std::string(buf);
  }
};

/**
 * @brief All data associated with one snapshot of the walkability graph.
 *
 * The double-buffered design (active / pending) relies on this struct being
 * cheaply swappable via std::shared_ptr.  Heavy members (nodes, adj) are only
 * deep-copied when a new planning snapshot is needed.
 */
struct GraphData
{
  //! Sparse node map: stable node id → GraphNode
  std::unordered_map<std::string, GraphNode> nodes;
  //! Adjacency list: node id → list of adjacent node ids
  std::unordered_map<std::string, std::vector<std::string>> adj;

  // --- Costmap (penalty cache) ---
  //! Final penalty per walkable node (used directly by A*)
  std::unordered_map<std::string, double> node_penalty;
  //! Nodes with non-zero penalty (fast iteration for visualization)
  std::unordered_set<std::string> penalized_nodes;
  //! Cached centroid-shift raw penalty per node (incremental reuse)
  std::unordered_map<std::string, double> node_cs_penalty;
  //! Cached graph-border raw penalty per node (incremental reuse)
  std::unordered_map<std::string, double> node_gb_penalty;
  //! Nodes whose raw penalties (cs + gb) have been computed and cached
  std::unordered_set<std::string> penalty_computed_nodes;

  // --- Incremental build tracking ---
  //! Occupied voxel keys already processed (avoids re-adding nodes)
  std::unordered_set<std::string> processed_occupied_keys;
  //! Node ids whose adjacency edges need updating on next build
  std::unordered_set<std::string> nodes_needing_adjacency_update;

  bool empty() const { return nodes.empty(); }
  size_t size() const { return nodes.size(); }
};

}  // namespace mbf_octo_core

#endif  // MBF_OCTO_CORE__OCTO_GRAPH_DATA_H
