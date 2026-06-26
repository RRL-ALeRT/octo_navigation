#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "astar_avoid_holes_octo_planner/srv/find_frontiers.hpp"

#include <vector>
#include <memory>
#include <mutex>
#include <cmath>
#include <algorithm>

using FindFrontiers = astar_avoid_holes_octo_planner::srv::FindFrontiers;

// ---------------------------------------------------------------------------
// Hilfsdatentypen
// ---------------------------------------------------------------------------

struct FrontierVoxel {
  double x, y, z;
};

struct FrontierCluster {
  std::vector<FrontierVoxel> voxels;
  double cx = 0.0, cy = 0.0, cz = 0.0; // Centroid
};

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class FrontierServer : public rclcpp::Node
{
public:
  FrontierServer()
  : Node("frontier_server")
  {
    // --- Parameter ---
    this->declare_parameter("octomap_topic",    "/octomap_binary");
    this->declare_parameter("robot_frame",      "base_footprint");
    this->declare_parameter("map_frame",        "map");
    this->declare_parameter("cluster_radius",   0.5);   // m, euklidisches Clustering

    octomap_topic_    = this->get_parameter("octomap_topic").as_string();
    robot_frame_      = this->get_parameter("robot_frame").as_string();
    map_frame_        = this->get_parameter("map_frame").as_string();
    cluster_radius_   = this->get_parameter("cluster_radius").as_double();

    // --- TF ---
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- OctoMap Subscriber ---
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, rclcpp::QoS(1).durability_volatile(),
      [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        octree_.reset(dynamic_cast<octomap::OcTree*>(
          octomap_msgs::msgToMap(*msg)));
        if (!octree_) {
          RCLCPP_WARN(this->get_logger(), "OctoMap konnte nicht deserialisiert werden.");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "OctoMap empfangen (res=%.3fm).",
            octree_->getResolution());
        }
      });

    // --- Marker Publisher ---
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "frontier_markers", rclcpp::QoS(1));

    // --- Service ---
    service_ = this->create_service<FindFrontiers>(
      "find_frontiers",
      std::bind(&FrontierServer::handle_find_frontiers, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "frontier_server bereit. Service: find_frontiers | "
      "OctoMap: %s | Cluster-Radius: %.2fm",
      octomap_topic_.c_str(), cluster_radius_);
  }

private:
  // ---------------------------------------------------------------------------
  // Service-Handler
  // ---------------------------------------------------------------------------

  void handle_find_frontiers(
    const std::shared_ptr<FindFrontiers::Request>  request,
    const std::shared_ptr<FindFrontiers::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "FindFrontiers aufgerufen (min_cluster_size=%d)", request->min_cluster_size);

    // 1. OctoMap verfügbar?
    std::unique_lock<std::mutex> lock(octree_mutex_);
    if (!octree_) {
      response->success = false;
      response->message = "Keine OctoMap empfangen.";
      return;
    }
    // Kopie des Zeigers für die Verarbeitung – Lock danach freigeben
    auto octree = octree_;
    lock.unlock();

    // 2. Roboterposition in map-Frame holen
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!get_robot_pose(robot_pose)) {
      response->success = false;
      response->message = "Roboterposition konnte nicht aus TF gelesen werden.";
      return;
    }
    const double robot_x = robot_pose.pose.position.x;
    const double robot_y = robot_pose.pose.position.y;

    // Boden-Z per Raycast von base_footprint nach unten ermitteln
    const double floor_z = get_floor_z(*octree, robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(),
      "Roboterposition: (%.2f, %.2f, %.2f) | Boden-Z: %.2f",
      robot_x, robot_y, robot_pose.pose.position.z, floor_z);
    // 3. Frontier-Voxel suchen – nur Voxel auf Bodenhöhe (±1 Voxel)
    std::vector<FrontierVoxel> frontiers = find_frontier_voxels(*octree, floor_z);
    RCLCPP_INFO(this->get_logger(), "%zu Frontier-Voxel gefunden.", frontiers.size());

    if (frontiers.empty()) {
      response->success = true;
      response->message = "Keine Frontiers gefunden – Karte möglicherweise vollständig.";
      return;
    }

    // 4. Euklidisches Clustering
    std::vector<FrontierCluster> clusters = cluster_frontiers(frontiers);
    RCLCPP_INFO(this->get_logger(), "%zu Cluster gebildet.", clusters.size());

    // 5. Cluster filtern (Mindestgröße) und nach Größe sortieren (größter zuerst)
    clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
        [&](const FrontierCluster& c) {
          return static_cast<int>(c.voxels.size()) < request->min_cluster_size;
        }),
      clusters.end());

    std::sort(clusters.begin(), clusters.end(),
      [](const FrontierCluster& a, const FrontierCluster& b) {
        return a.voxels.size() > b.voxels.size();
      });

    RCLCPP_INFO(this->get_logger(),
      "%zu Cluster nach Größenfilter übrig.", clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
      auto goal = geometry_msgs::msg::PoseStamped();
      
      goal.pose.position.x = clusters[i].cx;
      goal.pose.position.y = clusters[i].cy;
      goal.pose.position.z = floor_z;
      

      response->frontier_centroids.push_back(goal);
      response->cluster_sizes.push_back(static_cast<int>(clusters[i].voxels.size()));
 
    }
        

    // 7. Visualisierung publishen
    publish_markers(response->frontier_centroids, floor_z);

    response->success = true;
    response->message = std::to_string(response->frontier_centroids.size()) +
                        " Frontiers zurückgegeben.";
  }

  // ---------------------------------------------------------------------------
  // Raycast nach unten → Boden-Z
  // ---------------------------------------------------------------------------
  // Schießt einen Strahl von der Roboterposition nach unten und gibt die Z-Höhe
  // des ersten belegten Voxels + 1 Voxelgröße zurück (= Navigationsebene).
  // Fallback: robot_z wenn kein Treffer innerhalb von 5m.

  double raycast_floor_z(
    const octomap::OcTree& octree,
    double x, double y, double robot_z)
  {
    const octomap::point3d origin(x, y, robot_z);
    const octomap::point3d direction(0.0, 0.0, -1.0);
    octomap::point3d hit;

    bool found = octree.castRay(origin, direction, hit,
      /*ignoreUnknown=*/true, /*maxRange=*/5.0);

    if (found) {
      // Oberkante des getroffenen Voxels = Boden + halbe Voxelgröße
      return hit.z() + octree.getResolution();
    }

    RCLCPP_WARN(this->get_logger(),
      "Raycast nach unten hat keinen Boden gefunden, nutze robot_z als Fallback.");
    return robot_z;
  }

  double get_floor_z(const octomap::OcTree& octree, double x, double y, double z)
  {
      const double res = octree.getResolution();
      // Suche in einem kleinen Z-Band um die gegebene Position nach einem
      // freien Voxel der einen belegten Voxel darunter hat
      for (double dz = 0.0; dz <= 1.0; dz += res) {
          for (double sign : {0.0, 1.0, -1.0}) {
              double probe_z = z + sign * dz;
              octomap::point3d probe(x, y, probe_z);
              auto* node = octree.search(probe);
              if (!node || octree.isNodeOccupied(node)) continue;

              octomap::point3d below(x, y, probe_z - res);
              auto* below_node = octree.search(below);
              if (below_node && octree.isNodeOccupied(below_node)) {
                  // Voxelmittelpunkt exakt quantisiert zurückgeben
                  return octree.keyToCoord(octree.coordToKey(probe)).z();
              }
          }
      }
      RCLCPP_WARN(this->get_logger(), "Kein Bodenvoxel in der Nähe von Z=%.2f gefunden.", z);
      return z;
  }

  // ---------------------------------------------------------------------------
  // Frontier-Voxel finden
  // ---------------------------------------------------------------------------
  // Nur ±x/±y Nachbarn (kein ±z) – Level-Übergänge erzeugen keine Frontiers.
  // Zwei Filter:
  //   1. Der freie Voxel muss auf der Bodenebene liegen: direkt darunter
  //      muss ein belegter Voxel sein (sonst ist er schwebend/lila).
  //   2. Der unbekannte Nachbar darf selbst keine belegten X/Y-Nachbarn haben
  //      (filtert Wand- und Stufen-Kanten).

  bool unknown_neighbor_has_occupied_xy(
    const octomap::OcTree& octree,
    const octomap::point3d& pos,
    double res)
  {
    const std::array<octomap::point3d, 4> dirs = {{
      { res,  0.0, 0.0}, {-res,  0.0, 0.0},
      { 0.0,  res, 0.0}, { 0.0, -res, 0.0},
    }};
    for (const auto& d : dirs) {
      auto* n = octree.search(pos + d);
      if (n && octree.isNodeOccupied(n)) return true;
    }
    return false;
  }

  std::vector<FrontierVoxel> find_frontier_voxels(
    const octomap::OcTree& octree,
    double floor_z)
  {
    const double res = octree.getResolution();

    // Toleranzband um die Bodenebene: freie Voxel müssen innerhalb liegen
    // (1.5 × res damit Quantisierungsunschärfe kein Problem ist)
    const double z_min = floor_z - res * 0.5;
    const double z_max = floor_z + res * 1.5;

    const std::array<octomap::point3d, 4> directions = {{
      { res,  0.0, 0.0},
      {-res,  0.0, 0.0},
      { 0.0,  res, 0.0},
      { 0.0, -res, 0.0},
    }};

    std::vector<FrontierVoxel> result;

    for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
      if (octree.isNodeOccupied(*it)) continue;

      const octomap::point3d center = it.getCoordinate();
      // Filter 1: nur Voxel auf der Navigationsebene
      if (center.z() < z_min || center.z() > z_max) continue;

      // Filter 2: direkt darunter muss ein belegter Voxel liegen
      // (sonst ist der Voxel schwebend – lila Bereich)
      const octomap::point3d below(center.x(), center.y(), center.z() - res * 1.01);
      auto* below_node = octree.search(below);
      if (!below_node || !octree.isNodeOccupied(below_node)) continue;
      // Frontier-Check: hat einer der 4 horizontalen Nachbarn unbekannten Raum?
      for (const auto& dir : directions) {
        const octomap::point3d neighbor = center + dir;
        if (octree.search(neighbor) == nullptr) {
          // Filter 3: unbekannter Nachbar nicht direkt an Wand/Stufe
          if (!unknown_neighbor_has_occupied_xy(octree, neighbor, res)) {
            result.push_back({center.x(), center.y(), center.z()});
          }
          break;
        }
      }
    }

    return result;
  }

  // ---------------------------------------------------------------------------
  // Euklidisches Clustering
  // ---------------------------------------------------------------------------
  // Einfaches Single-Linkage: ein Voxel kommt in den ersten Cluster,
  // dessen Centroid innerhalb von cluster_radius_ liegt.
  // Für Exploration mit typisch hunderten bis wenigen tausend Frontier-Voxeln
  // ist das schnell genug (keine KD-Tree-Abhängigkeit nötig).

  std::vector<FrontierCluster> cluster_frontiers(
    const std::vector<FrontierVoxel>& voxels)
  {
    std::vector<FrontierCluster> clusters;
    const double r2 = cluster_radius_ * cluster_radius_;

    for (const auto& v : voxels) {
      // Nächsten Cluster suchen
      FrontierCluster* best = nullptr;
      double best_dist2 = r2;

      for (auto& c : clusters) {
        double dx = v.x - c.cx;
        double dy = v.y - c.cy;
        double dz = v.z - c.cz;
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 <= best_dist2) {
          best_dist2 = d2;
          best = &c;
        }
      }

      if (best) {
        // Voxel zu vorhandenem Cluster hinzufügen, Centroid inkrementell updaten
        const double n = static_cast<double>(best->voxels.size());
        best->cx = (best->cx * n + v.x) / (n + 1.0);
        best->cy = (best->cy * n + v.y) / (n + 1.0);
        best->cz = (best->cz * n + v.z) / (n + 1.0);
        best->voxels.push_back(v);
      } else {
        // Neuen Cluster anlegen
        FrontierCluster c;
        c.cx = v.x;
        c.cy = v.y;
        c.cz = v.z;
        c.voxels.push_back(v);
        clusters.push_back(std::move(c));
      }
    }

    return clusters;
  }

  // ---------------------------------------------------------------------------
  // Visualisierung
  // ---------------------------------------------------------------------------
  // Index 0 = größter Cluster → rot, alle anderen → blau.
  // Zusätzlich ein grüner Pfeil-Marker pro Ziel (der zurückgezogene Punkt).

  void publish_markers(
    const std::vector<geometry_msgs::msg::PoseStamped>& goals,
    double robot_z)
  {
    visualization_msgs::msg::MarkerArray array;
    const auto now = this->now();

    // Erst alle alten Marker löschen
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = map_frame_;
    delete_all.header.stamp    = now;
    delete_all.ns              = "frontiers";
    delete_all.action          = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(delete_all);

    for (size_t i = 0; i < goals.size(); ++i) {
      const auto& g = goals[i];
      const bool is_selected = (i == 0);

      // --- Kugel am zurückgezogenen Zielpunkt ---
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = map_frame_;
      sphere.header.stamp    = now;
      sphere.ns              = "frontiers";
      sphere.id              = static_cast<int>(i);
      sphere.type            = visualization_msgs::msg::Marker::SPHERE;
      sphere.action          = visualization_msgs::msg::Marker::ADD;

      sphere.pose = g.pose;
      sphere.pose.position.z = robot_z;

      sphere.scale.x = is_selected ? 0.5 : 0.3;
      sphere.scale.y = is_selected ? 0.5 : 0.3;
      sphere.scale.z = is_selected ? 0.5 : 0.3;

      if (is_selected) {
        // rot
        sphere.color.r = 1.0f; sphere.color.g = 0.0f;
        sphere.color.b = 0.0f; sphere.color.a = 1.0f;
      } else {
        // blau
        sphere.color.r = 0.0f; sphere.color.g = 0.4f;
        sphere.color.b = 1.0f; sphere.color.a = 0.8f;
      }
      sphere.lifetime = rclcpp::Duration::from_seconds(0); // bleibt bis nächster Call
      array.markers.push_back(sphere);

      // --- Pfeil: zeigt Richtung zur Frontier (nur beim ausgewählten) ---
      if (is_selected) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = map_frame_;
        arrow.header.stamp    = now;
        arrow.ns              = "frontiers";
        arrow.id              = 10000; // feste ID damit er überschrieben wird
        arrow.type            = visualization_msgs::msg::Marker::ARROW;
        arrow.action          = visualization_msgs::msg::Marker::ADD;

        arrow.pose = g.pose;
        arrow.pose.position.z = robot_z;

        arrow.scale.x = 0.6;  // Schaftlänge
        arrow.scale.y = 0.08; // Schaftbreite
        arrow.scale.z = 0.12; // Pfeilkopfbreite

        arrow.color.r = 0.0f; arrow.color.g = 1.0f;
        arrow.color.b = 0.0f; arrow.color.a = 1.0f;

        arrow.lifetime = rclcpp::Duration::from_seconds(0);
        array.markers.push_back(arrow);
      }

      // --- Text-Label mit Cluster-Rang ---
      visualization_msgs::msg::Marker label;
      label.header.frame_id = map_frame_;
      label.header.stamp    = now;
      label.ns              = "frontiers";
      label.id              = static_cast<int>(20000 + i);
      label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action          = visualization_msgs::msg::Marker::ADD;

      label.pose = g.pose;
      label.pose.position.z = robot_z + 0.4;

      label.scale.z = 0.2; // Texthöhe

      label.color.r = 1.0f; label.color.g = 1.0f;
      label.color.b = 1.0f; label.color.a = 1.0f;

      label.text = is_selected ? "[1] Ziel" : "[" + std::to_string(i + 1) + "]";
      label.lifetime = rclcpp::Duration::from_seconds(0);
      array.markers.push_back(label);
    }

    marker_pub_->publish(array);
  }

  // ---------------------------------------------------------------------------
  // TF-Hilfsfunktion
  // ---------------------------------------------------------------------------

  bool get_robot_pose(geometry_msgs::msg::PoseStamped& pose_out)
  {
    try {
      geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform(
          map_frame_, robot_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(1.0));

      pose_out.header = tf.header;
      pose_out.pose.position.x = tf.transform.translation.x;
      pose_out.pose.position.y = tf.transform.translation.y;
      pose_out.pose.position.z = tf.transform.translation.z;
      pose_out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(),
        "TF lookup fehlgeschlagen (%s → %s): %s",
        robot_frame_.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
  }

  // ---------------------------------------------------------------------------
  // Member
  // ---------------------------------------------------------------------------

  std::string octomap_topic_;
  std::string robot_frame_;
  std::string map_frame_;
  double cluster_radius_;

  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Service<FindFrontiers>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierServer>());
  rclcpp::shutdown();
  return 0;
}
