import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from astar_octo_msgs.srv import PlanPath
from sensor_msgs_py import point_cloud2
from std_srvs.srv import SetBool
import numpy as np
import heapq
import open3d as o3d
import math
from copy import deepcopy

VOXEL_SIZE = 0.1  # Adjust voxel size as needed
Z_THRESHOLD = 0.5  # Max step height allowed
ROBOT_RADIUS = 0.3  # Robot radius in meters

class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')
        self.occupancy_grid = None
        self.min_bound = None

        pointcloud_topic = "/octomap_point_cloud_centers"
        self.sub_pc2 = self.create_subscription(PointCloud2, pointcloud_topic, self.pointcloud2_callback, 1)
        self.srv = self.create_service(PlanPath, 'plan_path', self.plan_path_callback)

    def pointcloud2_callback(self, msg):
        try:
            points = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in point_cloud2.read_points(msg, skip_nans=True)])
            if points.size == 0:
                self.get_logger().info("No points received in the point cloud")
                return

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, VOXEL_SIZE)

            min_bound = voxel_grid.get_min_bound()
            max_bound = voxel_grid.get_max_bound()
            self.min_bound = deepcopy(min_bound)

            grid_size = ((max_bound - min_bound) / VOXEL_SIZE).astype(int) + 1
            occupancy_grid = np.full(grid_size, -1, dtype=int)

            for voxel in voxel_grid.get_voxels():
                x, y, z = voxel.grid_index
                if 0 <= x < grid_size[0] and 0 <= y < grid_size[1] and 0 <= z < grid_size[2]:
                    occupancy_grid[x, y, z] = 100

            self.occupancy_grid = deepcopy(occupancy_grid)
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def is_within_z_constraint(self, current, neighbor):
        z_constraint = int(Z_THRESHOLD / VOXEL_SIZE)
        return abs(current[2] - neighbor[2]) <= z_constraint

    def is_valid(self, coord):
        return (0 <= coord[0] < self.occupancy_grid.shape[0] and
                0 <= coord[1] < self.occupancy_grid.shape[1] and
                0 <= coord[2] < self.occupancy_grid.shape[2] and
                self.occupancy_grid[coord] != 100)

    def has_no_occupied_cells_above(self, coord, vertical_min=0.3, vertical_range=0.6):
        z_min = coord[2] + int(vertical_min / VOXEL_SIZE)
        z_max = coord[2] + int(vertical_range / VOXEL_SIZE)

        coords_to_check = [(coord[0], coord[1], z) for z in range(z_min, z_max + 1) if self.is_valid((coord[0], coord[1], z))]

        return not any(self.occupancy_grid[c] == 100 for c in coords_to_check)

    def is_cylinder_collision_free(self, coord, radius):
        grid_radius = int(radius / VOXEL_SIZE)
        grid_z_start = int(0.4 / VOXEL_SIZE)
        grid_z_end = int(0.6 / VOXEL_SIZE)

        num_points = int(2 * math.pi * grid_radius)

        for angle in range(0, num_points, 2):
            theta = 2 * math.pi * angle / num_points
            i = int(grid_radius * math.cos(theta))
            j = int(grid_radius * math.sin(theta))

            for k in range(grid_z_start, grid_z_end + 1, 2):
                check_coord = (coord[0] + i, coord[1] + j, coord[2] + k)
                if self.is_valid(check_coord) and self.occupancy_grid[check_coord] == 100:
                    return False
        return True

    def astar(self, start, goal):
        neighbors = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy, dz in neighbors:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                if not self.is_valid(neighbor):
                    continue
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return None

    def plan_path_callback(self, request, response):

        start = (request.start.pose.position.x, request.start.pose.position.y, request.start.pose.position.z)  # Replace with actual start
        goal = (request.goal.pose.position.x, request.goal.pose.position.y, request.goal.pose.position.z)  # Replace with actual goal
        self.get_logger().info(f"Start position: {start}")
        path = self.astar(start, goal)
        if path:
            self.get_logger().info("Path found")
            response.success = True
        else:
            self.get_logger().info("No path found")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    print("Hi from path planner node")
    planner = AStarPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
