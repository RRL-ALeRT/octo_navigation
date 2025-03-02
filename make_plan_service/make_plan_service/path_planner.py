import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from astar_octo_msgs.srv import PlanPath
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
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
        self.occupancy_array = None
        self.min_bound = None
        self.current_pose = None

        pointcloud_topic = "/octomap_point_cloud_centers"
        self.sub_pc2 = self.create_subscription(PointCloud2, pointcloud_topic, self.pointcloud2_callback, 1)
        self.sub_pose = self.create_subscription(Odometry, "/Spot/odometry", self.pose_callback, 1)
        self.srv = self.create_service(PlanPath, 'plan_path', self.plan_path_callback)

    def pose_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def pointcloud2_callback(self, msg: PointCloud2):
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

    def astar(self, start: tuple, goal: tuple):
        self.occupancy_array = self.occupancy_grid
        neighbors = [
            # 3D diagonals (corners) - prioritized first
            (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
            (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1),
            # x-y plane diagonals
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
            # x-z plane diagonals
            (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
            # y-z plane diagonals
            (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
            # x-direction
            (1, 0, 0), (-1, 0, 0),
            # y-direction
            (0, 1, 0), (0, -1, 0),
            # z-direction
            (0, 0, 1), (0, 0, -1)
        ]
        if not self.is_within_bounds(start) or not self.is_within_bounds(goal):
            self.get_logger().error("Start or goal out of bounds")
            return None
        if self.is_occupied_space(start) or self.is_occupied_space(goal):
            self.get_logger().error(f"Start or goal in occupied space: start={start}, goal={goal}")
            self.get_logger().error(f"Occupancy at start: {self.occupancy_array[start]}, Occupancy at goal: {self.occupancy_array[goal]}")
            return None

        open_list = []
        heapq.heappush(open_list, (0, start))  # Priority queue with (f-score, node)

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        self.get_logger().info(f"Initial open_list: {open_list}")
        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                # Reconstruct the path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for i, j, k in neighbors:
                neighbor = (current[0] + i, current[1] + j, current[2] + k)
                # Ensure the neighbor is within the grid bounds
                if not self.is_within_bounds(neighbor):
                    continue

                # Avoid occupied cells
                if self.is_occupied_space(neighbor):
                    continue

                # Check z constraint
                if not self.is_within_z_constraint(current, neighbor):
                    continue

                # Check if there are occupied cells above the neighbor
                if not self.has_no_occupied_cells_above(neighbor):
                    continue

                if not self.is_cylinder_collision_free(neighbor, ROBOT_RADIUS):
                    continue

                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  # Return None if no path is found

    def heuristic(self, a: tuple, b: tuple) -> float:
        # Euclidean distance as heuristic
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5

    def is_within_z_constraint(self, current: tuple, neighbor: tuple) -> bool:
        z_constraint = int(Z_THRESHOLD / VOXEL_SIZE)
        return abs(current[2] - neighbor[2]) <= z_constraint

    def is_within_bounds(self, coord: tuple) -> bool:
        return 0 <= coord[0] < self.occupancy_array.shape[0] and 0 <= coord[1] < self.occupancy_array.shape[1] and 0 <= coord[2] < self.occupancy_array.shape[2]

    def is_occupied_space(self, coord: tuple) -> bool:
        coord = tuple(map(int, coord))
        return self.occupancy_array[coord[0], coord[1], coord[2]] == 100

    def has_no_occupied_cells_above(self, coord: tuple, vertical_min: float = 0.3, vertical_range: float = 0.6) -> bool:
        z_min = coord[2] + int(vertical_min / VOXEL_SIZE)  # Start checking
        z_max = coord[2] + int(vertical_range / VOXEL_SIZE)  # End checking within the vertical range

        # Use list comprehension to create a list of coordinates to check
        coords_to_check = [(coord[0], coord[1], z) for z in range(z_min, z_max + 1) if self.is_within_bounds((coord[0], coord[1], z))]

        # Check all coordinates in one batch if possible
        return not any(self.is_occupied_space(c) for c in coords_to_check)

    def is_cylinder_collision_free(self, coord: tuple, radius: float) -> bool:
        # Convert radius and base offset from meters to voxel grid units
        grid_radius = radius / VOXEL_SIZE
        grid_z_start = int(0.4 / VOXEL_SIZE)
        grid_z_end = int(0.6 / VOXEL_SIZE)

        # Calculate the number of points to check around the circumference
        num_points = int(2 * math.pi * grid_radius)

        # Iterate over the points on the circumference
        for angle in range(0, num_points, 2):
            theta = 2 * math.pi * angle / num_points
            i = int(grid_radius * math.cos(theta))
            j = int(grid_radius * math.sin(theta))

            for k in range(grid_z_start, grid_z_end + 1, 2):
                check_coord = (coord[0] + i, coord[1] + j, coord[2] + k)
                if self.is_within_bounds(check_coord) and self.is_occupied_space(check_coord):
                    return False  # Early exit if any occupied space is found

        return True  # No collision found

    def plan_path_callback(self, request: PlanPath.Request, response: PlanPath.Response) -> PlanPath.Response:
        if self.current_pose:
            start = (int(self.current_pose.position.x), int(self.current_pose.position.y), int(self.current_pose.position.z))
        else:
            start = (int(request.start.pose.position.x), int(request.start.pose.position.y), int(request.start.pose.position.z))
        
        goal = (int(request.goal.pose.position.x), int(request.goal.pose.position.y), int(request.goal.pose.position.z))

        self.get_logger().info(f"Start position: {start}")
        self.get_logger().info(f"Goal position: {goal}")

        path = self.astar(start, goal)

        if path:
            self.get_logger().info("Path found")
            response.plan = [self.create_pose_stamped(coord) for coord in path]
        else:
            self.get_logger().info("No path found")
            response.plan = []

        return response

    def create_pose_stamped(self, coord: tuple) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"  # Set the appropriate frame ID
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(coord[0])
        pose_stamped.pose.position.y = float(coord[1])
        pose_stamped.pose.position.z = float(coord[2])
        pose_stamped.pose.orientation.w = 1.0  # Set a default orientation
        return pose_stamped


def main(args=None):
    rclpy.init(args=args)
    print("Hi from path planner node")
    planner = AStarPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
