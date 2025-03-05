import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from astar_octo_msgs.srv import PlanPath
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from octomap_msgs.msg import Octomap
from std_srvs.srv import SetBool
import numpy as np
import heapq
import open3d as o3d
import math
from copy import deepcopy

VOXEL_SIZE = 0.05  # Adjust voxel size as needed
Z_THRESHOLD = 0.3  # Max step height allowed
ROBOT_RADIUS = 0.3  # Robot radius in meters
IS_SIMULATION = False
class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')
        self.occupancy_grid = None
        self.occupancy_array = None
        self.min_bound = None

        pointcloud_topic = "/octomap_point_cloud_centers"
        self.sub_pc2 = self.create_subscription(PointCloud2, pointcloud_topic, self.pointcloud2_callback, 1)
        self.srv = self.create_service(PlanPath, 'plan_path', self.plan_path_callback)

        self.path_publisher = self.create_publisher(Path, '/astar_path', 1)

    def world_to_grid(self, world_coords, min_bound, voxel_size):
        return tuple(
            int((world_coords[i] - min_bound[i]) / voxel_size)
            for i in range(3)
            )


    def grid_to_world(self, grid_coords, min_bound, voxel_size):
        return tuple(
            grid_coords[i] * voxel_size + min_bound[i]
            for i in range(3)
            )


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
            self.get_logger().info(f"Voxel grid created, min bound: {min_bound}, max bound: {max_bound}")
            self.min_bound = deepcopy(min_bound)
            grid_size_x = int((max_bound[0] - min_bound[0]) / VOXEL_SIZE) + 1
            grid_size_y = int((max_bound[1] - min_bound[1]) / VOXEL_SIZE) + 1
            grid_size_z = int((max_bound[2] - min_bound[2]) / VOXEL_SIZE) + 1

            #grid_size = ((max_bound - min_bound) / VOXEL_SIZE).astype(int) + 1
            occupancy_grid = np.full((grid_size_x, grid_size_y, grid_size_z), -1, dtype=int)
            for voxel in voxel_grid.get_voxels():
                grid_index = voxel.grid_index
                x, y, z = grid_index

                # Ensure indices are within bounds
                if 0 <= x < grid_size_x and 0 <= y < grid_size_y and 0 <= z < grid_size_z:
                    occupancy_grid[x, y, z] = 100  # Mark the voxel as occupied

            # Store the occupancy grid
            self.occupancy_grid = deepcopy(occupancy_grid)
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")
    def find_nearest_3d_point(self, x, y, z, array_3d):
        indices = np.argwhere(array_3d == 100)

        distances = np.sqrt((indices[:, 0] - x)**2 + (indices[:, 1] - y)**2 + (indices[:, 2] - z)**2)
        nearest_index = indices[np.argmin(distances)]

        return tuple(nearest_index)

    def astar(self, start, goal):
        self.occupancy_array = self.occupancy_grid
        #start = self.world_to_grid(start, self.min_bound, VOXEL_SIZE)
        #goal = self.world_to_grid(goal, self.min_bound, VOXEL_SIZE)
        self.get_logger().info(f"World Start: {start}")
        self.get_logger().info(f"World Goal: {goal}")
        #start = self.find_nearest_3d_point(start[0], start[1], start[2], self.occupancy_grid)
        #goal = self.find_nearest_3d_point(goal[0], goal[1], goal[2], self.occupancy_grid)
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

        # Check if the calculated indices are within the bounds of the occupancy grid
        if not (0 <= start[0] < self.occupancy_grid.shape[0] and
                0 <= start[1] < self.occupancy_grid.shape[1] and
                0 <= start[2] < self.occupancy_grid.shape[2]):
            self.get_logger().warn(f"Start coordinates {start} are out of bounds.")
            return None

        if not (0 <= goal[0] < self.occupancy_grid.shape[0] and
                0 <= goal[1] < self.occupancy_grid.shape[1] and
                0 <= goal[2] < self.occupancy_grid.shape[2]):
            self.get_logger().warn(f"Goal coordinates {goal} are out of bounds.")
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
                #self.get_logger().info(f"Checking neighbor: {neighbor}")

                # Ensure the neighbor is within the grid bounds
                if not (0 <= neighbor[0] < self.occupancy_array.shape[0] and
                    0 <= neighbor[1] < self.occupancy_array.shape[1] and
                    0 <= neighbor[2] < self.occupancy_array.shape[2]):
                    self.get_logger().info(f"Neighbor {neighbor} out of bounds")
                    continue

                # Plan through occupied cells only
                if self.occupancy_array[neighbor[0], neighbor[1], neighbor[2]] != 100:
                    self.get_logger().info(f"Neighbor {neighbor} is not occupied")
                    continue

                # Check z constraint
                if not self.is_within_z_constraint(current, neighbor):
                    self.get_logger().info(f"Neighbor {neighbor} not within z constraint")
                    continue

                # Check if there are occupied cells above the neighbor
                # if not self.has_no_occupied_cells_above(neighbor):
                #     self.get_logger().info(f"Neighbor {neighbor} has occupied cells above")
                #     continue

                # if not self.is_cylinder_collision_free(neighbor, ROBOT_RADIUS):
                #     self.get_logger().info(f"Neighbor {neighbor} is not collision-free")
                #     continue

                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    self.get_logger().info(f"✅ Updating {neighbor}: g={tentative_g_score}")
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  # Return None if no path is found

    def heuristic(self,a, b):
        # Euclidean distance as heuristic
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5

    def is_within_z_constraint(self,current, neighbor):
        z_constraint = int(Z_THRESHOLD / VOXEL_SIZE)
        return abs(current[2] - neighbor[2]) <= z_constraint

    def is_within_bounds(self, coord):
        return 0 <= coord[0] < self.occupancy_array.shape[0] and 0 <= coord[1] < self.occupancy_array.shape[1] and 0 <= coord[2] < self.occupancy_array.shape[2]

    def is_occupied_space(self, coord):
        #coord = tuple(map(int, coord))
        return self.occupancy_array[coord[0], coord[1], coord[2]] == 100

    def has_no_occupied_cells_above(self, coord, vertical_min=0.3, vertical_range=0.6):
        z_min = coord[2] + int(vertical_min / VOXEL_SIZE)  # Start checking
        z_max = coord[2] + int(vertical_range / VOXEL_SIZE)  # End checking

        for z in range(z_min, z_max + 1):
            if self.is_within_bounds((coord[0], coord[1], z)):
                if self.is_occupied_space((coord[0], coord[1], z)):  # Found an obstacle
                    return True  # Allow stepping over it
        return False

    def is_cylinder_collision_free(self, coord, radius):
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



    def plan_path_callback(self, request, response):
        start = ((request.start.pose.position.x), (request.start.pose.position.y), (request.start.pose.position.z))
        goal = ((request.goal.pose.position.x), (request.goal.pose.position.y), (request.goal.pose.position.z))

        self.get_logger().info(f"Start position: {start}")
        self.get_logger().info(f"Goal position: {goal}")

        #path = self.astar(start, goal)
        path = [start, goal]
        if path:
            self.get_logger().info("Path found")
            response.plan = [self.create_pose_stamped(coord) for coord in path]
            self.publish_path(path)
        else:
            self.get_logger().info("No path found")
            response.plan = []

        return response

    def create_pose_stamped(self, coord):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"  # Set the appropriate frame ID
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(coord[0])
        pose_stamped.pose.position.y = float(coord[1])
        pose_stamped.pose.position.z = float(coord[2])
        pose_stamped.pose.orientation.w = 1.0  # Set a default orientation
        return pose_stamped

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = [self.create_pose_stamped(coord) for coord in path]
        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    print("Hi from path planner node")
    planner = AStarPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
