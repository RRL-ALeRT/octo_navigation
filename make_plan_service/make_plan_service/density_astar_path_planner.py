import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from astar_octo_msgs.srv import PlanPath
import numpy as np
import heapq
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
from typing import Tuple, List, Optional
import math
from copy import deepcopy

VOXEL_SIZE = 0.1  # Adjust voxel size as needed
Z_THRESHOLD = 0.5  # Max step height allowed
ROBOT_RADIUS = 0.3  # Robot radius in meters
DENSITY_WEIGHT = 0.5  # Weight for density in cost function
ALTITUDE_WEIGHT = 1.2  # Weight for altitude changes

class AStar3DService(Node):
    def __init__(self):
        super().__init__('density_astar_path_planner')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.pointcloud_callback,
            10
        )

        self.plan_path_srv = self.create_service(
            PlanPath,
            'density_astar_plan_path',
            self.plan_path_callback
        )

        self.occupancy_grid: Optional[np.ndarray] = None
        self.density_map: Optional[np.ndarray] = None
        self.min_bound: Optional[np.ndarray] = None
        self.max_bound: Optional[np.ndarray] = None
        self.map_received = False
        self.get_logger().info("Density-based 3D A* Pathfinding Service Initialized")

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        try:
            points = np.array([[float(p[0]), float(p[1]), float(p[2])] 
                            for p in pc2.read_points(msg, skip_nans=True)])
            if points.size == 0:
                self.get_logger().warn("No points received in the point cloud")
                return

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, VOXEL_SIZE)

            self.min_bound = deepcopy(voxel_grid.get_min_bound())
            self.max_bound = deepcopy(voxel_grid.get_max_bound())
            grid_size = ((self.max_bound - self.min_bound) / VOXEL_SIZE).astype(int) + 1
            
            # Initialize occupancy and density grids
            occupancy_grid = np.zeros(grid_size, dtype=np.uint8)
            density_map = np.zeros(grid_size, dtype=np.float32)

            occupied_voxels = 0
            for voxel in voxel_grid.get_voxels():
                x, y, z = voxel.grid_index
                if (0 <= x < grid_size[0] and 
                    0 <= y < grid_size[1] and 
                    0 <= z < grid_size[2]):
                    occupancy_grid[x, y, z] = 100  # Occupied
                    occupied_voxels += 1
                    # Update density in a 3x3x3 neighborhood
                    for dx in range(-1, 2):
                        for dy in range(-1, 2):
                            for dz in range(-1, 2):
                                nx, ny, nz = x + dx, y + dy, z + dz
                                if (0 <= nx < grid_size[0] and 
                                    0 <= ny < grid_size[1] and 
                                    0 <= nz < grid_size[2]):
                                    density_map[nx, ny, nz] += 1

            self.occupancy_grid = occupancy_grid
            self.density_map = density_map / np.max(density_map, initial=1.0)  # Normalize to [0, 1]
            self.map_received = True
            #self.get_logger().info(f"Processed point cloud: {points.shape[0]} points, "
            #                     f"{occupied_voxels} occupied voxels, "
            #                     f"grid size: {grid_size}, "
            #                     f"min_bound: {self.min_bound}, "
            #                     f"max_bound: {self.max_bound}, "
            #                     f"max density: {np.max(density_map):.2f}")
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def grid_coordinates(self, x: float, y: float, z: float, z_offset: float = 0.2) -> Tuple[int, int, int]:
        if self.min_bound is None or self.occupancy_grid is None:
            return (0, 0, 0)
        gx = int(np.clip((x - self.min_bound[0]) / VOXEL_SIZE, 0, self.occupancy_grid.shape[0] - 1))
        gy = int(np.clip((y - self.min_bound[1]) / VOXEL_SIZE, 0, self.occupancy_grid.shape[1] - 1))
        gz = int(np.clip((z + z_offset - self.min_bound[2]) / VOXEL_SIZE, 0, self.occupancy_grid.shape[2] - 1))
        return gx, gy, gz

    def is_within_bounds(self, coord: Tuple[int, int, int]) -> bool:
        if self.occupancy_grid is None:
            return False
        return (0 <= coord[0] < self.occupancy_grid.shape[0] and 
                0 <= coord[1] < self.occupancy_grid.shape[1] and 
                0 <= coord[2] < self.occupancy_grid.shape[2])

    def is_occupied_space(self, coord: Tuple[int, int, int]) -> bool:
        return self.occupancy_grid[coord[0], coord[1], coord[2]] == 100

    def get_density(self, coord: Tuple[int, int, int]) -> float:
        return self.density_map[coord[0], coord[1], coord[2]]

    def is_within_z_constraint(self, current: Tuple[int, int, int], neighbor: Tuple[int, int, int]) -> bool:
        z_constraint = int(Z_THRESHOLD / VOXEL_SIZE)
        return abs(current[2] - neighbor[2]) <= z_constraint

    def has_no_occupied_cells_above(self, coord: Tuple[int, int, int], 
                                  vertical_min: float = 0.3, 
                                  vertical_range: float = 0.6) -> bool:
        z_min = coord[2] + int(vertical_min / VOXEL_SIZE)
        z_max = coord[2] + int(vertical_range / VOXEL_SIZE)

        coords_to_check = [(coord[0], coord[1], z) for z in range(z_min, z_max + 1) 
                         if self.is_within_bounds((coord[0], coord[1], z))]
        return not any(self.is_occupied_space(c) for c in coords_to_check)

    def is_cylinder_collision_free(self, coord: Tuple[int, int, int], radius: float) -> bool:
        grid_radius = int(radius / VOXEL_SIZE)
        grid_z_start = int(0.4 / VOXEL_SIZE)
        grid_z_end = int(0.6 / VOXEL_SIZE)
        num_points = int(2 * math.pi * grid_radius)

        for angle in range(num_points):
            theta = 2 * math.pi * angle / num_points
            i = int(grid_radius * math.cos(theta))
            j = int(grid_radius * math.sin(theta))
            for k in range(grid_z_start, grid_z_end + 1):
                check_coord = (coord[0] + i, coord[1] + j, coord[2] + k)
                if self.is_within_bounds(check_coord) and self.is_occupied_space(check_coord):
                    return False
        return True

    def heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int], density: float) -> float:
        chebyshev_distance = max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))
        altitude_cost = ALTITUDE_WEIGHT * abs(a[2] - b[2])
        density_cost = DENSITY_WEIGHT * density
        return chebyshev_distance + altitude_cost + density_cost

    def a_star_3d(self, start: Tuple[int, int, int], goal: Tuple[int, int, int]) -> Optional[List[Tuple[int, int, int]]]:
        if not self.map_received or self.occupancy_grid is None or self.density_map is None:
            self.get_logger().warn("No occupancy or density map available")
            return None

        if not (self.is_within_bounds(start) and self.is_within_bounds(goal)):
            self.get_logger().warn(f"Start {start} or goal {goal} out of bounds")
            return None

        if self.is_occupied_space(start):
            self.get_logger().warn(f"Start {start} is in occupied space")
        if self.is_occupied_space(goal):
            self.get_logger().warn(f"Goal {goal} is in occupied space")

        if self.is_occupied_space(start) or self.is_occupied_space(goal):
            self.get_logger().warn(f"Start {start} or goal {goal} in occupied space")
            return None

        neighbors = [
            (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
            (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1),
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
            (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
            (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
            (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1)
        ]

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal, self.get_density(start))}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for i, j, k in neighbors:
                neighbor = (current[0] + i, current[1] + j, current[2] + k)
                if not self.is_within_bounds(neighbor):
                    continue
                if self.is_occupied_space(neighbor):
                    continue
                if not self.is_within_z_constraint(current, neighbor):
                    continue
                if not self.has_no_occupied_cells_above(neighbor):
                    continue
                if not self.is_cylinder_collision_free(neighbor, ROBOT_RADIUS):
                    continue

                neighbor_density = self.get_density(neighbor)
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                total_cost = tentative_g_score + self.heuristic(neighbor, goal, neighbor_density)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = total_cost
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None

    def distance(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

    def plan_path_callback(self, request: PlanPath.Request, response: PlanPath.Response) -> PlanPath.Response:
        self.get_logger().info(f"Planning path from {request.start.pose.position} "
                             f"to {request.goal.pose.position}")
        
        if not self.map_received or self.occupancy_grid is None:
            self.get_logger().warn("No map data available - waiting for point cloud")
            response.plan = []
            return response

        start_coords = self.grid_coordinates(
            request.start.pose.position.x,
            request.start.pose.position.y,
            request.start.pose.position.z,
            z_offset=0.2  # Adjust the offset as needed
        )
        goal_coords = self.grid_coordinates(
            request.goal.pose.position.x,
            request.goal.pose.position.y,
            request.goal.pose.position.z,
            z_offset=0.2  # Adjust the offset as needed
        )

        self.get_logger().info(f"Start grid coords: {start_coords}")
        self.get_logger().info(f"Goal grid coords: {goal_coords}")
        self.get_logger().info(f"Map bounds in world coords: min {self.min_bound}, max {self.max_bound}")
        self.get_logger().info(f"Start occupancy: {self.is_occupied_space(start_coords)}")
        self.get_logger().info(f"Goal occupancy: {self.is_occupied_space(goal_coords)}")

        path = self.a_star_3d(start_coords, goal_coords)
        if path:
            response.plan = self.convert_path_to_pose_stamped(path)
            response.plan[0] = request.start
            response.plan[-1] = request.goal
            self.get_logger().info(f"Path found with {len(path)} waypoints")
        else:
            response.plan = []
            self.get_logger().warn("No path found - check if start/goal are within map bounds and obstacle-free")
        return response

    def convert_path_to_pose_stamped(self, path: List[Tuple[int, int, int]]) -> List[PoseStamped]:
        poses = []
        for point in path:
            pose = PoseStamped()
            pose.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
            pose.pose.position = Point(
                x=float(point[0] * VOXEL_SIZE + self.min_bound[0]),
                y=float(point[1] * VOXEL_SIZE + self.min_bound[1]),
                z=float(point[2] * VOXEL_SIZE + self.min_bound[2])
            )
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        return poses


def main(args=None):
    rclpy.init(args=args)
    astar_service = AStar3DService()
    rclpy.spin(astar_service)
    astar_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()