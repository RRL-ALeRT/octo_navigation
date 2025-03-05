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

VOXEL_SIZE = 0.2
ROBOT_RADIUS = 0.3
DENSITY_WEIGHT = 0.8
OCCUPANCY_THRESHOLD = 0.8
GROUND_OFFSET = 0.1
ROBOT_HEIGHT = 0.6
MAX_SLOPE = 0.5
MAX_STEP_HEIGHT = 0.2

class AStar3DService(Node):
    def __init__(self):
        super().__init__('a_star_3d_service')
        self.subscription = self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.pointcloud_callback, 10)
        self.plan_path_srv = self.create_service(PlanPath, 'density_astar_plan_path', self.plan_path_callback)
        self.occupancy_grid_3d = None
        self.density_map_3d = None
        self.occupancy_grid_2d = None
        self.density_map_2d = None
        self.min_bound = None
        self.max_bound = None
        self.map_received = False
        self.ground_level = 0.0
        self.fixed_z = 0
        self.get_logger().info("Density-based 2D A* Pathfinding Service Initialized (with slope-limited z adjustments)")

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        try:
            points = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in pc2.read_points(msg, skip_nans=True)])
            if points.size == 0:
                self.get_logger().info("No points received in the point cloud")
                return
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, VOXEL_SIZE)
            self.min_bound = deepcopy(voxel_grid.get_min_bound())
            self.max_bound = deepcopy(voxel_grid.get_max_bound())
            grid_size = ((self.max_bound - self.min_bound) / VOXEL_SIZE).astype(int) + 1
            max_allowed_size = 10000
            if np.any(grid_size > max_allowed_size):
                self.get_logger().error(f"Grid size {grid_size} exceeds maximum allowed size {max_allowed_size}")
                raise ValueError("Environment too large")

            self.occupancy_grid_3d = np.zeros(grid_size, dtype=np.uint8)
            self.density_map_3d = np.zeros(grid_size, dtype=np.float32)
            occupied_voxels = 0
            min_z_traversable = float('inf')
            for voxel in voxel_grid.get_voxels():
                x, y, z = voxel.grid_index
                if 0 <= x < grid_size[0] and 0 <= y < grid_size[1] and 0 <= z < grid_size[2]:
                    self.occupancy_grid_3d[x, y, z] = 100
                    occupied_voxels += 1
                    for dx in range(-1, 2):
                        for dy in range(-1, 2):
                            for dz in range(-1, 2):
                                nx, ny, nz = x + dx, y + dy, z + dz
                                if 0 <= nx < grid_size[0] and 0 <= ny < grid_size[1] and 0 <= nz < grid_size[2]:
                                    self.density_map_3d[nx, ny, nz] += 1
                                    if self.density_map_3d[nx, ny, nz] < OCCUPANCY_THRESHOLD:
                                        min_z_traversable = min(min_z_traversable, nz)

            max_density = np.max(self.density_map_3d)
            self.density_map_3d = self.density_map_3d / (max_density if max_density > 0 else 1.0)

            self.ground_level = self.min_bound[2] + min_z_traversable * VOXEL_SIZE if min_z_traversable != float('inf') else self.min_bound[2]
            self.fixed_z = int((self.ground_level + GROUND_OFFSET - self.min_bound[2]) / VOXEL_SIZE)
            if not 0 <= self.fixed_z < grid_size[2]:
                self.get_logger().warn(f"Fixed z {self.fixed_z} out of bounds; clamping")
                self.fixed_z = max(0, min(self.fixed_z, grid_size[2] - 1))

            self.occupancy_grid_2d = np.zeros((grid_size[0], grid_size[1]), dtype=np.uint8)
            self.density_map_2d = np.zeros((grid_size[0], grid_size[1]), dtype=np.float32)
            z_range = int(ROBOT_HEIGHT / VOXEL_SIZE)
            for x in range(grid_size[0]):
                for y in range(grid_size[1]):
                    is_occupied = False
                    max_density_xy = 0.0
                    for z in range(self.fixed_z, min(self.fixed_z + z_range + 1, grid_size[2])):
                        density = self.density_map_3d[x, y, z]
                        if density >= OCCUPANCY_THRESHOLD:
                            is_occupied = True
                            break
                        max_density_xy = max(max_density_xy, density)
                    self.occupancy_grid_2d[x, y] = 100 if is_occupied else 0
                    self.density_map_2d[x, y] = max_density_xy if not is_occupied else 1.0

            self.map_received = True
            self.get_logger().info(f"Processed point cloud: {points.shape[0]} points, {occupied_voxels} occupied voxels, grid size: {grid_size}, min_bound: {self.min_bound}, max_bound: {self.max_bound}, max density: {max_density:.2f}, ground level: {self.ground_level:.2f}, fixed z: {self.fixed_z}")
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def grid_coordinates(self, x: float, y: float, z: float) -> Tuple[int, int]:
        if self.min_bound is None or self.occupancy_grid_2d is None:
            self.get_logger().warn("Grid coordinates requested before map initialization")
            raise ValueError("Map not initialized")
        gx = int(np.clip((x - self.min_bound[0]) / VOXEL_SIZE, 0, self.occupancy_grid_2d.shape[0] - 1))
        gy = int(np.clip((y - self.min_bound[1]) / VOXEL_SIZE, 0, self.occupancy_grid_2d.shape[1] - 1))
        return gx, gy

    def is_within_bounds_2d(self, coord: Tuple[int, int]) -> bool:
        if self.occupancy_grid_2d is None:
            return False
        return (0 <= coord[0] < self.occupancy_grid_2d.shape[0] and 0 <= coord[1] < self.occupancy_grid_2d.shape[1])

    def is_within_bounds_3d(self, coord: Tuple[int, int, int]) -> bool:
        if self.occupancy_grid_3d is None:
            return False
        return (0 <= coord[0] < self.occupancy_grid_3d.shape[0] and 0 <= coord[1] < self.occupancy_grid_3d.shape[1] and 0 <= coord[2] < self.occupancy_grid_3d.shape[2])

    def get_density_2d(self, coord: Tuple[int, int]) -> float:
        return self.density_map_2d[coord[0], coord[1]]

    def get_density_3d(self, coord: Tuple[int, int, int]) -> float:
        return self.density_map_3d[coord[0], coord[1], coord[2]]

    def is_traversable_2d(self, coord: Tuple[int, int]) -> bool:
        return self.get_density_2d(coord) < OCCUPANCY_THRESHOLD

    def is_traversable_3d(self, coord: Tuple[int, int, int]) -> bool:
        return self.get_density_3d(coord) < OCCUPANCY_THRESHOLD and self.is_clear_above(coord)

    def is_clear_above(self, coord: Tuple[int, int, int]) -> bool:
        z_range = int(ROBOT_HEIGHT / VOXEL_SIZE)
        for z in range(coord[2] + 1, min(coord[2] + z_range + 1, self.occupancy_grid_3d.shape[2])):
            if self.is_within_bounds_3d((coord[0], coord[1], z)) and self.get_density_3d((coord[0], coord[1], z)) >= OCCUPANCY_THRESHOLD:
                return False
        return True

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int], density: float) -> float:
        chebyshev_distance = max(abs(a[0] - b[0]), abs(a[1] - b[1]))
        density_cost = DENSITY_WEIGHT * density
        return chebyshev_distance + density_cost

    def distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def find_nearest_traversable(self, coord: Tuple[int, int]) -> Tuple[int, int]:
        if self.is_traversable_2d(coord):
            return coord
        queue = [coord]
        visited = set()
        directions = [(dx, dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1] if not (dx == 0 and dy == 0)]
        while queue:
            current = queue.pop(0)
            if current in visited or not self.is_within_bounds_2d(current):
                continue
            if self.is_traversable_2d(current):
                return current
            visited.add(current)
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor not in visited:
                    queue.append(neighbor)
        return None

    def a_star_2d(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        if not self.map_received or self.occupancy_grid_2d is None or self.density_map_2d is None:
            self.get_logger().warn("No occupancy or density map available")
            return None
        if not (self.is_within_bounds_2d(start) and self.is_within_bounds_2d(goal)):
            self.get_logger().warn(f"Start {start} or goal {goal} out of bounds")
            return None
        start_density = self.get_density_2d(start)
        goal_density = self.get_density_2d(goal)
        if not self.is_traversable_2d(start) or not self.is_traversable_2d(goal):
            self.get_logger().warn(f"Start {start} density: {start_density:.2f}, Goal {goal} density: {goal_density:.2f} - too high")
            return None
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal, start_density)}
        max_iterations = 100000
        iteration = 0
        while open_list and iteration < max_iterations:
            iteration += 1
            _, current = heapq.heappop(open_list)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            for i, j in neighbors:
                neighbor = (current[0] + i, current[1] + j)
                if not self.is_within_bounds_2d(neighbor) or not self.is_traversable_2d(neighbor):
                    continue
                neighbor_density = self.get_density_2d(neighbor)
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                total_cost = tentative_g_score + self.heuristic(neighbor, goal, neighbor_density)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = total_cost
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        if iteration >= max_iterations:
            self.get_logger().warn("A* search exceeded maximum iterations")
        return None

    def adjust_z_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int, int]]:
        adjusted_path = []
        prev_z = None
        for i, (x, y) in enumerate(path):
            z_start = self.fixed_z if prev_z is None else prev_z
            z_found = None
            # Only check z >= fixed_z (no underground)
            for dz in range(0, 3):  # Check up to 2 voxels above fixed_z
                z = z_start + dz
                coord = (x, y, z)
                if (self.is_within_bounds_3d(coord) and self.is_traversable_3d(coord)):
                    if prev_z is None or abs(z - prev_z) * VOXEL_SIZE <= MAX_STEP_HEIGHT:
                        z_found = z
                        break
                    elif i > 0:
                        prev_x, prev_y = path[i-1]
                        xy_dist = self.distance((prev_x, prev_y), (x, y)) * VOXEL_SIZE
                        slope = abs(z - prev_z) * VOXEL_SIZE / xy_dist if xy_dist > 0 else float('inf')
                        if slope <= MAX_SLOPE:
                            z_found = z
                            break
            if z_found is None:
                self.get_logger().warn(f"No traversable z found at ({x}, {y}) above ground; path invalid due to obstacle")
                return None
            adjusted_path.append((x, y, z_found))
            prev_z = z_found
        return adjusted_path

    def plan_path_callback(self, request: PlanPath.Request, response: PlanPath.Response) -> PlanPath.Response:
        self.get_logger().info(f"Planning path from {request.start.pose.position} to {request.goal.pose.position}")
        if not self.map_received or self.occupancy_grid_2d is None:
            self.get_logger().warn("No map data available - waiting for point cloud")
            response.plan = []
            return response
        try:
            start_coords = self.grid_coordinates(request.start.pose.position.x, request.start.pose.position.y, request.start.pose.position.z)
            goal_coords = self.grid_coordinates(request.goal.pose.position.x, request.goal.pose.position.y, request.goal.pose.position.z)
            
            adjusted_start = self.find_nearest_traversable(start_coords)
            if adjusted_start is None:
                self.get_logger().warn("No traversable point found near start")
                response.plan = []
                return response
            elif adjusted_start != start_coords:
                self.get_logger().info(f"Start adjusted from {start_coords} to {adjusted_start}")
                start_coords = adjusted_start

            adjusted_goal = self.find_nearest_traversable(goal_coords)
            if adjusted_goal is None:
                self.get_logger().warn("No traversable point found near goal")
                response.plan = []
                return response
            elif adjusted_goal != goal_coords:
                self.get_logger().info(f"Goal adjusted from {goal_coords} to {adjusted_goal}")
                goal_coords = adjusted_goal

            self.get_logger().info(f"Start grid coords: {start_coords}")
            self.get_logger().info(f"Goal grid coords: {goal_coords}")
            self.get_logger().info(f"Map bounds in world coords: min {self.min_bound}, max {self.max_bound}")
            path_2d = self.a_star_2d(start_coords, goal_coords)
            if path_2d:
                path_3d = self.adjust_z_path(path_2d)
                if path_3d is None:
                    response.plan = []
                    self.get_logger().warn("Path invalid due to obstacles in z adjustment")
                else:
                    response.plan = self.convert_path_to_pose_stamped(path_3d)
                    response.plan[0] = request.start
                    response.plan[-1] = request.goal
                    self.get_logger().info(f"Path found with {len(path_3d)} waypoints")
            else:
                response.plan = []
                self.get_logger().warn("No path found - check if start/goal are within map bounds and obstacle-free")
        except ValueError as e:
            self.get_logger().error(f"Path planning failed: {e}")
            response.plan = []
        return response

    def convert_path_to_pose_stamped(self, path: List[Tuple[int, int, int]]) -> List[PoseStamped]:
        poses = []
        for x, y, z in path:
            pose = PoseStamped()
            pose.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
            x_world = float(x * VOXEL_SIZE + self.min_bound[0])
            y_world = float(y * VOXEL_SIZE + self.min_bound[1])
            z_world = float(z * VOXEL_SIZE + self.min_bound[2])
            pose.pose.position = Point(x=x_world, y=y_world, z=z_world)
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