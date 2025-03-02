import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
import tf2_ros
import math
import numpy as np

# Constants
SPEED = 0.4
LOOKAHEAD_DISTANCE = 0.4
ROBOT_RADIUS = 0.3
TARGET_ERROR = 0.3
OBSTACLE_DISTANCE = 1.0

class PurePursuitWithPCL2(Node):
    def __init__(self):
        super().__init__('pure_pursuit_pcl2')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.declare_parameter('path_topic', '/move_base_flex/path')
        self.declare_parameter('cmd_vel_topic', '/pp_vel')
        self.declare_parameter('pcl_topic', 'Spot/Velodyne_Puck/point_cloud')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')

        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.pcl_topic = self.get_parameter('pcl_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.twist_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 1)

        self.path_subscriber = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.pcl_subscriber = self.create_subscription(PointCloud2, self.pcl_topic, self.pcl_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.path_follower_timer_callback)

        self.srv = self.create_service(SetBool, '/navigation/follow_path', self.set_in_motion_callback)

        self.in_motion = False
        self.current_path_world = []
        self.pursuit_index = 0

        self.obstacle_detected = False

    def set_in_motion_callback(self, request, response):
        self.in_motion = request.data
        response.success = True
        response.message = f"in_motion set to {self.in_motion}"
        return response

    def path_callback(self, msg):
        self.current_path_world = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.pursuit_index = 0
        self.in_motion = True
        self.get_logger().info(f"Received new path with {len(msg.poses)} poses")

    def pcl_callback(self, msg):
        self.obstacle_detected = self.detect_obstacle(msg)

    def detect_obstacle(self, cloud, z_offset=0.5):
        points = np.array([[float(p[0]), float(p[1]), float(p[2])] 
                            for p in point_cloud2.read_points(cloud, skip_nans=True)])
        
        for point in points:
            x, y, z = point[:3]
            z += z_offset  # Add the z-offset to avoid considering the floor as an obstacle
            distance = math.sqrt(x ** 2 + y ** 2)

            if distance < 0.05: # Ignore points too close to the robot
                continue

            # Log the point being processed
            self.get_logger().debug(f"Point: x={x:.2f}, y={y:.2f}, z={z:.2f}, distance={distance:.2f}")

            # Check if the obstacle is above 0.2 meters from the ground
            if z > 0.1 and (-ROBOT_RADIUS < y < OBSTACLE_DISTANCE or -ROBOT_RADIUS < x < OBSTACLE_DISTANCE):
                self.get_logger().warn(f"Obstacle detected at {x:.2f}m, {y:.2f}m, {z:.2f}m")
                return True

        return False

    def path_follower_timer_callback(self):
        if not self.in_motion or len(self.current_path_world) == 0:
            self.publish_stop()
            return

        if self.obstacle_detected:
            self.get_logger().warn("Stopping due to obstacle!")
            self.publish_stop()
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_yaw = self.yaw_from_quaternion(transform.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        linear_vel, angular_vel, self.pursuit_index = self.pure_pursuit(robot_x, robot_y, robot_yaw)

        if self.reached_goal(robot_x, robot_y):
            self.get_logger().info("Goal reached!")
            self.in_motion = False
            self.publish_stop()
            return

        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.twist_publisher.publish(twist)

    def pure_pursuit(self, robot_x, robot_y, robot_yaw):
        while self.pursuit_index < len(self.current_path_world):
            target_x, target_y = self.current_path_world[self.pursuit_index]
            distance = math.hypot(robot_x - target_x, robot_y - target_y)

            if distance > LOOKAHEAD_DISTANCE:
                break

            self.pursuit_index += 1

        if self.pursuit_index >= len(self.current_path_world):
            return 0.0, 0.0, self.pursuit_index

        target_x, target_y = self.current_path_world[self.pursuit_index]
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
        angle_diff = angle_to_target - robot_yaw

        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        return SPEED, angle_diff, self.pursuit_index

    def yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def reached_goal(self, x, y):
        goal_x, goal_y = self.current_path_world[-1]
        return math.hypot(x - goal_x, y - goal_y) < TARGET_ERROR

    def publish_stop(self):
        self.twist_publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitWithPCL2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
