import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros
from std_srvs.srv import SetBool
import math

# Constants
SPEED = 0.4
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.3

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('path_topic', '/move_base_flex/path')
        self.declare_parameter('cmd_vel_topic', '/pp_vel')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')

        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.twist_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.path_subscriber = self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Run at 20 Hz for smoother control
        self.create_timer(0.05, self.path_follower_timer_callback)

        self.in_motion = False
        self.pursuit_index = 0
        self.current_path_world = None  # 2D path: list of [x, y]

        self.srv = self.create_service(SetBool, '/navigation/follow_path', self.set_in_motion_callback)

    def set_in_motion_callback(self, request, response):
        """Service callback to enable/disable path following."""
        self.in_motion = request.data
        response.success = True
        response.message = f"in_motion set to {self.in_motion}"
        return response

    def path_callback(self, msg):
        """Handle new path messages, projecting 3D path to 2D (x, y only)."""
        # Extract only x, y from the 3D path, ignoring z
        self.current_path_world = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.pursuit_index = 0
        self.in_motion = True
        self.get_logger().info(f"Received new path with {len(msg.poses)} poses; projected to 2D (x, y)")

    def path_follower_timer_callback(self):
        """Timer callback to compute and publish velocity commands based on 2D path."""
        if not self.in_motion or self.current_path_world is None or len(self.current_path_world) == 0:
            self.stop_robot()
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            self.robot_yaw = self.yaw_from_quaternion(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"Could not get transform from {self.map_frame} to {self.robot_frame}: {ex}")
            self.stop_robot()
            return

        linear_velocity, angular_velocity, self.pursuit_index = self.pure_pursuit(
            self.x,
            self.y,
            self.robot_yaw,
            self.current_path_world,
            self.pursuit_index,
            SPEED,
            LOOKAHEAD_DISTANCE
        )

        # Check if the robot has reached the target (2D distance only)
        if math.hypot(self.x - self.current_path_world[-1][0], self.y - self.current_path_world[-1][1]) < TARGET_ERROR:
            self.get_logger().info("Target reached")
            self.stop_robot()
            self.in_motion = False
            self.current_path_world = None
            self.pursuit_index = 0
            return

        # Publish the twist commands
        twist_command = Twist()
        twist_command.linear.x = float(linear_velocity)
        twist_command.angular.z = float(angular_velocity)
        self.twist_publisher.publish(twist_command)

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        twist_command = Twist()
        twist_command.linear.x = 0.0
        twist_command.angular.z = 0.0
        self.twist_publisher.publish(twist_command)

    def yaw_from_quaternion(self, x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pure_pursuit(self, current_x, current_y, current_heading, path, index, speed, lookahead_distance):
        """Compute linear and angular velocities using Pure Pursuit algorithm in 2D."""
        # Find the lookahead point
        closest_point = None
        for i in range(index, len(path)):
            x, y = path[i]
            distance = math.hypot(current_x - x, current_y - y)
            if distance >= lookahead_distance:
                closest_point = (x, y)
                index = i
                break

        if closest_point is None:
            # Use the final point if no point is beyond lookahead distance
            closest_point = path[-1]
            index = len(path) - 1

        # Calculate the lookahead angle and difference (2D only)
        lookahead_angle = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        angle_diff = lookahead_angle - current_heading
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Decide direction based on angle difference
        forward = abs(angle_diff) < math.pi / 2
        v = speed if forward else -speed

        # Calculate desired steering angle
        if forward:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        else:
            target_heading = math.atan2(current_y - closest_point[1], current_x - closest_point[0])
        desired_steering_angle = target_heading - current_heading
        desired_steering_angle = (desired_steering_angle + math.pi) % (2 * math.pi) - math.pi  # Normalize

        # Limit steering angle and adjust velocity if sharp turn is needed
        if abs(desired_steering_angle) > math.pi / 6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = sign * math.pi / 4
            v = 0.0  # Stop linear motion for sharp turns

        return v, desired_steering_angle, index

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()