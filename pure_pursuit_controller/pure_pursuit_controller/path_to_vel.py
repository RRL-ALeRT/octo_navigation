import rclpy
from rclpy.node import Node
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

IS_SIMULATION = False

def yaw_from_quaternion(x, y, z, w):
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/pp_vel', 10)
        self.path_sub = self.create_subscription(Path, '/move_base_flex/path', self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.pure_pursuit_control)

        self.robot_position = [0.0, 0.0]
        self.robot_yaw = 0.0
        self.robot_foot_location = [0.0, 0.0, 0.0]
        self.path = []
        self.lookahead_distance = 0.3  # meters
        self.goal_tolerance = 0.5  # meters

    def path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def get_robot_pose(self):
        try:
            base_frame = "base_footprint" if IS_SIMULATION else "gpe"
            transform = self.tf_buffer.lookup_transform("map", base_frame, rclpy.time.Time())
            self.robot_foot_location = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]

            # Extract the robot's position and orientation in the "map" frame
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            self.robot_yaw = yaw_from_quaternion(transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z,
                                                transform.transform.rotation.w)
            self.robot_position = [x, y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            pass

    def find_lookahead_point(self):
        if not self.path:
            return None

        for point in self.path:
            dist = math.sqrt((point[0] - self.robot_position[0])**2 + (point[1] - self.robot_position[1])**2)
            if dist >= self.lookahead_distance:
                return point
        return self.path[-1] if self.path else None

    def goal_reached(self):
        if not self.path:
            return False
        goal_x, goal_y = self.path[-1]
        dist_to_goal = math.sqrt((goal_x - self.robot_position[0])**2 + (goal_y - self.robot_position[1])**2)
        return dist_to_goal < self.goal_tolerance

    def pure_pursuit_control(self):
        self.get_robot_pose()

        if self.goal_reached():
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info("Goal reached!")
            return

        lookahead = self.find_lookahead_point()
        if not lookahead:
            return

        lx, ly = lookahead
        dx = lx - self.robot_position[0]
        dy = ly - self.robot_position[1]

        angle_to_target = math.atan2(dy, dx)
        steering_angle = angle_to_target - self.robot_yaw
        steering_angle = math.atan2(math.sin(steering_angle), math.cos(steering_angle))

        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5  # Constant forward velocity
        cmd_msg.angular.z = 2.0 * steering_angle  # Proportional control

        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
