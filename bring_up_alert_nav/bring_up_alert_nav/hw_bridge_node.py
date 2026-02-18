#!/usr/bin/env python3
import rclpy, serial, threading
from rclpy.node   import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

PORT = '/dev/ttyUSB0'
BAUD = 115200

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

class HWBridge(Node):
    def __init__(self):
        super().__init__('hw_bridge')

        # ---- serial ----
        try:
            self.port = serial.Serial(PORT, BAUD, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {PORT}: {e}')
            raise SystemExit(1)
        self.get_logger().info(f'Serial open on {self.port.name}')

        # ---- ROS → Arduino ----
        self.servo_sub  = self.create_subscription(
            Float32, 'servo/angle', self.cb_servo_angle, 10)
        self.cmdvel_sub = self.create_subscription(
            Twist,   'cmd_vel',     self.cb_cmd_vel,    10)
        self.led_srv    = self.create_service(
            SetBool, 'led_strip/set', self.cb_led)

        # ---- Arduino → ROS ----
        self.mag_pub = self.create_publisher(Bool, 'victim/magnet', 10)

        # ---- background reader ----
        threading.Thread(target=self.serial_reader, daemon=True).start()

        self.current_angle = 90  # Start at midpoint

    # ---------- callbacks (PC → Arduino) ----------
    def cb_servo_angle(self, msg: Float32):
        angle = int(clamp(msg.data, 0, 180))
        self.current_angle = angle
        self._write(f"S:{angle}\n")
        self.get_logger().info(f'Servo (topic): {angle}°')

    def cb_cmd_vel(self, msg: Twist):
        # Teleop: increment/decrement angle based on angular.z
        step = 2  # degrees per command, adjust for sensitivity
        if msg.angular.z > 0.1:
            self.current_angle = clamp(self.current_angle + step, 0, 180)
        elif msg.angular.z < -0.1:
            self.current_angle = clamp(self.current_angle - step, 0, 180)
        # Send updated angle to Arduino
        self._write(f"S:{int(self.current_angle)}\n")
        self.get_logger().debug(f'Servo (teleop): {self.current_angle}°')

    def cb_led(self, req, resp):
        self._write(f"L:{1 if req.data else 0}\n")
        resp.success = True
        self.get_logger().info(f'LED strip set to {"ON" if req.data else "OFF"}')
        return resp

    # ---------- serial reader (Arduino → ROS) ----------
    def serial_reader(self):
        while rclpy.ok():
            try:
                line = self.port.readline().decode(errors='ignore').strip()
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                continue
            if not line:
                continue

            if line.startswith('M:'):
                mag = bool(int(line[2:]))
                self.mag_pub.publish(Bool(data=mag))
                self.get_logger().info(f'Magnet: {mag}')
            elif line.startswith('S:'):
                currentAngle = int(float(line[2:]))
                self.get_logger().info(f'Current Angle: {currentAngle}')
                # self.get_logger().error(f'Servo{line}')
            # else: ignore unknown prefixes for now

    # ---------- low-level write ----------
    def _write(self, line: str):
        try:
            self.port.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Write failed: {e}')

def main():
    rclpy.init()
    rclpy.spin(HWBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
