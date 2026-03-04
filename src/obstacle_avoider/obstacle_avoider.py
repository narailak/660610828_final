import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pub_cmd = self.create_publisher(
            Twist, '/cmd_vel_command', 10)
            
        self.is_escaping = False
        self.get_logger().info('High-Speed Obstacle Avoider Started...')

    def scan_callback(self, msg):
        min_dist = float('inf')
        min_angle_rad = 0.0

        for i, r in enumerate(msg.ranges):
            if 0.02 < r < msg.range_max:
                if r < min_dist:
                    min_dist = r
                    min_angle_rad = msg.angle_min + i * msg.angle_increment

        if min_dist == float('inf'):
            return

        dist_mm = min_dist * 1000.0

        robot_angle_rad = min_angle_rad + math.pi
        robot_deg = math.degrees(robot_angle_rad)
        cw_deg = (-robot_deg) % 360.0

        twist = Twist()

        if dist_mm < 150.0:
            if dist_mm < 120.0:
                # --- โซนอันตราย (< 120 mm) ---
                error = 120.0 - dist_mm
                
                # 🚀 จุดที่ปรับความเร็ว: เพิ่ม Kp และ Max Speed
                kp = 0.008  # ลองปรับค่านี้ดูได้ครับ (เช่น 0.005 - 0.01)
                max_speed = 0.6  # ความเร็วสูงสุด (m/s)
                
                escape_speed = min(error * kp, max_speed)

                escape_angle_rad = robot_angle_rad + math.pi
                
                twist.linear.x = escape_speed * math.cos(escape_angle_rad)
                twist.linear.y = escape_speed * math.sin(escape_angle_rad)
                
                self.pub_cmd.publish(twist)
                self.is_escaping = True
                
                self.get_logger().warn(
                    f'🔴 ESCAPING | Angle: {cw_deg:.1f}°, Dist: {dist_mm:.1f} mm, Speed: {escape_speed:.3f}'
                )

            else:
                # --- โซนเฝ้าระวัง (120 - 150 mm) ---
                if self.is_escaping:
                    self.pub_cmd.publish(Twist())
                    self.is_escaping = False
                    
                self.get_logger().info(
                    f'🟡 WARNING | Object detected at {cw_deg:.1f}°, Dist: {dist_mm:.1f} mm'
                )
        else:
            # --- โซนปลอดภัย (> 150 mm) ---
            if self.is_escaping:
                self.pub_cmd.publish(Twist())
                self.is_escaping = False

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub_cmd.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
