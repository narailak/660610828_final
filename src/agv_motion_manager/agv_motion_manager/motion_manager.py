import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from agv_interfaces.msg import HandControl
from agv_interfaces.msg import ObstacleInfo
from agv_interfaces.srv import ControlMode

import math

class MotionManager(Node):

    def __init__(self):
        super().__init__('motion_manager')

        self.hand_sub = self.create_subscription(
            HandControl, '/hand_control_state', self.hand_callback, 10)
        self.obs_sub = self.create_subscription(
            ObstacleInfo, '/obstacle_info', self.obstacle_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_command', 10)
        self.mode_service = self.create_service(
            ControlMode, '/set_control_mode', self.service_callback)

        self.gear = 'P'
        self.speed_percent = 0.0
        self.steering_angle = 0.0

        self.obstacle_warning = False
        self.obstacle_emergency = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = 0.0

        self.control_mode = 0 
        self.get_logger().info("Motion Manager Started")

    def service_callback(self, request, response):
        self.control_mode = request.mode
        if self.control_mode == 0:
            msg = "MODE 0 : ROBOT STOP (ALL DISABLED)"
        elif self.control_mode == 1:
            msg = "MODE 1 : HAND CONTROL (OBSTACLE PRIORITY)"
        elif self.control_mode == 2:
            msg = "MODE 2 : OBSTACLE ONLY (HAND DISABLED)"
        else:
            msg = "UNKNOWN MODE"
            self.control_mode = 0 
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        self.update_motion()
        return response

    def hand_callback(self, msg):
        self.gear = msg.gear
        self.speed_percent = msg.speed_percent
        self.steering_angle = msg.steering_angle
        self.update_motion()

    def obstacle_callback(self, msg):
        self.obstacle_warning = msg.warning
        self.obstacle_emergency = msg.emergency
        self.obstacle_angle = msg.angle
        self.obstacle_distance = getattr(msg, 'distance', 0.0) 
        self.update_motion()

    def update_motion(self):
        twist = Twist()

        if self.control_mode == 0:
            self.cmd_pub.publish(twist)
            return

        # -------------------------------------------------------------
        # 🚨 PRIORITY 1: OBSTACLE ESCAPE (< 120mm)
        # -------------------------------------------------------------
        if self.obstacle_emergency:
            escape_speed = 0.3  
            
            obs_angle_ccw_rad = math.radians(-self.obstacle_angle)
            escape_angle = obs_angle_ccw_rad + math.pi

            twist.linear.x = escape_speed * math.cos(escape_angle)
            twist.linear.y = escape_speed * math.sin(escape_angle)
            twist.angular.z = 0.0

            self.get_logger().warn(
                f"ESCAPING! (Angle: {self.obstacle_angle:.1f} deg)"
            )

        # -------------------------------------------------------------
        # ✋ PRIORITY 2: HAND CONTROL (Warning & Safe Zones)
        # -------------------------------------------------------------
        elif self.control_mode == 1:
            if self.gear != 'P':
                # 🟢 เพิ่มการจำกัดความเร็วสูงสุด (Speed Limit) ป้องกันรถพุ่งแรงเกินไป
                MAX_LINEAR_SPEED = 0.2  
                MAX_ANGULAR_SPEED = 0.2   

                linear_speed = (self.speed_percent / 100.0) * MAX_LINEAR_SPEED
                
                # 🚦 ระบบตัดกำลังเมื่อคนฝืนสั่งให้ชนในระยะ Warning (120-150mm)
                is_moving_towards_obs = False
                
                if self.obstacle_warning:
                    # เช็คว่าสิ่งกีดขวางอยู่ด้านหน้า หรือ ด้านหลัง
                    is_front_obs = (self.obstacle_angle <= 90 or self.obstacle_angle >= 270)
                    is_back_obs = (90 < self.obstacle_angle < 270)
                    
                    if self.gear == 'D' and is_front_obs:
                        is_moving_towards_obs = True
                    elif self.gear == 'R' and is_back_obs:
                        is_moving_towards_obs = True

                if is_moving_towards_obs:
                    # ตัดกำลังการขับเคลื่อนเป็น 0 ป้องกันการพุ่งชน
                    twist.linear.x = 0.0
                else:
                    if self.gear == 'D':
                        twist.linear.x = linear_speed
                    elif self.gear == 'R':
                        twist.linear.x = -linear_speed

                # การหมุนซ้าย/ขวา
                twist.angular.z = (-self.steering_angle / 90.0) * MAX_ANGULAR_SPEED

        # -------------------------------------------------------------
        # 🤖 PRIORITY 3: AUTO / OBSTACLE ONLY
        # -------------------------------------------------------------
        elif self.control_mode == 2:
            # 🟢 แก้ไข: ในโหมด 2 ไม่ต้องส่งความเร็วอะไรให้เดินหน้า 
            # หุ่นจะอยู่นิ่งๆ และรอให้ Priority 1 (Emergency) ทำงานเมื่อมีของเข้ามาใกล้เท่านั้น
            pass

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()