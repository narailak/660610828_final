import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from agv_interfaces.msg import HandControl
from agv_interfaces.msg import ObstacleInfo

import math


class MotionManager(Node):

    def __init__(self):

        super().__init__('motion_manager')

        # subscribe mediapipe
        self.hand_sub = self.create_subscription(
            HandControl,
            '/hand_control_state',
            self.hand_callback,
            10
        )

        # subscribe obstacle
        self.obs_sub = self.create_subscription(
            ObstacleInfo,
            '/obstacle_info',
            self.obstacle_callback,
            10
        )

        # publish robot command
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_command',
            10
        )

        self.gear = 'P'
        self.speed_percent = 0.0
        self.steering_angle = 0.0

        self.obstacle_warning = False
        self.obstacle_emergency = False
        self.obstacle_angle = 0.0

        self.get_logger().info("Motion Manager Started")

    def hand_callback(self, msg):

        self.gear = msg.gear
        self.speed_percent = msg.speed_percent
        self.steering_angle = msg.steering_angle

        self.update_motion()

    def obstacle_callback(self, msg):

        self.obstacle_warning = msg.warning
        self.obstacle_emergency = msg.emergency
        self.obstacle_angle = msg.angle

        self.update_motion()

    def update_motion(self):

        twist = Twist()

        # ----------------------------
        # PRIORITY 1 : OBSTACLE
        # ----------------------------
        if self.obstacle_emergency:

            escape_speed = 0.2

            angle_rad = math.radians(self.obstacle_angle)

            escape_angle = angle_rad + math.pi

            twist.linear.x = escape_speed * math.cos(escape_angle)
            twist.linear.y = escape_speed * math.sin(escape_angle)

            self.get_logger().warn("ESCAPING obstacle")

        # ----------------------------
        # PRIORITY 2 : HUMAN CONTROL
        # ----------------------------
        else:

            if self.gear == 'P':

                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:

                linear_speed = self.speed_percent / 100.0

                if self.gear == 'D':
                    twist.linear.x = linear_speed

                elif self.gear == 'R':
                    twist.linear.x = -linear_speed

                twist.angular.z = -self.steering_angle / 90.0

        self.cmd_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = MotionManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()