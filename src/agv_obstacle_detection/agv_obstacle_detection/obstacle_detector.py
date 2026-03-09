import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from agv_interfaces.msg import ObstacleInfo

import math


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub_obs = self.create_publisher(
            ObstacleInfo,
            '/obstacle_info',
            10
        )

        self.get_logger().info("Obstacle Detector Started")

    def scan_callback(self, msg):
        min_dist = float('inf')
        min_angle = 0.0

        for i, r in enumerate(msg.ranges):
            if 0.02 < r < msg.range_max:
                if r < min_dist:
                    min_dist = r
                    min_angle = msg.angle_min + i * msg.angle_increment

        if min_dist == float('inf'):
            return

        raw_dist_mm = min_dist * 1000.0
        ROBOT_RADIUS_MM = 35.0

        dist_mm = max(0.0, raw_dist_mm - ROBOT_RADIUS_MM)

        # -----------------------------------------------------
        # ปรับแก้พิกัด LiDAR (บวก Pi เพราะ LiDAR หันหลัง)
        # -----------------------------------------------------
        robot_angle_rad = min_angle + math.pi
        
        # 🟢 แก้ไข: แปลงเป็นองศาตามโจทย์ (0 องศาด้านหน้า, นับตามเข็มนาฬิกา)
        robot_deg = math.degrees(robot_angle_rad)
        cw_deg = (-robot_deg) % 360.0

        msg_out = ObstacleInfo()
        msg_out.distance = float(dist_mm)
        msg_out.angle = float(cw_deg) # ส่งค่าตามเข็มนาฬิกาออกไป

        if dist_mm < 200:
            msg_out.warning = True
            msg_out.emergency = True
            self.get_logger().warn(f'🔴 EMERGENCY | Angle: {cw_deg:.1f}° | Dist: {dist_mm:.1f} mm')

        elif dist_mm < 280:
            msg_out.warning = True
            msg_out.emergency = False
            self.get_logger().info(f'🟡 WARNING | Angle: {cw_deg:.1f}° | Dist: {dist_mm:.1f} mm')

        else:
            msg_out.warning = False
            msg_out.emergency = False

        self.pub_obs.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()