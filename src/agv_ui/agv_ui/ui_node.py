import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from agv_interfaces.msg import HandControl
from agv_interfaces.msg import ObstacleInfo

import cv2
import numpy as np


class DashboardUI(Node):

    def __init__(self):
        super().__init__('ui_node')

        self.bridge = CvBridge()

        # ===== Robot State =====
        self.speed = 0.0
        self.gear = "P"
        self.steering_angle = 0.0
        self.steering_state = "STRAIGHT"

        # ===== Obstacle State =====
        self.obstacle_dist = None
        self.obstacle_angle = None
        self.obstacle_warning = False
        self.obstacle_emergency = False

        self.last_frame = None

        # ===== UI CONFIG =====
        self.window_w = 1280
        self.window_h = 720
        self.gear_x = 75
        self.speed_x = 1155
        self.bar_top = 250
        self.bar_bottom = 550
        self.gear_points = {'R': 250, 'P': 400, 'D': 550}

        # ===== Subscribers =====
        self.create_subscription(
            CompressedImage,
            "/camera_image/compressed",
            self.image_callback,
            qos_profile_sensor_data 
        )

        self.create_subscription(
            HandControl,
            "/hand_control_state",
            self.hand_callback,
            10
        )

        self.create_subscription(
            ObstacleInfo,
            "/obstacle_info",
            self.obstacle_callback,
            10
        )

        self.timer = self.create_timer(0.033, self.update_ui)
        self.get_logger().info("Dashboard UI (Modern Gradient Edition) Started")

    def hand_callback(self, msg):
        self.speed = msg.speed_percent
        self.gear = msg.gear
        self.steering_angle = msg.steering_angle
        self.steering_state = msg.steering_state

    def obstacle_callback(self, msg):
        self.obstacle_dist = msg.distance
        self.obstacle_angle = msg.angle
        self.obstacle_warning = msg.warning
        self.obstacle_emergency = msg.emergency

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.last_frame = frame

    # ฟังก์ชันช่วยคำนวณการไล่สี (Gradient) ของความเร็ว 0-100%
    def get_speed_color(self, speed):
        s = max(0, min(speed, 100))
        if s <= 50:
            # 0-50% : เขียว -> เหลือง
            r = int((s / 50.0) * 255)
            g = 255
            b = 0
        else:
            # 51-100% : เหลือง -> แดง
            r = 255
            g = int((1.0 - (s - 50) / 50.0) * 255)
            b = 0
        return (b, g, r) # OpenCV ใช้ BGR

    def draw_dashboard(self, img):
        
        # =========================
        # สร้างแผง UI โปร่งแสง (Transparent Overlay)
        # =========================
        overlay = img.copy()
        
        # แผงซ้าย (เกียร์)
        cv2.rectangle(overlay, (0, 0), (200, self.window_h), (10, 10, 10), -1)
        # แผงขวา (ความเร็ว)
        cv2.rectangle(overlay, (1080, 0), (1280, self.window_h), (10, 10, 10), -1)
        # แผงด้านบน (บอกสถานะพวงมาลัย)
        cv2.rectangle(overlay, (460, 20), (820, 130), (10, 10, 10), -1)
        
        # นำแผงโปร่งแสงไปซ้อนกับภาพจริง (ความโปร่ง 60%)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

        # เส้นขอบ UI
        cv2.line(img, (200, 0), (200, self.window_h), (100, 100, 100), 2)
        cv2.line(img, (1080, 0), (1080, self.window_h), (100, 100, 100), 2)

        # =========================
        # GEAR BAR
        # =========================
        cv2.putText(img, "GEAR", (60, 80), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (200, 200, 200), 1)
        cv2.putText(img, f"{self.gear}", (80, 140), cv2.FONT_HERSHEY_TRIPLEX, 2.0, (0, 255, 0) if self.gear != 'P' else (0,0,255), 3)

        cv2.rectangle(img, (self.gear_x, self.bar_top), (self.gear_x + 50, self.bar_bottom), (40, 40, 40), -1)
        cv2.rectangle(img, (self.gear_x, self.bar_top), (self.gear_x + 50, self.bar_bottom), (150, 150, 150), 2)

        target_y = self.gear_points.get(self.gear, 400)
        # ไฮไลต์เกียร์ที่เลือก
        cv2.circle(img, (self.gear_x + 25, target_y), 30, (0, 165, 255), -1)
        
        # ตัวหนังสือเกียร์
        for g, y_pos in self.gear_points.items():
            color = (0, 0, 0) if g == self.gear else (150, 150, 150)
            cv2.putText(img, g, (self.gear_x + 13, y_pos + 12), cv2.FONT_HERSHEY_TRIPLEX, 1.0, color, 2)

        # =========================
        # STEERING WHEEL
        # =========================
        str_display = self.steering_state
        str_color = (0, 0, 0)

        if self.gear == 'P':
            str_display = "LOCKED"
            str_color = (0, 0, 255)
        elif self.steering_state == "LEFT":
            str_color = (255, 0, 0)
        elif self.steering_state == "RIGHT":
            str_color = (0, 0, 255)
        else:
            str_color = (0, 255, 0)

        cv2.putText(img, "STEERING", (575, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
        
        # จัดตำแหน่งข้อความให้ตรงกลาง
        text_size = cv2.getTextSize(str_display, cv2.FONT_HERSHEY_TRIPLEX, 1.2, 2)[0]
        text_x = 640 - (text_size[0] // 2)
        cv2.putText(img, str_display, (text_x, 105), cv2.FONT_HERSHEY_TRIPLEX, 1.2, str_color, 2)

        # วาดพวงมาลัยวงแหวน
        cv2.circle(img, (640, 400), 120, (50, 50, 50), 12) 
        cv2.circle(img, (640, 400), 120, str_color if self.gear != 'P' else (100,100,100), 3) 
        cv2.circle(img, (640, 400), 15, (200, 200, 200), -1)

        angle_to_draw = 0 if self.gear == 'P' else self.steering_angle
        end_x = int(640 + 120 * np.sin(np.radians(angle_to_draw)))
        end_y = int(400 - 120 * np.cos(np.radians(angle_to_draw)))
        cv2.line(img, (640, 400), (end_x, end_y), str_color if self.gear != 'P' else (100,100,100), 8)

        # =========================
        # SPEED BAR (GRADIENT COLOR)
        # =========================
        safe_speed = max(0, min(self.speed, 100))
        speed_color = self.get_speed_color(safe_speed) if self.gear != 'P' else (100, 100, 100)
        
        cv2.putText(img, "SPEED", (1140, 80), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (200, 200, 200), 1)
        speed_text = "LOCK" if self.gear == 'P' else f"{int(self.speed)}%"
        
        text_size_spd = cv2.getTextSize(speed_text, cv2.FONT_HERSHEY_TRIPLEX, 1.2, 2)[0]
        text_x_spd = 1180 - (text_size_spd[0] // 2)
        cv2.putText(img, speed_text, (text_x_spd, 140), cv2.FONT_HERSHEY_TRIPLEX, 1.2, (0, 0, 255) if self.gear == 'P' else speed_color, 2)

        # พื้นหลังของหลอดความเร็ว
        cv2.rectangle(img, (self.speed_x, self.bar_top), (self.speed_x + 50, self.bar_bottom), (40, 40, 40), -1)
        
        circle_speed_y = int(self.bar_bottom - (safe_speed * (self.bar_bottom - self.bar_top) / 100))

        # *** ไฮไลต์เด็ด: วาดแถบความเร็วแบบไล่สี (Gradient) ***
        if self.gear != 'P' and safe_speed > 0:
            for y in range(circle_speed_y, self.bar_bottom):
                # คำนวณความเร็ว ณ จุด Y นั้น เพื่อหาสีที่ถูกต้อง
                curr_s = ((self.bar_bottom - y) / (self.bar_bottom - self.bar_top)) * 100
                c = self.get_speed_color(curr_s)
                # ตีเส้นแนวนอนเรียงต่อกันเป็นหลอดสี
                cv2.line(img, (self.speed_x, y), (self.speed_x + 50, y), c, 1)

        # กรอบหลอดความเร็ว
        cv2.rectangle(img, (self.speed_x, self.bar_top), (self.speed_x + 50, self.bar_bottom), (150, 150, 150), 2)
        # ขีดบอกระดับ (Indicator)
        cv2.rectangle(img, (self.speed_x - 10, circle_speed_y - 5), (self.speed_x + 60, circle_speed_y + 5), (255, 255, 255), -1)

        # =========================
        # OBSTACLE WARNING
        # =========================
        if self.obstacle_dist is not None:
            if self.obstacle_emergency:
                color = (0, 0, 255)
                status_text = "EMERGENCY STOP"
            elif self.obstacle_warning:
                color = (0, 255, 255)
                status_text = "WARNING"
            else:
                color = (0, 255, 0)
                status_text = "CLEAR"

            if self.obstacle_warning or self.obstacle_emergency:
                # วาดกล่องเตือนโปร่งแสง
                obs_overlay = img.copy()
                cv2.rectangle(obs_overlay, (340, 620), (940, 700), (0, 0, 0), -1)
                cv2.addWeighted(obs_overlay, 0.8, img, 0.2, 0, img)
                
                # กรอบสีตามสถานะ
                cv2.rectangle(img, (340, 620), (940, 700), color, 3)

                text = f"[{status_text}] {self.obstacle_angle:.1f} DEG | {self.obstacle_dist:.0f} MM"
                obs_text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
                obs_text_x = 640 - (obs_text_size[0] // 2)
                cv2.putText(img, text, (obs_text_x, 670), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            else:
                text = f"SENSOR ACTIVE: Closest {self.obstacle_dist:.0f} mm"
                obs_text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                obs_text_x = 640 - (obs_text_size[0] // 2)
                cv2.putText(img, text, (obs_text_x, 680), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)

    def update_ui(self):
        if self.last_frame is None:
            frame = np.zeros((self.window_h, self.window_w, 3), dtype=np.uint8)
            cv2.putText(frame, "Waiting for Camera System...", (410, 360), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
        else:
            frame = cv2.resize(self.last_frame, (self.window_w, self.window_h))

        self.draw_dashboard(frame)
        cv2.imshow("AGV Control Dashboard", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()