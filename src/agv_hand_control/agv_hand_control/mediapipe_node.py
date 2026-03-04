import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # [เพิ่ม] นำเข้า Twist message สำหรับ cmd_vel
import cv2
import numpy as np
import os
import time
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

BaseOptions = python.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode


class MediaPipeNode(Node):

    def __init__(self):
        super().__init__('mediapipe_node')

        self.publisher_ = self.create_publisher(String, 'hand_tracking', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_command', 10) # [เพิ่ม] สร้าง Publisher สำหรับ cmd_vel
        self.timer = self.create_timer(0.033, self.timer_callback)

        model_path = os.path.join(
            os.path.dirname(__file__),
            "hand_landmarker.task"
        )

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=2
        )

        self.landmarker = HandLandmarker.create_from_options(options)

        self.cap = cv2.VideoCapture(0)
        self.window_w = 1280
        self.window_h = 720

        # ===== STATE =====
        self.current_gear = 'P'
        self.gear_selected = False
        self.current_speed_val = 0
        self.current_speed_text = "0%"
        self.steering_state = "STRAIGHT"
        self.steering_angle = 0

        # ===== UI CONFIG =====
        self.gear_x = 75
        self.speed_x = 1155
        self.bar_y_start = 200
        self.gear_points = {'R': 250, 'P': 400, 'D': 550}
        self.y_min, self.y_max = 250, 550

    # =======================================================
    # UI DRAW
    # =======================================================
    def draw_all_ui(self, img):

        # Zones
        cv2.line(img, (200, 0), (200, self.window_h), (180, 180, 180), 2)
        cv2.line(img, (1080, 0), (1080, self.window_h), (180, 180, 180), 2)

        # ===== GEAR =====
        gear_color = (0, 200, 0) if self.gear_selected else (0, 0, 0)
        cv2.putText(img, f"G: {self.current_gear}", (30, 100),
                    cv2.FONT_HERSHEY_TRIPLEX, 1.0, gear_color, 2)

        cv2.rectangle(img, (self.gear_x, self.bar_y_start),
                      (self.gear_x + 50, 600), (0, 0, 0), 2)

        cv2.circle(img,
                   (self.gear_x + 25,
                    self.gear_points[self.current_gear]),
                   22,
                   (0, 165, 255) if self.gear_selected else (0, 0, 0),
                   -1)

        # ===== STEERING =====
        str_display = self.steering_state
        str_color = (0, 0, 0)

        if self.current_gear == 'P':
            str_display = "LOCKED (P)"
            str_color = (0, 0, 255)
        elif self.steering_state == "LEFT":
            str_color = (255, 0, 0)
        elif self.steering_state == "RIGHT":
            str_color = (0, 0, 255)

        cv2.putText(img, f"STR: {str_display}", (450, 100),
                    cv2.FONT_HERSHEY_TRIPLEX, 1.2, str_color, 2)

        cv2.circle(img, (640, 400), 100, (200, 200, 200), 5)

        angle_to_draw = 0 if self.current_gear == 'P' else self.steering_angle

        end_x = int(640 + 80 * np.sin(np.radians(angle_to_draw)))
        end_y = int(400 - 80 * np.cos(np.radians(angle_to_draw)))

        cv2.line(img, (640, 400), (end_x, end_y), str_color, 5)

        # ===== SPEED =====
        speed_text = "LOCKED" if self.current_gear == 'P' else self.current_speed_text

        cv2.putText(img, f"S: {speed_text}", (1090, 100),
                    cv2.FONT_HERSHEY_TRIPLEX, 0.7,
                    (0, 0, 255) if self.current_gear == 'P' else (0, 0, 0), 2)

        cv2.rectangle(img, (self.speed_x, self.bar_y_start),
                      (self.speed_x + 50, 600), (0, 0, 0), 2)

        circle_speed_y = int(
            self.y_max - (self.current_speed_val *
                          (self.y_max - self.y_min) / 100)
        )

        cv2.circle(img, (self.speed_x + 25, circle_speed_y),
                   22, (0, 0, 0), -1)

    # =======================================================
    # MAIN LOOP
    # =======================================================
    def timer_callback(self):

        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.resize(frame, (self.window_w, self.window_h))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb
        )

        result = self.landmarker.detect_for_video(
            mp_image,
            int(time.time() * 1000)
        )

        frame = cv2.flip(frame, 1)

        left_hand_present = False
        thumb_positions = {}

        if result.hand_landmarks:

            for i, hand in enumerate(result.hand_landmarks):

                handedness = result.handedness[i][0].category_name

                wrist_x = (1 - hand[0].x) * self.window_w

                # ===== GEAR ZONE =====
                if wrist_x < 200:
                    left_hand_present = True

                    tips_y = [hand[idx].y for idx in [8, 12, 16, 20]]
                    avg_f_y = sum(tips_y) / 4 * self.window_h

                    if (max(tips_y) - min(tips_y)) < 0.08:

                        if abs(avg_f_y -
                               self.gear_points[self.current_gear]) < 50:
                            self.gear_selected = True

                        if self.gear_selected:
                            for g, y in self.gear_points.items():
                                if abs(avg_f_y - y) < 60:
                                    self.current_gear = g
                    else:
                        self.gear_selected = False

                # ===== CENTER ZONE =====
                if 200 <= wrist_x <= 1080:

                    thumb_y = hand[4].y
                    pinky_y = hand[20].y

                    if thumb_y < pinky_y:
                        thumb_positions[handedness] = thumb_y * self.window_h

            # ===== STEERING + SPEED =====
            if self.current_gear == 'P':
                self.steering_state = "STRAIGHT"
                self.steering_angle = 0
                self.current_speed_val = 0
                self.current_speed_text = "0%"

            elif len(thumb_positions) == 2:

                left = thumb_positions.get("Left")
                right = thumb_positions.get("Right")

                if left and right:

                    diff = right - left
                    self.steering_angle = np.clip(diff * 0.5, -90, 90)

                    if self.steering_angle > 15:
                        self.steering_state = "RIGHT"
                    elif self.steering_angle < -15:
                        self.steering_state = "LEFT"
                    else:
                        self.steering_state = "STRAIGHT"

                    avg_y = (left + right) / 2
                    raw_spd = ((self.y_max - avg_y) /
                               (self.y_max - self.y_min)) * 100

                    self.current_speed_val = np.clip(raw_spd, 0, 100)
                    self.current_speed_text = f"{int(self.current_speed_val)}%"

            else:
                self.steering_state = "STRAIGHT"
                self.steering_angle = 0
                self.current_speed_val = 0
                self.current_speed_text = "0%"

        if not left_hand_present:
            self.gear_selected = False

        self.draw_all_ui(frame)

        # =======================================================
        # [เพิ่ม] CMD_VEL MAPPING & PUBLISH
        # =======================================================
        twist_msg = Twist()
        
        if self.current_gear == 'P':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            # แมพสเกลความเร็ว 0-100 ให้เป็น 0.0 - 1.0 (สมมติให้ max speed = 1.0 m/s ปรับคูณเพิ่มได้ตามสเปคหุ่นจริง)
            linear_speed = float(self.current_speed_val) / 100.0
            
            # เช็คเกียร์ D และ R
            if self.current_gear == 'D':
                twist_msg.linear.x = linear_speed
            elif self.current_gear == 'R':
                twist_msg.linear.x = -linear_speed
                
            # แมพมุมเลี้ยว (-90 ถึง 90) เป็น angular z
            # มาตรฐาน ROS: เลี้ยวซ้าย = + (บวก), เลี้ยวขวา = - (ลบ)
            # ในโค้ดเดิม steering_angle ถ้าเป็นบวกคือ RIGHT ดังนั้นต้องใส่เครื่องหมายลบ (-) กลับทิศให้ตรงตามมาตรฐาน ROS
            twist_msg.angular.z = -float(self.steering_angle) / 90.0

        self.cmd_vel_pub.publish(twist_msg)
        # =======================================================

        cv2.imshow("AGV Safety Control Master", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()