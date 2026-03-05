import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage # <--- เปลี่ยนเป็น CompressedImage
from agv_interfaces.msg import HandControl 
from cv_bridge import CvBridge             

import cv2
import numpy as np
import os
import time
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ament_index_python.packages import get_package_share_directory

BaseOptions = python.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode


class MediaPipeNode(Node):

    def __init__(self):
        super().__init__('mediapipe_node')

        self.bridge = CvBridge()

        # ===== Publishers =====
        self.publisher_ = self.create_publisher(String, 'hand_tracking', 10)
        self.hand_pub = self.create_publisher(HandControl, '/hand_control_state', 10)
        
        # เปลี่ยน Topic และ Type เป็น CompressedImage
        self.image_pub = self.create_publisher(CompressedImage, '/camera_image/compressed', 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

        model_path = os.path.join(
            get_package_share_directory('agv_hand_control'),
            'resource',
            'hand_landmarker.task'
        )

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=2
        )

        self.landmarker = HandLandmarker.create_from_options(options)

        self.cap = cv2.VideoCapture(0)
        
        # [OPTIONAL] ถ้าอยากให้ลื่นสุดๆ สามารถปลดคอมเมนต์ 2 บรรทัดนี้เพื่อลดความละเอียดกล้องก่อนประมวลผลได้
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.window_w = 1280
        self.window_h = 720

        # ===== STATE =====
        self.current_gear = 'P'
        self.gear_selected = False
        self.current_speed_val = 0
        self.steering_state = "STRAIGHT"
        self.steering_angle = 0
        self.gear_points = {'R': 250, 'P': 400, 'D': 550}
        self.y_min, self.y_max = 250, 550

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.resize(frame, (self.window_w, self.window_h))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self.landmarker.detect_for_video(mp_image, int(time.time() * 1000))

        frame = cv2.flip(frame, 1)

        left_hand_present = False
        thumb_positions = {}

        if result.hand_landmarks:
            for i, hand in enumerate(result.hand_landmarks):
                handedness = result.handedness[i][0].category_name
                wrist_x = (1 - hand[0].x) * self.window_w

                if wrist_x < 200:
                    left_hand_present = True
                    tips_y = [hand[idx].y for idx in [8, 12, 16, 20]]
                    avg_f_y = sum(tips_y) / 4 * self.window_h

                    if (max(tips_y) - min(tips_y)) < 0.08:
                        if abs(avg_f_y - self.gear_points[self.current_gear]) < 50:
                            self.gear_selected = True

                        if self.gear_selected:
                            for g, y in self.gear_points.items():
                                if abs(avg_f_y - y) < 60:
                                    self.current_gear = g
                    else:
                        self.gear_selected = False

                if 200 <= wrist_x <= 1080:
                    thumb_y = hand[4].y
                    pinky_y = hand[20].y
                    if thumb_y < pinky_y:
                        thumb_positions[handedness] = thumb_y * self.window_h

            if self.current_gear == 'P':
                self.steering_state = "STRAIGHT"
                self.steering_angle = 0
                self.current_speed_val = 0
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
                    raw_spd = ((self.y_max - avg_y) / (self.y_max - self.y_min)) * 100
                    self.current_speed_val = np.clip(raw_spd, 0, 100)
            else:
                self.steering_state = "STRAIGHT"
                self.steering_angle = 0
                self.current_speed_val = 0

        if not left_hand_present:
            self.gear_selected = False

        hand_msg = HandControl()
        hand_msg.gear = self.current_gear
        hand_msg.speed_percent = float(self.current_speed_val)
        hand_msg.steering_angle = float(self.steering_angle)
        hand_msg.steering_state = self.steering_state
        self.hand_pub.publish(hand_msg)

        # =======================================================
        # PUBLISH COMPRESSED IMAGE
        # =======================================================
        # บีบอัดภาพเป็น JPEG คุณภาพ 70% ก่อนส่งเพื่อลดความหน่วง
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = encoded_image.tobytes()
            self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()