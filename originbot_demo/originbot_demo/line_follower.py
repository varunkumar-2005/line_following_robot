import time

import cv2
import cv_bridge
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class LineFollower(Node):
    def __init__(self):
        super().__init__('follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        self.bridge = cv_bridge.CvBridge()
        self.last_angular_z = 0.0
        self.last_error = 0.0
        self.filtered_error = 0.0
        self.missed_frames = 0
        self.stop_until = 0.0
        self.active_checkpoint = None
        self.last_checkpoint_time = 0.0
        self.checkpoint_hold_seconds = 2.5
        self.checkpoint_clear_frames = 0
        self.checkpoint_release_frames = 10
        self.resume_until = 0.0
        self.resume_heading_until = 0.0
        self.kernel = np.ones((5, 5), np.uint8)

        self.track_ranges = [
            (np.array([0, 110, 80]), np.array([10, 255, 255])),
            (np.array([170, 110, 80]), np.array([180, 255, 255])),
        ]

        self.checkpoint_specs = [
            ('loading point', (np.array([20, 140, 120]), np.array([35, 255, 255]))),
            ('drop point', (np.array([45, 90, 90]), np.array([75, 255, 255]))),
            ('warehouse stop', (np.array([95, 120, 120]), np.array([130, 255, 255]))),
            ('recharge stop', (np.array([135, 80, 80]), np.array([165, 255, 255]))),
        ]

    def build_track_mask(self, hsv_image):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in self.track_ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))

        mask = cv2.GaussianBlur(mask, (9, 9), 0)
        _, mask = cv2.threshold(mask, 80, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return mask

    def detect_checkpoint(self, hsv_image):
        h, w, _ = hsv_image.shape
        zone_top = int(0.68 * h)
        zone_bottom = int(0.94 * h)
        zone_left = int(0.05 * w)
        zone_right = int(0.95 * w)
        checkpoint_roi = hsv_image[zone_top:zone_bottom, zone_left:zone_right]

        best_name = None
        best_area = 0
        for name, (lower, upper) in self.checkpoint_specs:
            mask = cv2.inRange(checkpoint_roi, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 120 or area > 5000:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                if width < 10 or height < 10:
                    continue
                if area > best_area:
                    best_area = area
                    best_name = name

        if best_name is not None:
            return best_name
        return None

    def handle_checkpoint(self, hsv_image):
        now = time.monotonic()
        checkpoint = self.detect_checkpoint(hsv_image)

        if checkpoint is None:
            self.checkpoint_clear_frames += 1
            if self.checkpoint_clear_frames >= self.checkpoint_release_frames:
                self.active_checkpoint = None
            return

        self.checkpoint_clear_frames = 0

        if checkpoint == self.active_checkpoint:
            return

        if now - self.last_checkpoint_time < 3.0:
            return

        self.active_checkpoint = checkpoint
        self.last_checkpoint_time = now
        self.stop_until = now + self.checkpoint_hold_seconds
        self.resume_until = self.stop_until + 1.2
        self.resume_heading_until = self.stop_until + 0.7
        self.filtered_error = 0.0
        self.last_error = 0.0
        self.get_logger().info(f'Checkpoint reached: {checkpoint}')

    def image_callback(self, msg):
        image_input = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image_input, cv2.COLOR_BGR2HSV)

        self.handle_checkpoint(hsv)

        twist = Twist()
        now = time.monotonic()
        if now < self.stop_until:
            self.publisher.publish(twist)
            cv2.imshow("Check cache", image_input)
            cv2.waitKey(3)
            return

        mask = self.build_track_mask(hsv)
        h, w, _ = image_input.shape

        near_top = int(0.68 * h)
        near_bottom = int(0.93 * h)
        far_top = int(0.52 * h)
        far_bottom = int(0.68 * h)

        near_mask = np.zeros_like(mask)
        far_mask = np.zeros_like(mask)
        near_mask[near_top:near_bottom, :] = mask[near_top:near_bottom, :]
        far_mask[far_top:far_bottom, :] = mask[far_top:far_bottom, :]

        near_moments = cv2.moments(near_mask)
        far_moments = cv2.moments(far_mask)

        if near_moments['m00'] > 0:
            self.missed_frames = 0

            near_cx = near_moments['m10'] / near_moments['m00']
            if far_moments['m00'] > 0:
                far_cx = far_moments['m10'] / far_moments['m00']
                target_cx = 0.65 * near_cx + 0.35 * far_cx
            else:
                target_cx = near_cx

            error = target_cx - (w / 2.0)
            self.filtered_error = 0.7 * self.filtered_error + 0.3 * error
            derivative = self.filtered_error - self.last_error

            angular = -(0.0016 * self.filtered_error + 0.0022 * derivative)
            angular = float(np.clip(angular, -0.25, 0.25))

            abs_error = abs(self.filtered_error)
            if abs_error < 35:
                linear = 0.17
            elif abs_error < 85:
                linear = 0.14
            else:
                linear = 0.11

            if now < self.resume_until:
                linear = max(linear, 0.15)

            if abs_error < 14:
                angular *= 0.45
            elif abs_error < 28:
                angular *= 0.70

            if now < self.resume_heading_until:
                angular = float(np.clip(angular, -0.14, 0.14))

            twist.linear.x = linear
            twist.angular.z = 0.75 * self.last_angular_z + 0.25 * angular

            self.last_angular_z = twist.angular.z
            self.last_error = self.filtered_error

            cv2.circle(image_input, (int(target_cx), int((near_top + near_bottom) / 2)), 14, (0, 255, 0), -1)
            print(f"Following line. Linear: {twist.linear.x:.3f}, Angular: {twist.angular.z:.3f}")
        else:
            self.missed_frames += 1
            if now < self.resume_until:
                twist.linear.x = 0.12
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.04
                recovery_turn = self.last_angular_z * 0.60
                if abs(recovery_turn) < 0.08:
                    recovery_turn = 0.08 if self.last_angular_z >= 0.0 else -0.08
                if self.missed_frames > 18:
                    recovery_turn = 0.12 if self.last_angular_z >= 0.0 else -0.12
                    twist.linear.x = 0.03
                twist.angular.z = recovery_turn
            print("No line detected. Searching for line.")

        self.publisher.publish(twist)
        cv2.imshow("Check cache", image_input)
        cv2.waitKey(3)

    def stop_robot(self):
        self.get_logger().info("Shutting down: Stopping robot")
        stop_twist = Twist()
        for _ in range(5):
            self.publisher.publish(stop_twist)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Stopping robot.")
    finally:
        node.stop_robot()
        time.sleep(1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
