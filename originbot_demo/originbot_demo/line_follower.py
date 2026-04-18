import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import cv_bridge
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = cv_bridge.CvBridge()
        self.last_angular_z = 0.0
        self.missed_frames = 0

    def image_callback(self, msg):
        print("Received image frame")  # Debugging
        image_input = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image_input, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, np.array([15, 70, 70]), np.array([45, 255, 255]))
        green_mask = cv2.inRange(hsv, np.array([40, 60, 60]), np.array([90, 255, 255]))
        blue_mask = cv2.inRange(hsv, np.array([95, 80, 60]), np.array([130, 255, 255]))
        red_mask_1 = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([10, 255, 255]))
        red_mask_2 = cv2.inRange(hsv, np.array([170, 90, 70]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(yellow_mask, green_mask)
        mask = cv2.bitwise_or(mask, blue_mask)
        mask = cv2.bitwise_or(mask, red_mask_1)
        mask = cv2.bitwise_or(mask, red_mask_2)
        
        h, w, _ = image_input.shape
        search_top = int(0.60 * h)
        search_bot = int(0.95 * h)
        mask[:search_top, :] = 0  # Mask out upper part
        mask[search_bot:, :] = 0  # Mask out lower part

        M = cv2.moments(mask)
        twist = Twist()

        if M['m00'] > 0:  # If a yellow line is detected
            self.missed_frames = 0
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image_input, (cx, cy), 20, (0, 0, 255), -1)

            err = cx - w / 2
            twist.linear.x = 0.12
            twist.angular.z = max(min(-float(err) / 600, 0.45), -0.45)
            self.last_angular_z = twist.angular.z
            print(f"Following line. Linear: {twist.linear.x}, Angular: {twist.angular.z}") 

        else:  # If the line is briefly lost, keep searching in the last known direction
            self.missed_frames += 1
            print("No line detected. Searching for line.")
            twist.linear.x = 0.06
            if self.missed_frames < 10:
                twist.angular.z = self.last_angular_z * 0.5
            else:
                fallback_turn = 0.2 if self.last_angular_z >= 0.0 else -0.2
                twist.angular.z = fallback_turn

        # Publish movement command
        self.publisher.publish(twist)
        s = str(M['m00'])
        # Debugging
        cv2.imshow("Check cache", image_input)
        cv2.waitKey(3)

    def stop_robot(self):
        """Stops the robot when the node is destroyed."""
        print("Shutting down: Stopping the robot")
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        for _ in range(5):  
            self.publisher.publish(stop_twist)
            time.sleep(0.1)

def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Stopping robot.")
    finally:
        node.stop_robot()
        time.sleep(1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
