# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# class MyNode(Node):
#     def __init__(self):
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#             history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
#             depth=1
#         )

#         sub = Subscriber(
#             self,
#             Image,
#             "rgb_img",
#             qos_profile=qos_profile
#         )
#         sub.registerCallback(self._on_rgb)

#     def _on_rgb(self, msg):
#         ...

import io
import cv2
import time
import base64

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage

# import tqdm
from PIL import Image as pil


class StringPublisher(Node):
    def __init__(self, topic, text='Hellooooooo', interval=0.5, depth=100):
        super().__init__('string_publisher')
        self.text = text
        self.publisher_ = self.create_publisher(String, topic, depth)
        self.timer = self.create_timer(interval, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.text} {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class NumberPublisher(Node):
    def __init__(self, topic, interval=0.5, depth=100):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int32, topic, depth)
        self.timer = self.create_timer(interval, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class CompressedVideoPublisher(Node):
    def __init__(self, topic, src=0, img_format='jpeg', fps=30, depth=100):
        super().__init__('compressed_video_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, topic, depth)
        self.timer_ = self.create_timer(1/fps, self.publish_frame)
        self.cap = cv2.VideoCapture(src)  # Open default camera
        self.i = 0

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        # Convert the frame to JPEG compressed ROS2 CompressedImage message
        _, jpeg_data = cv2.imencode('.jpg', frame)
        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = 'jpeg'
        img_msg.data = jpeg_data.tostring()
        self.publisher_.publish(img_msg)
        self.get_logger().info(f'Publishing {self.i}')
        self.i += 1

TOPICS = {
    'helloworld': StringPublisher,
    'count': NumberPublisher,
    'cam': CompressedVideoPublisher,
}

def main(topic, *a, args=None, **kw):
    rclpy.init(args=args)

    try:
        pub = TOPICS[topic](topic, *a, **kw)
        try:
            rclpy.spin(pub)
        finally:
            pub.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    import fire
    fire.Fire(main)