import time
import io
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage

import tqdm

import numpy as np
from PIL import Image as pil


def raw(msg):
    print('Heard talking:', msg)




class StringSubscriber(Node):
    def __init__(self, topic, throttle=None, depth=1):
        super().__init__('string_subscriber')
        self.subscription = self.create_subscription(String, topic, self.callback, depth)
        self.throttle = throttle

    def callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        if self.throttle:
            time.sleep(self.throttle)


class NumberSubscriber(Node):
    def __init__(self, topic, throttle=None, depth=1):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(Int32, topic, self.callback, depth)
        self.throttle = throttle

    def callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        if self.throttle:
            time.sleep(self.throttle)


class CompressedVideoSubscriber(Node):
    def __init__(self, topic, throttle=None, depth=1):
        super().__init__('compressed_video_subscriber')
        self.subscription = self.create_subscription(CompressedImage, topic, self.callback, depth)
        self.throttle = throttle
        self.i = 0
        self.winname = f'blah-{time.time()}'

    def callback(self, msg):
        # Convert the compressed image message to OpenCV image format
        im = np.array(pil.open(io.BytesIO(msg.data)))[:,:,::-1]
        cv2.imshow(self.winname, im)
        cv2.waitKey(1)
        self.get_logger().info(f'Received: "{im.shape}" {self.i}')
        self.i += 1
        if self.throttle:
            time.sleep(self.throttle)




TOPICS = {
    'helloworld': StringSubscriber,
    'count': NumberSubscriber,
    'cam': CompressedVideoSubscriber,
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


if __name__ == '__main__':
    import fire
    fire.Fire(main)