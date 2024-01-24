import io
import cv2
import base64

import tqdm

import numpy as np
from PIL import Image as pil
import roslibpy


def raw(msg):
    print('Heard talking:', msg)

def receive_image(msg):
    image_bytes = base64.b64decode(msg['data'].encode('ascii'))
    im = np.array(pil.open(io.BytesIO(image_bytes)))
    # Using cv2.imshow() method
    # Displaying the image
    cv2.imshow('blah', im)
    cv2.waitKey(1)
    

TOPICS = {
    '/helloworld': ['std_msgs/String', raw],
    '/cam': ['sensor_msgs/CompressedImage', receive_image]
}

def use_pbar(func):
    pbar = tqdm.tqdm()
    def callback(msg):
        pbar.update()
        return func(msg)
    return callback

def main(topic='/helloworld', throttle_rate=100):
    client = roslibpy.Ros(host='rosbridge', port=9090)
    client.run()
    print('connected?', client.is_connected)

    dtype, callback = TOPICS[topic]

    listener = roslibpy.Topic(client, topic, dtype, throttle_rate=throttle_rate, queue_length=1)
    listener.subscribe(use_pbar(callback))
    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()

if __name__ == '__main__':
    import fire
    fire.Fire(main)