import io
import cv2
import time
import base64

import tqdm

import roslibpy
from PIL import Image as pil


def generate_string(client, topic, text='Hello World!', interval=1):
    try:
        talker = roslibpy.Topic(client, topic, 'std_msgs/String')
        while client.is_connected:
            talker.publish(roslibpy.Message({'data': text}))
            print('Sending message...')
            time.sleep(interval)
    finally:
        talker.unadvertise()

def generate_numbers(client, topic, interval=1):
    try:
        talker = roslibpy.Topic(client, topic, 'std_msgs/Int32')

        i = 0
        while client.is_connected:
            talker.publish(roslibpy.Message(i))
            i += 1
            print('Sending message...')
            time.sleep(interval)
    finally:
        talker.unadvertise()


def generate_video(client, topic, src=0, img_format='jpeg'):
    publisher = roslibpy.Topic(client, topic, 'sensor_msgs/CompressedImage')
    publisher.advertise()

    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video source: {src}")

    try:
        with tqdm.tqdm() as pbar:
            while client.is_connected:
                # read
                ret, im = cap.read()
                if not ret:
                    time.sleep(0.3)
                    continue

                buf = io.BytesIO()
                pil.fromarray(im).save(buf, img_format)
                encoded = base64.b64encode(buf.getvalue()).decode('ascii')
                publisher.publish(dict(format=img_format, data=encoded))    
                # pbar.set_description(f'{pbar.n / (time.time() - t0):.4f}')
                pbar.update()      
    finally:
        cap.release()


def show(name):
    pass


TOPICS = {
    '/helloworld': generate_string,
    '/cam': generate_video,
}

def main(topic='/helloworld', *a, **kw):
    ros = roslibpy.Ros(host='localhost', port=9090)
    ros.run()
    print(ros)
    print(ros.is_connected)

    try:
        TOPICS[topic](ros, topic, *a, **kw)
    finally:
        ros.terminate()

if __name__ == "__main__":
    import fire
    fire.Fire(main)