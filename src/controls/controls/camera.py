import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

def imgmsg_to_cv2(msg):
    # Convert sensor_msgs/Image to a cv2 (numpy) image. Supports 'bgr8', 'rgb8', 'mono8'.
    dtype = np.uint8
    height = msg.height
    width = msg.width
    enc = msg.encoding.lower()
    if 'bgr' in enc or 'rgb' in enc:
        channels = 3
    elif 'mono' in enc:
        channels = 1
    else:
        channels = 3
    arr = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        arr = arr.reshape((height, width))
    else:
        arr = arr.reshape((height, width, channels))
        if enc == 'rgb8':
            arr = arr[:, :, ::-1]
    return arr


class Camera:
    def __init__(self, node, topic):
        self.topic = topic
        self.node = node
        self.subscriber = self.node.create_subscription(Image, self.topic, self.image_callback, 10)

    def image_callback(self, msg):
        try:
            frame = imgmsg_to_cv2(msg)
            frame = cv2.resize(frame, (640, 480))
            cv2.imshow(f'{self.topic[1:]}', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.node.get_logger().error(f'Error processing image message: {e}')

    def close(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
