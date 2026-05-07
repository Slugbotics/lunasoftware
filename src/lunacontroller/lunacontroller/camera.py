import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from lunacontroller import constants

def cv2_to_imgmsg(frame, encoding='bgr8'):
    msg = Image()
    if frame is None:
        return msg
    if frame.ndim == 2:
        height, width = frame.shape
        channels = 1
        msg.encoding = 'mono8'
    else:
        height, width, channels = frame.shape
        if encoding == 'bgr8':
            msg.encoding = 'bgr8'
        elif encoding == 'rgb8':
            # convert BGR (cv2) to RGB for message
            frame = frame[:, :, ::-1]
            msg.encoding = 'rgb8'
        else:
            msg.encoding = encoding
    msg.height = int(height)
    msg.width = int(width)
    msg.is_bigendian = 0
    msg.step = int(width * (channels if 'channels' in locals() else 1))
    msg.data = frame.tobytes()
    return msg


class Camera:
    def __init__(self, node, topic, camera_id=0):
        self.topic = topic
        self.camera_id = camera_id
        self.node = node
        self.publisher = self.node.create_publisher(Image, self.topic, 10)
        self.cap = None
        self.connect_camera()
        if self.cap is None:
            self.node.get_logger().warning('No camera connected at initialization')

    def connect_camera(self):
        if self.cap is not None and self.cap.isOpened():
            return
        if self.cap is not None and not self.cap.isOpened():
            self.cap = None
            self.node.get_logger().warning('Camera disconnected')
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                self.cap = None
            else:
                self.node.get_logger().info('Camera connected')
        except:
            self.cap = None

    def update(self):
        self.connect_camera()
        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                frame = cv2.resize(frame, constants.CAMERA_RESOLUTION)
                msg = cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(msg)

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None