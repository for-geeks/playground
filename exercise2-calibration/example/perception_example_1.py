#!/usr/bin/env python
import sys
import time

import numpy as np
import cv2

from cyber_py3 import cyber
from modules.sensors.proto.sensor_image_pb2 import Image

sys.path.append("../")

# roll
src_corners = [[274, 250], [438, 252], [250, 339], [502, 341]]

# turn to
dst_corners = [[262, 470], [342, 470], [262, 550], [342, 550]]

M = cv2.getPerspectiveTransform(
    np.float32(src_corners), np.float32(dst_corners))


def perspective_transform(image, m, img_size=None):
    if img_size is None:
        img_size = (image.shape[1], image.shape[0])
    warped = cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_LINEAR)
    return warped


class Exercise(object):

    def __init__(self, node):
        self.node = node
        self.msg = Image()

        # create reader
        self.node.create_reader(
            "/realsense/color_image/compressed", Image, self.callback)
        # create writer
        self.writer = self.node.create_writer(
            "/perception/vertical_view", Image)

    def callback(self, data):
        # data frame 
        print(data.frame_no)
        # APT to image reshape
        self.reshape(data)
        # publish, write to channel
        self.write_to_channel()

    def write_to_channel(self):
        # write message to channel
        self.writer.write(self.msg)

    def reshape(self, data):
        new_image = np.frombuffer(data.data, dtype=np.uint8)
        image = cv2.imdecode(new_image, cv2.IMREAD_COLOR)

        wrap_img = perspective_transform(image, M, img_size=(580, 560))

        img_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
        #new_image = new_image.reshape(816/2, 848/2)

        img_encode = cv2.imencode('.jpeg', wrap_img, img_param)[1]
        data_encode = np.array(img_encode)
        str_encode = data_encode.tostring()
        data.data = str_encode
        self.msg = data


if __name__ == '__main__':
    cyber.init()

    # update node to your name
    exercise_node = cyber.Node("your_name")
    exercise = Exercise(exercise_node)

    exercise_node.spin()

    cyber.shutdown()
