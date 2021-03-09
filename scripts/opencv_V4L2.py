'''
References:
https://www.pyimagesearch.com/2014/09/29/finding-brightest-spot-image-using-python-opencv/
https://gist.github.com/royshil/0f674c96281b686a9a62
https://docs.opencv.org/master/d7/d4d/tutorial_py_thresholding.html
https://roboticsbackend.com/ros2-python-publisher-example/
'''

# !/usr/bin/env python

import numpy as np
import cv2
import v4l2capture
import select
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64

# variables
radius = 41  # radius of circle
bg_threshold = 127  # 0 to 255


class CVCommandsNode(Node):
    def __int__(self):
        super().__init__('cv_commands')
        self.cv_commands_publisher_ = self.create_publisher(Int64, 'cv_commands', 30)

    def publish_cv_commands(self):
        command = 0
        msg = Int64
        msg.data = command
        self.cv_commands_publisher_.publish(msg)
        self.cv_command_timer_ = self.create_timer(0.1, self.publish_cv_commands)

def main(args=None):
    rclpy.init(args=args)
    command_node = CVCommandsNode()
    rclpy.spin(command_node)
    rclpy.shutdown()


if __name__ == '__main__':
    # Open the video device.
    video = v4l2capture.Video_device("/dev/video0")

    # Suggest an image size to the device. The device may choose and
    # return another size if it doesn't support the suggested one.
    size_x, size_y = video.set_format(1920, 1080, fourcc='MJPG')

    print("device chose {0}x{1} res".format(size_x, size_y))

    # Create a buffer to store image data in. This must be done before
    # calling 'start' if v4l2capture is compiled with libv4l2. Otherwise
    # raises IOError.
    video.create_buffers(30)

    # Send the buffer to the device. Some devices require this to be done
    # before calling 'start'.
    video.queue_all_buffers()

    # Start the device. This lights the LED if it's a camera that has one.
    print("start capture")
    video.start()

    while True:
        # Wait for the device to fill the buffer.
        select.select((video,), (), ())

        # OpenCV stuffs
        image_data = video.read_and_queue()
        frames = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.cv.CV_LOAD_IMAGE_GRAYSCALE)
        ret, thresh1 = cv2.threshold(frames, bg_threshold, 255, cv2.THRESH_BINARY)
        orig = frames.copy()
        # frames = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)

        # apply a Gaussian blur to the image then find the brightest
        # region
        thresh1 = cv2.GaussianBlur(thresh1, (radius, radius), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frames)
        frames = orig.copy()
        cv2.circle(frames, maxLoc, radius, (255, 0, 0), 2)

        cv2.imshow('Video Stream', frames)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break

    # cap.release()
    video.close()
    cv2.destroyAllWindows()
