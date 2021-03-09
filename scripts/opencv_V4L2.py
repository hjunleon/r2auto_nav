'''
References:
https://www.pyimagesearch.com/2014/09/29/finding-brightest-spot-image-using-python-opencv/
https://gist.github.com/royshil/0f674c96281b686a9a62
'''


#!/usr/bin/env python

import numpy as np
import cv2
import os
import v4l2capture
import select
import argparse

# variables
radius = 41
bg_threshold = 0  # 0 to 255


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
        ret, thresh1 = cv2.threshold(frames, 127, 255, cv2.THRESH_BINARY)
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
