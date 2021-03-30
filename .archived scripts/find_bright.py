'''
References:
https://www.pyimagesearch.com/2014/09/29/finding-brightest-spot-image-using-python-opencv/
'''

# import the necessary packages
import numpy as np
import argparse
import cv2
import v4l2capture

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image file")
ap.add_argument("-r", "--radius", type = int, help = "radius of Gaussian blur; must be odd")
args = vars(ap.parse_args())

# apply a Gaussian blur to the image then find the brightest region
gray = cv2.GaussianBlur(gray, (args["radius"], args["radius"]), 0)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
image = orig.copy()
cv2.circle(image, maxLoc, args["radius"], (255, 0, 0), 2)
# display the results of our newly improved method
cv2.imshow("Video Stream", image)
cv2.waitKey(0)
if __name__ = '__main__':
  
  # Open the video device.
  video = v4l2capture.Video_device("/dev/video0")
  # Create a buffer to store image data in. This must be done before
  # calling 'start' if v4l2capture is compiled with libv4l2. Otherwise
  # raises IOError.
  video.create_buffers(3)
  # Send the buffer to the device. Some devices require this to be done
  # before calling 'start'.
  video.queue_all_buffers()
  # Start the device. This lights the LED if it's a camera that has one.
  video.start()
  # Wait for the device to fill the buffer.
  select.select((video,), (), ())
  
  while True:
    image_data = video.read_and_queue()
    image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
    image_data = image_data>>2
    image_data = image_data.astype(np.uint8)
    
    
    
    
