# -*- encoding: UTF-8 -*-

import sys
import time
import numpy as np
import Image
from naoqi import ALProxy
import cv2

def showNaoImage(IP, PORT):

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 2    # VGA
  colorSpace = 11   # RGB

  videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)
  while 1:
    t0 = time.time()

    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    naoImage = camProxy.getImageRemote(videoClient)

    t1 = time.time()

    # Time the image transfer.
    # print "acquisition delay ", t1 - t0
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]

    # Create a PIL Image from our pixel array.
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
    # Creates an opencv ready image
    open_cv_image = np.array(im)
    # convert from BGR to RGB
    open_cv_image = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2RGB)
    # create a copy in grayscale and apply a blur just for testing
    # img = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2GRAY)
    img = cv2.Canny(open_cv_image, 100, 200);
    lines = cv2.HoughLines(img, 1, np.pi / 90, 200)
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(open_cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)



    # create two windows to display the images
    cv2.imshow('gray_frame', img)
    cv2.imshow('original_frame', open_cv_image)
    cv2.waitKey(1)


if __name__ == '__main__':
  IP = "127.0.0.1"  # Replace here with your NaoQi's IP address.
  PORT = 9559

  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    IP = sys.argv[1]

  naoImage = showNaoImage(IP, PORT)


