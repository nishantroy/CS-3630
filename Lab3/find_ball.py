#!/usr/bin/env python3

# Madelyn Juby, Nishant Roy

import cv2
import sys
import copy
from math import sqrt

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit('install Pillow to run this code')


def gammaCorrect(opencv_image, gamma):
    gamma = 1.0 / gamma

    gammaLookupTable = np.array([((i / 255.0) ** gamma) * 255
                                 for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(opencv_image, gammaLookupTable)


def find_ball(opencv_image, debug=False):
    """Find the ball in an image.

        Arguments:
        opencv_image -- the image
        debug -- an optional argument which can be used to control whether
                debugging information is displayed.

        Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
    """

    imgHeight, imgWidth = opencv_image.shape[:2]  # Get ball dimensions

    opencv_image = cv2.medianBlur(opencv_image, 5)  # Blur the image to reduce noise

    # opencv_image = gammaCorrect(opencv_image, 1.8)  # Use gamma correction to brighten the image, reducing shadow
    # opencv_image = cv2.equalizeHist(opencv_image)
    opencv_image = cv2.normalize(opencv_image, opencv_image, 0, 700, cv2.NORM_MINMAX)



    circles = cv2.HoughCircles(opencv_image, cv2.HOUGH_GRADIENT, 2, 1,
                               param1=250, param2=20, minRadius=6, maxRadius=300)  # Look for circles


    # cv2.imshow("processed", opencv_image)
    # cv2.waitKey(1)
    ball = [0, 0, 0]  # Initialize ball

    if circles is not None:

        ball = circles[0][0]
        r = int(ball[2])
        #print("radius: ", r)

        xCoord = int(ball[0])
        yCoord = int(ball[1])

        pixelSum = 0
        pixelCount = 0

        xmin = int(xCoord - r) if (xCoord - r >= 0) else 0

        xmax = int(xCoord + r) if (xCoord + r <= imgWidth - 1) else imgWidth

        ymin = int(yCoord - r) if (yCoord - r >= 0) else 0

        ymax = int(yCoord + r) if (yCoord + r < imgHeight - 1) else imgHeight - 1

        for x in range(xmin, xmax):
            for y in range(ymin, ymax):
                if (sqrt((x - xCoord) ** 2 + (y - yCoord) ** 2) <= r):
                    pixel = opencv_image[y][x]
                    pixelSum += pixel
                    pixelCount += 1

        pixelAverage = int(pixelSum / pixelCount)
        #print("pixel average: ", pixelAverage)

        if (pixelAverage > 95):
            ball = [0, 0, 0]

    return ball


def display_circles(opencv_image, circles, best=None):
    """Display a copy of the image with superimposed circles.

       Provided for debugging purposes, feel free to edit as needed.

       Arguments:
        opencv_image -- the image
        circles -- list of circles, each specified as [x,y,radius]
        best -- an optional argument which may specify a single circle that will
                be drawn in a different color.  Meant to be used to help show which
                circle is ranked as best if there are multiple candidates.

    """
    # make a copy of the image to draw on
    circle_image = copy.deepcopy(opencv_image)
    circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)

    for c in circles:
        # draw the outer circle
        cv2.circle(circle_image, (c[0], c[1]), c[2], (255, 255, 0), 2)
        # draw the center of the circle
        cv2.circle(circle_image, (c[0], c[1]), 2, (0, 255, 255), 3)
        # write coords
        cv2.putText(circle_image, str(c), (c[0], c[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (255, 255, 255), 2, cv2.LINE_AA)

        # highlight the best circle in a different color
    if best is not None:
        # draw the outer circle
        cv2.circle(circle_image, (best[0], best[1]), best[2], (0, 0, 255), 2)
        # draw the center of the circle
        cv2.circle(circle_image, (best[0], best[1]), 2, (0, 0, 255), 3)
        # write coords
        cv2.putText(circle_image, str(best), (best[0], best[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (255, 255, 255), 2, cv2.LINE_AA)

        # display the image
    pil_image = Image.fromarray(circle_image)
    pil_image.show()


if __name__ == "__main__":
    pass
