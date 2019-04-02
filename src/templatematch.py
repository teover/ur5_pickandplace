#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from matplotlib import pyplot as plt
import time


def callback(data):

    # raw image data in
    img_rgb = ros_numpy.numpify(data)

    # converts to grayscale
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    # applies gaussian blur filter
    img_blurred = cv2.GaussianBlur(img_gray,(5,5),0)
    # reads template data

    template = cv2.imread('template.png',0)
    # blurs template
    if not template.any():
        print "No template found"
    template_blurred = cv2.GaussianBlur(template,(5,5),0)
    w, h = template.shape[::-1]

    # matches pattern against image data, creates a intensity map of match confidence
    res = cv2.matchTemplate(img_blurred,template_blurred,cv2.TM_CCOEFF_NORMED)

    # # For single rectangle drawn - always draws rectangle, no threshhold
    # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    # top_left = max_loc
    # bottom_right = (top_left[0] + w, top_left[1] + h)
    # cv2.rectangle(img_rgb,top_left, bottom_right, (0,0,255), 2)
    # match threshhold, 0-1(%)

    threshold = 0.7
    #
    loc = np.where( res >= threshold)
    coords = zip(*loc[::-1])

    # draw ractangle at every match

    for point in coords:
        cv2.rectangle(img_rgb, point, (point[0] + w, point[1] + h), (0,0,255), 2)

    # draws image
    cv2.imshow("res", img_rgb)
    cv2.waitKey(1)

# check if no coords are found and publish to publisher

    pub = rospy.Publisher('/object_status', Bool, queue_size=10)

    if not coords:
        pub.publish(False)
    else:
        pub.publish(True)


    # sets display frequency
    time.sleep(0.01)



# resten er kode jeg bare subscriber kode jeg kopierte fra beginner tutorialen, ser ut til aa funke.

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
