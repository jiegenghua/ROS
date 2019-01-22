#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect_base')

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from face_detect_base.msg import Face
from sensor_msgs.msg import Image


min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True
L = 330*10**(-2)
R = 20*10**(-2)

wheelPubL = rospy.Publisher("/vrep/leftWheelCommand", std_msgs.msg.Float64, queue_size=1)
wheelPubR = rospy.Publisher("/vrep/rightWheelCommand", std_msgs.msg.Float64, queue_size=1)
pub = rospy.Publisher("/vrep/twistCommand", Twist,queue_size=10)
pubFace = rospy.Publisher('face_publisher', Face, queue_size = 10)
image_pub = rospy.Publisher("faces_image", Image, queue_size = 1)


if __name__ == '__main__':
     
    opencv_dir = '/usr/share/opencv/haarcascades/';
    face_msg = Face()
    face2 = []
    face_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print "Could not find face cascade"
        sys.exit(-1)
    eye_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_eye.xml')
    if eye_cascade.empty():
        print "Could not find eye cascade"
        sys.exit(-1)
    br = CvBridge()
    rospy.init_node('facedetect')
    display = rospy.get_param("~display",True)

    def detect_and_draw(imgmsg):
        ROI_Interest = sensor_msgs.msg.RegionOfInterest()
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
         #print('faces detected',faces)
        for (x,y,w,h) in faces:
            ROI_Interest.x_offset=x
            ROI_Interest.y_offset=y
            ROI_Interest.width=w
            ROI_Interest.height=h
            face2.append(ROI_Interest)
            ROI_Interest = sensor_msgs.msg.RegionOfInterest()
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            #eyes = eye_cascade.detectMultiScale(roi_gray)
            #for (ex,ey,ew,eh) in eyes:
            #    cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

        #print('FACES RECORDED',face2)
        face_msg.faces=face2
        pubFace.publish(face_msg)
        image_pub.publish(br.cv2_to_imgmsg(img, "bgr8"))
        del face2[:]
        cv2.imshow('img',img)
        cv2.waitKey(10)
        
    def joycmd(data):
		twist = Twist()
		twist.linear.x = 0.5*data.axes[1]
		twist.angular.z= 0.5*data.axes[0]
		velocity_filter(twist)
    
    def velocity_filter(twist):
		a = twist.linear.x
		b = twist.angular.z
		wl = (a - b*L/2)/R
		wr = (a + b*L/2)/R
		wheelPubL.publish(wl)
		wheelPubR.publish(wr)
		#pub.publish(twist)

    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_draw)
    rospy.Subscriber("/joy", Joy, joycmd)
    
    rospy.spin()

