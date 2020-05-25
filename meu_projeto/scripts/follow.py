from __future__ import division, print_function


import rospy
import numpy as numpy
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet_simples as mnet

def cross(img_rgb, point, color, width,length):
  cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
  cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length) 


def image_callback(image):

  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  lower_yellow = numpy.array([ 20, 50, 50])
  upper_yellow = numpy.array([ 30, 255, 255])
  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
  segmentado_cor = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,numpy.ones((50, 50)))
  selecao = cv2.bitwise_and(image, image, mask=segmentado_cor)
    
  h, w, d = image.shape
  search_top = 3*h/4
  search_bot = 3*h/4 + 20
  #mask[0:search_top, 0:w] = 0
  #mask[search_bot:h, 0:w] = 0
  M = cv2.moments(mask)
  centro = (image.shape[1]//2, image.shape[0]//2)
  if M['m00'] > 0:
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    centro_bola = (cx, cy)

    cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
  else:
    centro_bola =  None

  cv2.line(image, (centro[0] - int(17/2), centro[1]),  (centro[0] + int(17/2), centro[1]), [255,0,0] ,1, 17)
  cv2.line(image, (centro[0], centro[1] - int(17/2)), (centro[0], centro[1] + int(17/2)), [255,0,0]  ,1, 17) 
    #cross(image, centro, [255,0,0], 1, 17)


    # END CONTROL
  return image, centro, centro_bola
