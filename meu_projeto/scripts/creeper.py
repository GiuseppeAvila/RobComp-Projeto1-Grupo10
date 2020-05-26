
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

def image_callback(image, color):

	if color == "green":
		lower_color = numpy.array([51, 50, 50])
		upper_color = numpy.array([61, 255, 255])


	elif color == "pink":
		lower_color = numpy.array([140,  50,  50])
		upper_color = numpy.array([150, 255, 255])



	elif color == "blue":

		lower_color = numpy.array([94, 50, 50])
		upper_color = numpy.array([104, 255, 255])



	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_color, upper_color)
	segmentado_cor = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,numpy.ones((300, 300)))
	selecao = cv2.bitwise_and(image, image, mask=segmentado_cor)
	hsv = cv2.cvtColor(selecao, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv, lower_color, upper_color)




	h, w, d = image.shape
	M = cv2.moments(mask)
	if M['m00'] > 0:
	 	cx = int(M['m10']/M['m00'])
	 	cy = int(M['m01']/M['m00'])
		centro_bola = (cx, cy)

		cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
		saida = True
	else:
		centro_bola =  None
		saida = False


	return image, centro_bola, saida


