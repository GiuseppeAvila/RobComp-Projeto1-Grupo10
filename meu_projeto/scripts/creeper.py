
#-*- coding:utf-8 -*-

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

saida = False

def image_callback(image, color, D):
	global frame
	global saida
	frame = image.copy()
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	if color == "green":
		lower_color = numpy.array([51, 50, 50])
		upper_color = numpy.array([61, 255, 255])


	elif color == "pink":
		lower_color = numpy.array([140,  50,  50])
		upper_color = numpy.array([150, 255, 255])



	elif color == "blue":

		lower_color = numpy.array([94, 50, 50])
		upper_color = numpy.array([104, 255, 255])



	#def cross(img_rgb, point, color, width,length):
    	#cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
    	#cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length) 



    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
    # que um quadrado 7x7. É muito útil para juntar vários 
    # pequenos contornos muito próximos em um só.


	
	segmentado_cor = cv2.inRange(hsv, lower_color, upper_color)
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,numpy.ones((10, 10)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

	maior_contorno = None
	maior_contorno_area = 0

	if D == True:

		saida = True
	else:
		saida = False
	for cnt in contornos:
		area = cv2.contourArea(cnt)
		if area >maior_contorno_area:
			maior_contorno = cnt
			maior_contorno_area = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None and saida == True :
		cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)

		
		maior_contorno = numpy.reshape(maior_contorno, (maior_contorno.shape[0], 2))
		centro_bola = maior_contorno.mean(axis=0)
		centro_bola= centro_bola.astype(numpy.int32)
		cv2.circle(frame, (centro_bola[0],centro_bola[1]), 5, [0, 255, 0])



	else:
		centro_bola = None


	return frame, centro_bola, saida


