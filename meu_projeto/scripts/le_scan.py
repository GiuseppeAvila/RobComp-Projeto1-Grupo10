#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
menor_distancia = -1

def scaneou(dado):
	medicoes = np.array(dado.ranges).round(decimals=2)
	print (len(medicoes))
	
	frente = np.concatenate((medicoes[349:] , medicoes[:10]))
	

	global menor_distancia

	for valor in frente:
		if valor < dado.range_min:
			np.delete (frente,valor)
		if valor > dado.range_max:
			np.delete (frente,valor)
	menor_distancia =   min (frente)



	


