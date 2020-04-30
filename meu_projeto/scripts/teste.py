#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros





def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged



def desenhar_reta_media(frame, a, b, rho):
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 10000*(-b))
    y1 = int(y0 + 10000*(a))
    x2 = int(x0 - 10000*(-b))
    y2 = int(y0 - 10000*(a))
    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),1)




def interseccao(frame, a1, b1, rho1, a2, b2, rho2):
    x0_1 = a1*rho1
    y0_1 = b1*rho1
    x1_1 = int(x0_1 + 10000*(-b1))
    y1_1 = int(y0_1 + 10000*(a1))
    x2_1 = int(x0_1 - 10000*(-b1))
    y2_1 = int(y0_1 - 10000*(a1))

    m_1 = int((y2_1 - y1_1) / (x2_1 - x1_1))
    h_1 = y1_1 - (m_1 * x1_1)

    x0_2 = a2*rho2
    y0_2 = b2*rho2
    x1_2 = int(x0_2 + 10000*(-b2))
    y1_2 = int(y0_2 + 10000*(a2))
    x2_2 = int(x0_2 - 10000*(-b2))
    y2_2 = int(y0_2 - 10000*(a2))

    m_2 = int((y2_2 - y1_2) / (x2_2 - x1_2))
    h_2 = y1_2 - (m_2 * x1_2)


    if (m_1- m_2) != 0:
        x_ponto = int(h_2 - h_1) / int((m_1 - m_2))
    else:
        x_ponto = 1

    y_ponto = int((m_1 * x_ponto) + h_1)


    return x_ponto , y_ponto

    cv2.circle(frame,(x_ponto,y_ponto),10,(0,255,0),-1)


def identifica_cor(frame):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''
    hsv1_M = np.array([ 0, 0, 0], dtype=np.uint8)
    hsv2_M= np.array([0, 0, 255], dtype=np.uint8)

    # placeholders

    valores_esq = { "a_esq" : [], "b_esq" : [], "rho_esq" : [],"aMed_esq" : 1.0, "bMed_esq" : 1.0, "rhoMed_esq" : 1.0}
    valores_dir = { "a_dir" : [], "b_dir" : [], "rho_dir" : [],"aMed_dir" : 1.0, "bMed_dir" : 1.0, "rhoMed_dir" : 1.0}

    #aMed_esq = 1
    #bMed_esq = 1
    #rhoMed_esq = 1
    #aMed_dir = 1
    #bMed_dir = 1
    #rhoMed_dir = 1

    min_length = 250
    lista_ab = []

    #a_esq = []
    #b_esq = []
    #rho_esq = []
    #a_dir = []
    #b_dir = []
    #rho_dir = []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, hsv1_M, hsv2_M)
    seg = cv2.morphologyEx(mask1,cv2.MORPH_CLOSE,np.ones((1, 1)))
    selecao = cv2.bitwise_and(frame, frame, mask=seg)
    blur = cv2.GaussianBlur(selecao,(5,5),0)
    min_contrast = 50
    max_contrast = 250
    linhas = cv2.Canny(blur, min_contrast, max_contrast )
    bordas_color = cv2.cvtColor(linhas, cv2.COLOR_RGB2BGR)
    lines = cv2.HoughLines(linhas, 1, np.pi/180, min_length)
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            lista_ab.append([a, b, rho])
        for abrho in lista_ab:
            if -18 < abrho[0] < -0.1 :
                valores_esq["a_esq"].append(abrho[0])
                valores_esq["b_esq"].append(abrho[1])
                valores_esq["rho_esq"].append(abrho[2])
            elif 18 > abrho[0] > 0.1 :
                valores_dir["a_dir"].append(abrho[0])
                valores_dir["b_dir"].append(abrho[1])
                valores_dir["rho_dir"].append(abrho[2])

    if (len(valores_esq["a_esq"]) &  len(valores_esq["b_esq"]) & len(valores_esq["rho_esq"])) != 0:
        valores_esq["aMed_esq"] = sum(valores_esq["a_esq"]) / len(valores_esq["a_esq"])
        valores_esq["bMed_esq"] = sum(valores_esq["b_esq"]) / len(valores_esq["b_esq"])
        valores_esq["rhoMed_esq"] = sum(valores_esq["rho_esq"]) / len(valores_esq["rho_esq"])

    if (len(valores_dir["a_dir"]) &  len(valores_dir["b_dir"]) & len(valores_dir["rho_dir"])) != 0:
        valores_dir["aMed_dir"] = sum(valores_dir["a_dir"]) / len(valores_dir["a_dir"])
        valores_dir["bMed_dir"] = sum(valores_dir["b_dir"]) / len(valores_dir["b_dir"])
        valores_dir["rhoMed_dir"] = sum(valores_dir["rho_dir"]) / len(valores_dir["rho_dir"])
        
    else:
        x_ponto = 1

    x_ponto, y_ponto= interseccao(frame, valores_esq["aMed_esq"], valores_esq["bMed_esq"], valores_esq["rhoMed_esq"], valores_dir["aMed_dir"], valores_dir["bMed_dir"], valores_dir["rhoMed_dir"])
    media = (x_ponto,y_ponto)

        
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(mask1,'Press q to quit',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow ('Frame', selecao)


    centro = (frame.shape[1]//2, frame.shape[0]//2)


    return media, centro