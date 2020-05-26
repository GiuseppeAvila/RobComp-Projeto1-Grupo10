#! /usr/bin/env python
# -*- coding:utf-8 -*-

# IMPORTS

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math


import visao_module
import follow
import creeper


# MONTANDO AS VARIÁVES

bridge = CvBridge()

cv_image = None
cv_image_1 = None
media = []
centro_road = None
centro_frame = None
centro_creeper = None
atraso = 1.5E9 
y_road = 0
y_frame = 0
x_road = 0
x_frame = 0
estado = False
find = False
distancia = 0

area = 0.0
check_delay = False 

resultados = [] 

x_robo = 0.0
y_robo = 0.0
z_robo = 0.0

x_marker = 0.0
y_marker = 0.0
z_marker = 0.0
id = None

frame = "camera_link"

tfl = 0

tf_buffer = tf2_ros.Buffer()

iden = 11 


def recebe(msg):
    global x_marker # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
    global y_marker
    global z_marker
    global id
    for marker in msg.markers:
        id = marker.id
        marcador = "ar_marker_" + str(id)

        print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
        header = Header(frame_id=marcador)
        # Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
        # Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
        # Nao ser que queira levar angulos em conta
        trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
        
        # Separa as translacoes das rotacoes
        x_marker = trans.transform.translation.x
        y_marker = trans.transform.translation.y
        z_marker = trans.transform.translation.z
        # ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
        # Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
        # no eixo X do robo (que e'  a direcao para a frente)
        t = transformations.translation_matrix([x_marker, y_marker, z_marker])
        # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
        r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
        z_marker2 = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
        v2 = numpy.dot(m, z_marker2)
        v2_n = v2[0:-1] # Descartamos a ultima posicao
        n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
        x_robo = [1,0,0]
        cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
        angulo_marcador_robo = math.degrees(math.acos(cosa))


        # Terminamos
        print("id: {} x {} y {} z {} angulo {} ".format(id, x_marker,y_marker,z_marker, angulo_marcador_robo))




# FUNCAO PARA FRAMES

def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro_frame
    global centro_road
    global centro_creeper
    global resultados
    global estado

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        # CHAMADA PARA SEGUIR AS LINHAS AMARELAS 
        saida_follow , centro_frame, centro_road =  follow.image_callback(temp_image) 
        # CHAMADA PARA IDENTIFICACAO DOS CREEPERS 
        saida_creeper, centro_creeper, estado   = creeper.image_callback(temp_image, "pink")    
        for r in resultados:    
            pass

        depois = time.clock()
        cv_image = saida_creeper.copy()
    except CvBridgeError as e:
        print('ex', e)

def follow_road():

    x_speed = 0
    z_twist = 0
    
    for r in centro_road:
        x_road,y_road = centro_road

    for r in centro_frame:
        x_frame,y_frame = centro_frame

    if x_road -3 > x_frame:
        z_twist = -0.15

    if x_road +3 < x_frame:
        z_twist = 0.15  

    if x_road -20 < x_frame < x_road +20:
        x_speed = 5
    
    return x_speed, z_twist

def go_to_creeper():
    
    global distancia

    x_speed = 0
    z_twist = 0
    
    for r in centro_creeper:
        x_creeper,y_creeper = centro_creeper

    for r in centro_frame:
        x_frame,y_frame = centro_frame

    if x_creeper -3 > x_frame:
        z_twist = -0.15

    if x_creeper +3 < x_frame:
        z_twist = 0.15   

    if x_creeper-20 < x_frame < x_creeper+20:
        z_twist = 0
        if distancia > 5: #no blue mudar para 6
            x_speed = 3
        elif distancia > 2:
            x_speed = 3.5 #no blue mudar para 4
        elif distancia > 1:
            x_speed = 2
        elif distancia > 0.5:
            x_speed = 0.2
        elif distancia > 0.12:
            x_speed = 0.1

        #Descomentar caso for blue
        #else: 
            #x_speed = 0.3
            #z_twist = -0.3

        elif distancia < 0.12 and distancia > 0.05:
            x_speed = 0.1
    
    return x_speed, z_twist


#CODIGO MAIN        
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultadosrecebe
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # Inicializando - por default gira no sentido anti-horário
        
        while not rospy.is_shutdown():

            distancia = math.sqrt(x_marker**2 + y_marker**2)
            print("Distancia: {}".format(distancia))

            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            print(estado)

            if estado == True and centro_creeper is not None:

                if id == iden or find == True: 

                    x_speed, z_twist = go_to_creeper()

                    vel = Twist(Vector3(x_speed,0,0), Vector3(0,0,z_twist))

                if id == iden:
                    find = True

                elif id == None or id != iden and find == False:

                    x_speed, z_twist = follow_road()

                    vel = Twist(Vector3(x_speed,0,0), Vector3(0,0,z_twist))


            if estado == False and centro_road is not None:
                
                    x_speed, z_twist = follow_road() 

                    vel = Twist(Vector3(x_speed,0,0), Vector3(0,0,z_twist))


            #elif distancia > 0.02 and distancia < 0.05:
             #   vel = Twist(Vector3(-0.04,0,0), Vector3(0,0,z_twist))                

            
            velocidade_saida.publish(vel)

            if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("cv_image no loop principal", cv_image)
                cv2.waitKey(1)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


