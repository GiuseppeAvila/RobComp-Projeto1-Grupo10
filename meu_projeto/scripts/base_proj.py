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



########

COR = "pink"

IDENT = 12



############


# MONTANDO AS VARIÁVES

bridge = CvBridge()

cv_image = None
cv_image_1 = None
media = []
centro_bola = None
centro_frame = None
centro_creeper = None
atraso = 1.5E9 
y_bola = 0
y_frame = 0
x_bola = 0
x_frame = 0
estado = False
find = False
distancia = 0
cu =0
co = 0
d = False

obj_x = 0
obj_y = 0

area = 0.0
check_delay = False 

resultados = [] 

x = None
y = None
z = None
id = None

frame = "camera_link"

tfl = 0

tf_buffer = tf2_ros.Buffer()



def recebe(msg):
    global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
    global y
    global z
    global id
    global d
    for marker in msg.markers:

        id = marker.id

        if id == IDENT:

            d = True
            marcador = "ar_marker_" + str(id)

            print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
            header = Header(frame_id=marcador)
            # Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
            # Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
            # Nao ser que queira levar angulos em conta
            trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
            
            # Separa as translacoes das rotacoes
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            # ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
            # Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
            # no eixo X do robo (que e'  a direcao para a frente)
            t = transformations.translation_matrix([x, y, z])
            # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
            r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
            z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
            v2 = numpy.dot(m, z_marker)
            v2_n = v2[0:-1] # Descartamos a ultima posicao
            n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
            x_robo = [1,0,0]
            cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
            angulo_marcador_robo = math.degrees(math.acos(cosa))

            # Terminamos
            print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))

        else:
            pass




# FUNCAO PARA FRAMES

def roda_todo_frame(imagem):
    global cv_image
    global cv_image_1
    global media
    global centro_frame
    global centro_bola
    global centro_creeper
    global resultados
    global estado
    global cu

    global co

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
        saida_follow , centro_frame, centro_bola =  follow.image_callback(temp_image) 
        # CHAMADA PARA IDENTIFICACAO DOS CREEPERS 
        saida_creeper, centro_creeper, estado   = creeper.image_callback(temp_image, COR, d)   


        for r in resultados:    
            pass

        depois = time.clock()
        cv_image = saida_follow.copy()
        cv_image_1 = saida_creeper.copy()
    except CvBridgeError as e:
        print('ex', e)


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

        twist = 0.15 

        #distancia = int(math.sqrt(x^2 + y^2))

        
        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

            print (estado)
            

            if estado == True:

                x_bola,y_bola = centro_creeper


                x_frame,y_frame = centro_frame

                if x_bola -3 > x_frame:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-twist))

                if x_bola +3 < x_frame:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,twist))   

                if x_bola-20 < x_frame < x_bola+20:
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0)) 

                
            if centro_bola is None and estado == False:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-twist)) 

            if estado == False and centro_bola is not None:
                x_bola,y_bola = centro_bola

                x_frame,y_frame = centro_frame

                if x_bola -3 > x_frame:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-twist))

                if x_bola +3 < x_frame:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,twist))   

                if x_bola-20 < x_frame < x_bola+20:
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0)) 







            
            velocidade_saida.publish(vel)

            if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("cv_image no loop principal", cv_image_1 )
                cv2.waitKey(1)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


