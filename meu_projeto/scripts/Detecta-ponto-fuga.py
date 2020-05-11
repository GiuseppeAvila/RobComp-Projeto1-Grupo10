
import cv2
import numpy as np 

#utilizar min_length da captura para melhores resultados
#senão utilizar valor default

default_min_length = 200
min_length = default_min_length

#cap = cv2.VideoCapture('teste-1.mp4')
#min_length = 150

cap = cv2.VideoCapture('teste-2.mp4')
#cap = cv2.VideoCapture('teste-3.mp4')
min_length = 250

if (cap.isOpened()== False):
    print ("error")

hsv1_M = np.array([1, 0, 240], dtype=np.uint8)
hsv2_M= np.array([150, 20, 255], dtype=np.uint8)

# placeholders
aMed_esq = 1
bMed_esq = 1
rhoMed_esq = 1
aMed_dir = 1
bMed_dir = 1
rhoMed_dir = 1

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)

	# return the edged image
	return edged

def desenhar_reta_media(a, b, rho):
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 10000*(-b))
    y1 = int(y0 + 10000*(a))
    x2 = int(x0 - 10000*(-b))
    y2 = int(y0 - 10000*(a))
    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),1)

def interseccao(a1, b1, rho1, a2, b2, rho2):
    x0_1 = a1*rho1
    y0_1 = b1*rho1
    x1_1 = int(x0_1 + 10000*(-b1))
    y1_1 = int(y0_1 + 10000*(a1))
    x2_1 = int(x0_1 - 10000*(-b1))
    y2_1 = int(y0_1 - 10000*(a1))

    m_1 = (y2_1 - y1_1) / (x2_1 - x1_1)
    h_1 = y1_1 - (m_1 * x1_1)

    x0_2 = a2*rho2
    y0_2 = b2*rho2
    x1_2 = int(x0_2 + 10000*(-b2))
    y1_2 = int(y0_2 + 10000*(a2))
    x2_2 = int(x0_2 - 10000*(-b2))
    y2_2 = int(y0_2 - 10000*(a2))

    m_2 = (y2_2 - y1_2) / (x2_2 - x1_2)
    h_2 = y1_2 - (m_2 * x1_2)

    x_ponto = (h_2 - h_1) / (m_1 - m_2)
    y_ponto = int((m_1 * x_ponto) + h_1)
    x_ponto = int(x_ponto)

    cv2.circle(frame,(x_ponto,y_ponto),10,(0,255,0),-1)

while (cap.isOpened()):
    ret,frame = cap.read()
    lista_ab = []
    a_esq = []
    b_esq = []
    rho_esq = []
    a_dir = []
    b_dir = []
    rho_dir = []


    if ret == True:

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
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
                    a_esq.append(abrho[0])
                    b_esq.append(abrho[1])
                    rho_esq.append(abrho[2])
                elif 18 > abrho[0] > 0.1 :
                    a_dir.append(abrho[0])
                    b_dir.append(abrho[1])
                    rho_dir.append(abrho[2])

        if (len(a_esq) &  len(b_esq) & len(rho_esq)) != 0:
            aMed_esq = sum(a_esq) / len(a_esq)
            bMed_esq = sum(b_esq) / len(b_esq)
            rhoMed_esq = sum(rho_esq) / len(rho_esq)

        if (len(a_dir) &  len(b_dir) & len(rho_dir)) != 0:
            aMed_dir = sum(a_dir) / len(a_dir)
            bMed_dir = sum(b_dir) / len(b_dir)
            rhoMed_dir = sum(rho_dir) / len(rho_dir)
        
        desenhar_reta_media(aMed_esq, bMed_esq, rhoMed_esq)
        desenhar_reta_media(aMed_dir, bMed_dir, rhoMed_dir)
        interseccao(aMed_esq, bMed_esq, rhoMed_esq, aMed_dir, bMed_dir, rhoMed_dir)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(mask1,'Press q to quit',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.imshow ('Frame', frame)

        if cv2.waitKey(25) & 0XFF == ord('q'):
            break

    else:
        break

cap.release()

cv2.destroyAllWindows()
