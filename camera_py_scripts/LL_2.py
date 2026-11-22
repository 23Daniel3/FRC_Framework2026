import cv2
import numpy as np

# Seus valores originais (não mexi na calibração de cor)
LOWER_YELLOW = (20, 100, 100)
UPPER_YELLOW = (35, 255, 255)
MIN_AREA = 600

def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    llpython = [0,0,0,0,0,0,0,0]
    largestContour = np.array([[]])
    
    max_area = 0
    best_cnt = None

    # --- LÓGICA DE MULTIPLOS ALVOS ---
    if contours:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Se for maior que o mínimo, é um candidato
            if area > MIN_AREA:
                # Desenha um retângulo FINO em TODOS os candidatos (para você ver o que ele detecta)
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(image, (x,y), (x+w,y+h), (0, 255, 255), 1)
                
                # Verifica se este é o novo "Maior"
                if area > max_area:
                    max_area = area
                    best_cnt = cnt

    # --- FINALIZAÇÃO APENAS COM O MAIOR ALVO ---
    if best_cnt is not None:
        # Pega o círculo envolvente do MAIOR alvo
        (cx_float, cy_float), radius = cv2.minEnclosingCircle(best_cnt)
        cx = int(cx_float)
        cy = int(cy_float)
        
        # Desenha o círculo GROSSO no alvo escolhido
        cv2.circle(image, (cx, cy), int(radius), (0, 0, 255), 2)
        cv2.circle(image, (cx, cy), 4, (0, 255, 0), -1) # Centro verde

        # Dados para o robô (Bounding Box do maior)
        x, y, w, h = cv2.boundingRect(best_cnt)
        largestContour = best_cnt
        llpython = [1, cx, cy, w, h, int(max_area), 0, 0]

    return largestContour, image, llpython