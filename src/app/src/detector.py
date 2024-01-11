import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import colorsys
 
#Para cada foto, realizar la detección.
def deteccion_caras(imagenes):
    resultado=''
    for image in imagenes:
        #Obtenemos los cuadraditos de cada cara.
        cuadraditos=preparar_imagen(image)
        #Obtenemos los colores de los cuadraditos de la cara.
        lista_colores=obtener_colores(cuadraditos)
        #Obtenemos el string de la cara.
        cara=obtener_resultado(lista_colores)
        resultado+=cara
        print('--------------------------')
    return resultado
 
#Función que dada una imagen obtiene el cubo y sus cuadraditos, y muestra una imagen por panalla para controlar la detección.
def preparar_imagen(image):
    return np.array([
            image[i * 23 + 5 : i * 23 +27, j * 23 + 5: j * 23 + 27]
            for i in range(3)
            for j in range(3)
        ])

def recortar_cubo(image):
    #Recortar el cubo.
    x1,y1=(305,155)
    x2,y2=(385,235)
    img_recortada=image[y1:y2,x1:x2]
    return img_recortada
 
#Función que dada la lista de cuadraditos de la cara, obtiene sus colores.
def obtener_colores(squares):
    #Creamos un array para almacenar los colores.
    colores=[]
 
    #Para cada cuadrado, obtenemos su color.
    for c in squares:
        #Usamos un píxel de color.
        color=c[10,10]
        #Obtenemos el color.
        nombre_color = obtener_nombre_color(color)
 
        #Si el nombre es rojo_2, lo ponemos a rojo.
        if nombre_color=='rojo_2':
            nombre_color='rojo'
 
        #Añadimos el color al array.
        colores.append(nombre_color)
    #Devolvemos la lista de colores de la cara.
    return colores
 
#Función que dado el código del color, devuelve el nombre.
def obtener_nombre_color(color_rgb):
    #Convertimos el color RGB a HSV.
    color_hsv = cv2.cvtColor(np.uint8([[color_rgb]]), cv2.COLOR_RGB2HSV)[0][0]
 
    #Extraemos el componente de matiz (hue).
    hue = color_hsv[0]
 
    #Comprobamos en qué rango se encuentra el color y asignar el nombre correspondiente.
    for nombre, (umbral_min, umbral_max) in umbrales.items():
        if umbral_min[0] <= hue <= umbral_max[0]:
            #Convertir umbrales a numpy arrays para usarlos.
            umbral_min_np = np.array(umbral_min)
            umbral_max_np = np.array(umbral_max)
 
            #Verificar si el color_hsv está dentro del rango.
            if np.all(umbral_min_np <= color_hsv) and np.all(color_hsv <= umbral_max_np):
                return nombre
    #Si es un color desconocido, se devuelve que es desconocido.
    return 'desconocido'
 
#Umbrales de cada color.
umbrales = {
    'rojo': ([0, 50, 20], [10, 255, 255]),      
    'naranja': ([10, 100, 100], [25, 255, 255]),
    'amarillo': ([25, 50, 80], [35, 255, 255]),
    'verde': ([35, 100, 100], [85, 255, 255]),
    'azul': ([85, 50, 20], [130, 255, 255]),
    'blanco': ([0, 0, 100], [180, 30, 255]),
    'rojo_2': ([130, 50, 20], [255, 255, 255])
}
 
#Función que dados los colores de los cuadraditos de la cara, devuelve el string correspondiente.
def obtener_resultado(lista_colores):
    #Creamos un string al que le iremos añadiendo las letras correspondientes a los colores.
    cara=''
 
    #Creamos el string según los colores.
    for color in lista_colores:
        if color=='rojo':
            cara+='R'
        elif color=='blanco':
            cara+='U'
        elif color=='azul':
            cara+='B'
        elif color=='verde':
            cara+='F'
        elif color=='amarillo':
            cara+='D'
        elif color=='naranja':
            cara+='L'
        else:
            cara+='?'
 
    print('Resultado: ',cara)
    return cara
 
 
#Función que dada la lista de colores de los cuadraditos y el color del centro de la cara, devuelve si coinciden.
def comprobar_centro(lista_colores,centro):
    if lista_colores[4]==centro:
        print('El color del centro es correcto.')
        return True
    else:
        print('No es el color indicado.')
        return False
 
#Función que abre la cámara y guarda las fotos que van a utilizarse.
def obtener_fotos():
    imagenes=[]
    #Capturamos el video
    captura = cv2.VideoCapture(0)
 
    while True:
        #Capturamos fotograma del video
        ret, fotograma = captura.read()
 
        #Rompe el bucle si no puede capturar el fotograma
        if not ret:
            break
 
        #Pintamos el rectángulo.
        x1,y1=(305,155)
        x2,y2=(385,235)
        imagen_cuadrado=cv2.rectangle(fotograma,(x1, y1), (x2, y2), (0,255,0), 2)
        #Mostramos el resultado obtenido.
        cv2.imshow('Ventana', imagen_cuadrado)
 
        #Capturamos la imagen y cierre del programa
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'): 
            imagenes.append(fotograma)
        elif key == ord('q'):
            break
 
    #Liberar la camara
    captura.release()
    cv2.destroyAllWindows()
    return imagenes
 
#------------------------------------------------------------------------------
 
'''imagenes = cv2.imread('fotoPrueba3.jpg')
 
imagenes=cv2.pyrDown(imagenes)
imagenes=cv2.pyrDown(imagenes)
 
imagen_cuadrado=cv2.rectangle(imagenes, (530, 365), (437, 460), (0,255,0), 2)
#Mostramos el resultado obtenido.
cv2.imshow('Ventana', imagen_cuadrado)
cv2.waitKey(0)'''
 
# imagenes = obtener_fotos()
# resultado = deteccion_caras(imagenes)

# print('RESULTADO FINAL: ',resultado)