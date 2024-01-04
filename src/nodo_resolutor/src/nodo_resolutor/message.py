#!/usr/bin/env python 

import rospy
import time
from std_msgs.msg import String

rospy.init_node('nodo_publicador')

cube_solved = None

def recibir_mensajes(data: String) -> None:
    global cube_solved 
    cube_solved = data.data

pub = rospy.Publisher('/cube_unsolved', String, queue_size=10)
rospy.Subscriber('/cube_solved', String, recibir_mensajes)

rate = rospy.Rate(1)  # Publicar a una velocidad de 1 Hz

while not rospy.is_shutdown():
    message = input("Introduce el cubo a resolver: ")
    pub.publish(message)
    print(cube_solved)
    rate.sleep()