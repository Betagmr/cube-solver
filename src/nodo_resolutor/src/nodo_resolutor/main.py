#!/usr/bin/env python 

import rospy
from std_msgs.msg import String

rospy.init_node('nodo_publicador')

pub = rospy.Publisher('/mi_topico', String, queue_size=10)

rate = rospy.Rate(1)  # Publicar a una velocidad de 1 Hz

while not rospy.is_shutdown():
    mensaje = "Hola desde el nodo publicador"
    pub.publish(mensaje)
    rate.sleep()