#!/usr/bin/env python 

import rospy
from std_msgs.msg import String

rospy.init_node("primer_subscriber",anonymous=True)

def recibir_mensajes(data: String) -> None:
    print(data.data)

rospy.Subscriber("/mi_topico", String, recibir_mensajes)

rospy.spin()