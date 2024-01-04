#!/usr/bin/env python 

import rospy
import kociemba
from std_msgs.msg import String

rospy.init_node("primer_subscriber", anonymous=True)

pub = rospy.Publisher('/cube_solved', String, queue_size=10)

def recibir_mensajes(data: String) -> None:
    cube_string = data.data 
    try:
        solution = kociemba.solve(cube_string)
    except ValueError:
        solution = None 

    print("Cubo recibido:", cube_string) 
    print("La solución es:", solution)

    is_valid = solution is not None

    if rospy.is_shutdown() == False and is_valid:
        pub.publish(solution)
    else:
        pub.publish("BAD CONFIG")

rospy.Subscriber("/cube_unsolved", String, recibir_mensajes)

rospy.spin()