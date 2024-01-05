#!/usr/bin/env python 

import rospy
import kociemba
from std_msgs.msg import String


class CubeSolverServer:
    def __init__(self):
        rospy.init_node("primer_subscriber", anonymous=True)

        self.publisher = rospy.Publisher('/cube_solved', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/cube_unsolved', String, self.get_message_callback) 

    def get_message_callback(self, message: String) -> None:
        cube_string = message.data 
        try:
            solution = kociemba.solve(cube_string)
        except ValueError:
            solution = None 
            
        is_valid = solution is not None

        print("########### NUEVO CUBO ENVIADO ###########")
        print("Config:", cube_string) 
        print("Solucion:", solution)

        if not rospy.is_shutdown() and is_valid:
            self.publisher.publish(solution)
        else:
            self.publisher.publish("BAD CONFIG")


if __name__ == "__main__":
    try:
        node = CubeSolverServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
