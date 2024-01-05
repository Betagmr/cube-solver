#!/usr/bin/env python 

import rospy
from main import ControlRobot
from std_msgs.msg import String


class CubeSolverServer:

    def __init__(self):
        rospy.init_node("primer_subscriber", anonymous=True)

        self.publisher = rospy.Publisher('/cube_solved', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/cube_unsolved', String, self.get_message_callback) 
        self.robot_controller = ControlRobot(1)

    def get_message_callback(self, message: String) -> None:
        self.robot_controller.move_secuence(message.data)     

if __name__ == "__main__":
    try:
        node = CubeSolverServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass