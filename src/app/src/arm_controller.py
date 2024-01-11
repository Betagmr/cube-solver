#!/usr/bin/env python 

import rospy
from std_msgs.msg import String


class ArmMoveService:
    def __init__(self) -> None:
        rospy.init_node('nodo_publicador')

        self.publisher = rospy.Publisher('/arm_controller', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/arm_log', String, self.get_message_callback)
        self.rate = rospy.Rate(1) 

        self.solution = None

    def launch_movements(self, steps: str) -> str:
        self.solution = None
        self.publisher.publish(steps)

        while self.solution is None:
            self.rate.sleep()

        if self.solution == "Error":
            return False

        return True

    def get_message_callback(self, data: String) -> None:
        self.solution = data.data