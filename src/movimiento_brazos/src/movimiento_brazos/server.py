#!/usr/bin/env python 

import rospy
from main import ControlRobot
from std_msgs.msg import String


class ArmServer:

    def __init__(self):
        rospy.init_node("robot_controller", anonymous=True)

        self.publisher = rospy.Publisher('/arm_log', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/arm_controller', String, self.get_message_callback) 
        self.robot_controller = ControlRobot(1)
        self.is_runing = False 

    def get_message_callback(self, message: String) -> None:
        self.robot_controller.move_secuence(message.data) 
        self.publisher.publish("Secuencia ejecutada")

if __name__ == "__main__":
    node = ArmServer()
    rospy.spin()

    # try:
    #     node = ArmServer()
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass