#!/usr/bin/env python 

import rospy
from std_msgs.msg import String


class CubeSolverService:
    def __init__(self) -> None:
        rospy.init_node('nodo_publicador')

        self.publisher = rospy.Publisher('/cube_unsolved', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/cube_solved', String, self.get_message_callback)
        self.rate = rospy.Rate(1) 

        self.solution = None

    def get_cube_solution(self, cube: str) -> str:
        self.solution = None
        self.publisher.publish(cube)

        while self.solution is None:
            self.rate.sleep()

        if self.solution == "BAD CONFIG":
            return None

        return self.solution

    def get_message_callback(self, data: String) -> None:
        self.solution = data.data


if __name__ == "__main__":
    cube_service = CubeSolverService()

    solution_1 = cube_service.get_cube_solution(input("Ingrese el cubo 1: "))
    solution_2 = cube_service.get_cube_solution(input("Ingrese el cubo 2: "))

    print("Solution 1:", solution_1)
    print("Solution 2:", solution_2)