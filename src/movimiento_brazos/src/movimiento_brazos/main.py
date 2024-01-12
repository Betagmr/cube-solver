from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
import rospy
import time
from math import pi

import positions
from robot_arm import RobotArm



class ControlRobot:
    def __init__(self, vel: float=0.1) -> None:
        rospy.sleep(2)

        self.delay = 1

        self.left_arm = RobotArm("robot_205", self.delay)
        self.right_arm = RobotArm("robot_206", self.delay)

        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()

        self.pose_suelo = PoseStamped()
        self.pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        self.pose_suelo.pose.position.z = -0.011
        self.planning_scene.remove_world_object("cubo")
        self.planning_scene.add_box("suelo", self.pose_suelo, (3, 3, 0.02))

        self.left_arm.move_group.set_max_acceleration_scaling_factor(vel if vel <=1.0 and vel >=0.0 else 0.1)
        self.left_arm.move_group.set_max_velocity_scaling_factor(vel if vel <=1.0 and vel >=0.0 else 0.1)

        self.right_arm.move_group.set_max_acceleration_scaling_factor(vel if vel <=1.0 and vel >=0.0 else 0.1)
        self.right_arm.move_group.set_max_velocity_scaling_factor(vel if vel <=1.0 and vel >=0.0 else 0.1)

    def recoger_cubo(self):
        self.left_arm.execute_secuence(positions.APROXIMAR_CUBO)

        new_pose = self.pose_suelo
        new_pose.pose.position.x = 0.35
        new_pose.pose.position.y = 0.29
        new_pose.pose.position.z = 0.03501

        self.planning_scene.add_box("cubo", new_pose, (0.07, 0.07, 0.07))
        touch_links = self.robot_commander.get_link_names(group="gripper_205")
        self.planning_scene.attach_box("tool0_205", "cubo", touch_links=touch_links)

        self.left_arm.execute_secuence(positions.COLOCAR_CUBO)

    def dejar_cubo(self):
        self.left_arm.execute_secuence(positions.COLOCAR_CUBO[::-1])
        self.planning_scene.remove_attached_object("cubo")
        self.planning_scene.remove_world_object("cubo")
        self.left_arm.execute_secuence(positions.APROXIMAR_CUBO[::-1])


    def mover_abajo(self):
        self.right_arm.execute_secuence(positions.MOVER_ABAJO)

    def volver_abajo(self):
        self.right_arm.execute_secuence(positions.VOLVER_ABAJO)

    def mover_arriba(self):
        self.right_arm.execute_secuence(positions.MOVER_ARRIBA)

    def volver_arriba(self):
        self.right_arm.execute_secuence(positions.VOLVER_ARRIBA)


    def hacer_giro(self, is_inverted, is_double, ofset = 0):
        angulo = pi if is_double else pi / 2
        angulo = angulo * is_inverted

        self.right_arm.execute_secuence(
            [
                *positions.AGARAR_SIN_COLISION,
                [angulo + ofset],
                *positions.SOLTAR_SIN_COLISION,
            ]
        )

    def rotar_caras(self, angulo, tiempo):
        self.right_arm.execute_secuence(positions.AGARRE_RAPIDO_FUERTE)
        time.sleep(self.delay)

        self.left_arm.execute_secuence(positions.SOLTAR_RAPIDO_FLOJO)
        time.sleep(self.delay)

        self.right_arm.rotate_clamp(angulo, tiempo)
        time.sleep(self.delay)

        self.left_arm.execute_secuence(positions.AGARRE_RAPIDO_FUERTE)
        time.sleep(self.delay)

        self.right_arm.execute_secuence(positions.SOLTAR_RAPIDO_FLOJO)
        time.sleep(self.delay)

    def move_secuence(self, solve_path):
        orientation = 1
        actual_pos = 0

        if (
            self.right_arm.move_group.get_current_joint_values()
            != positions.I_RECOGER_P4
        ):
            self.recoger_cubo()

        self.right_arm.execute_secuence([positions.D_REPOSO])

        for index in solve_path.split(" "):
            # index = input("Inserte la jugada: ")

            position = index[0]
            direction = -1 if index.find("'") > -1 else 1
            is_double = index.find("2") > -1

            print(f"{position=} {direction=} {is_double=}")

            if position == "x":
                self.dejar_cubo()

            if position in ["L", "F", "R", "B"]:
                position_index = ["L", "F", "R", "B"].index(position)

                if position_index != actual_pos:
                    self.mover_abajo()
                    result = (position_index - actual_pos) % 4
                    rotacion = 1 if result == 3 else -result 

                    angulo = rotacion * orientation * pi / 2
                    self.rotar_caras(angulo, 5)
                    self.volver_abajo()
                    actual_pos = position_index

                self.mover_arriba()
                self.hacer_giro(direction, is_double, 0.12)
                self.volver_arriba()

            if position in ["D", "U"]:
                position_index = 1 if ["D", "U"].index(position) == 0 else -1

                if position_index != orientation:
                    self.mover_arriba()
                    self.rotar_caras(pi, 5)
                    self.volver_arriba()
                    orientation = position_index

                self.mover_abajo()
                self.hacer_giro(direction, is_double)
                self.volver_abajo()

            if position == "r":
                print("Las posiciones de la derecha son: ")
                print(self.right_arm.move_group.get_current_joint_values())

            if position == "l":
                print("Las posiciones de la izquierda son: ")
                print(self.left_arm.move_group.get_current_joint_values())

if __name__ == "__main__":
    control_robot = ControlRobot(1.0)
    
    control_robot.move_secuence("U R2 F L U D2 B B' D2 U' L' F' R2 U'")
    # print("Robot izquierda: ")
    # print(control_robot.left_arm.move_group.get_current_joint_values())
    # print("Robot derecha: ")
    # print(control_robot.right_arm.move_group.get_current_joint_values())
