from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandActionGoal
import rospy
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Duration
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from math import pi

import positions


class RobotArm:
    def __init__(self, robot_id: str) -> None:
        self.move_group = MoveGroupCommander(robot_id)

        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(5)

        self.publicador_pinza = rospy.Publisher(
            f"/{robot_id}/rg2_action_server/goal",
            GripperCommandActionGoal,
            queue_size=10,
        )

        self.action_client = SimpleActionClient(
            # f"/{robot_id}/sequence_move_group",
            f"/{robot_id}/scaled_pos_joint_traj_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

    def move_to_position(self, pos_values: list) -> bool:
        return self.move_group.go(pos_values)

    def move_clamp(self, anchura: float, fuerza: float) -> None:
        msg_pinza = GripperCommandActionGoal()
        msg_pinza.goal.command.position = anchura
        msg_pinza.goal.command.max_effort = fuerza

        self.publicador_pinza.publish(msg_pinza)

    def rotate_clamp(self, angle: float, time: float) -> bool:
        """_summary_

        Args:
            angle (float): Configuración ABSOLUTA de la última articulación en radianes.

        Returns:
            bool: _description_
        """
        conf_actual = self.move_group.get_current_state()
        punto_traj = JointTrajectoryPoint()
        punto_traj.positions = list(conf_actual.joint_state.position[12:17]) + [angle]
        punto_traj.time_from_start = rospy.Duration(secs=time)
        traj = JointTrajectory()
        traj.points.append(punto_traj)
        traj.joint_names = conf_actual.joint_state.name[12:18]
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj

        result = self.action_client.send_goal_and_wait(goal)

        if result == GoalStatus.SUCCEEDED:
            return True

        return False

    def execute_secuence(self, action_list):
        for action in action_list:
            if len(action) == 6:
                result = self.move_to_position(action)
            elif len(action) == 2:
                anchura, fuerza = action
                result = self.move_clamp(anchura, fuerza)
            else:
                angulo = action[0]
                result = self.rotate_clamp(angulo, 2)

            print(result)
            time.sleep(1)


class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("robot_controller", anonymous=True)
        rospy.sleep(2)

        self.left_arm = RobotArm("robot_205")
        self.right_arm = RobotArm("robot_206")

        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()

        self.pose_suelo = PoseStamped()
        self.pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        self.pose_suelo.pose.position.z = -0.011
        self.planning_scene.add_box("suelo", self.pose_suelo, (3, 3, 0.02))

    def recoger_cubo(self):
        self.left_arm.execute_secuence(
            [
                *positions.SOLTAR_RAPIDO_FLOJO,
                positions.I_RECOGER_P1,
                positions.I_RECOGER_P2,
                *positions.AGARRE_RAPIDO_FUERTE,
            ]
        )

        new_pose = self.pose_suelo
        new_pose.pose.position.x = 0.35
        new_pose.pose.position.y = 0.29
        new_pose.pose.position.z = 0.03501

        self.planning_scene.add_box("cubo", new_pose, (0.07, 0.07, 0.07))
        touch_links = self.robot_commander.get_link_names(group="gripper_205")
        self.planning_scene.attach_box("tool0_205", "cubo", touch_links=touch_links)

        self.left_arm.execute_secuence(
            [
                positions.I_RECOGER_P1,
                positions.I_RECOGER_P3,
                positions.I_RECOGER_P4,
            ]
        )

    def mover_abajo(self):
        self.right_arm.execute_secuence(
            [
                positions.D_REPOSO,
                positions.D_INFERIOR_APROXIMACION,
                positions.D_INFERIOR_AGARRE
            ]
        )

    def volver_abajo(self):
        self.right_arm.execute_secuence(
            [
                positions.D_INFERIOR_APROXIMACION,
                positions.D_REPOSO,
            ]
        )

    def mover_arriba(self):
        self.right_arm.execute_secuence(
            [
                positions.D_REPOSO,
                positions.D_SUPERIOR_APROXIMACION,
                positions.D_SUPERIOR_AGARRE,
            ]
        )

    def volver_arriba(self):
        self.right_arm.execute_secuence(
            [
                positions.D_SUPERIOR_APROXIMACION,
                positions.D_REPOSO,
            ]
        )


    def hacer_giro(self, is_inverted, is_double):
        angulo = pi if is_double else pi / 2
        angulo = angulo * is_inverted

        self.right_arm.execute_secuence(
            [
                *positions.AGARRE_RAPIDO_FLOJO,
                [angulo],
                *positions.SOLTAR_RAPIDO_FLOJO,
            ]
        )

    def rotar_caras(self, actual, posicion):
        angulo = (posicion - actual) * pi / 2

        self.right_arm.execute_secuence(positions.AGARRE_RAPIDO_FUERTE)
        time.sleep(1)

        self.left_arm.execute_secuence(positions.SOLTAR_RAPIDO_FLOJO)
        time.sleep(1)

        self.right_arm.execute_secuence([[angulo]])
        time.sleep(1)

        self.left_arm.execute_secuence(positions.AGARRE_RAPIDO_FLOJO)
        time.sleep(1)

        self.right_arm.execute_secuence(positions.SOLTAR_RAPIDO_FLOJO)

    def invertir_orientacion(self):
        self.right_arm.execute_secuence(positions.AGARRE_RAPIDO_FUERTE)
        time.sleep(1)

        self.left_arm.execute_secuence(positions.SOLTAR_RAPIDO_FLOJO)
        time.sleep(1)

        self.right_arm.rotate_clamp(pi / 2, 5)

        self.left_arm.execute_secuence(positions.AGARRE_RAPIDO_FUERTE)
        time.sleep(1)

        self.right_arm.execute_secuence(positions.SOLTAR_RAPIDO_FLOJO)
        time.sleep(1)

    def move_secuence(self, solve_path):
        orientacion = 1
        actual_pos = 0

        if (
            self.right_arm.move_group.get_current_joint_values()
            != positions.I_RECOGER_P4
        ):
            self.recoger_cubo()

        self.right_arm.execute_secuence([positions.D_REPOSO])

        for index in solve_path.split(" "):
            index = input("Inserte la jugada: ")

            position = index[0]
            direction = -1 if index.find("'") > -1 else 1
            is_double = index.find("2") > -1

            print(f"{position=} {direction=} {is_double=}")

            if position in ["L", "F", "R", "B"]:
                position_index = ["L", "F", "R", "B"].index(position)

                if position_index != actual_pos:
                    self.mover_abajo()
                    self.rotar_caras(position_index, actual_pos)
                    self.volver_abajo()
                    actual_pos = actual_pos

                self.mover_arriba()
                self.hacer_giro(direction * orientacion, is_double)
                self.volver_arriba()

            if position in ["D", "U"]:
                position_index = 1 if ["D", "U"].index(position) == 0 else -1

                if position_index != orientacion:
                    self.mover_arriba()
                    self.invertir_orientacion()
                    self.volver_arriba()
                    orientacion = position_index

                self.mover_abajo()
                self.hacer_giro(direction, is_double)
                self.volver_abajo()


if __name__ == "__main__":
    control_robot = ControlRobot()
    control_robot.move_secuence("D2 R' D' F2 B D R2 D2 R' F2 D' F2 U' B2 L2 U2 D R2 U")
    # print("Robot izquierda: ")
    # print(control_robot.left_arm.move_group.get_current_joint_values())
    # print("Robot derecha: ")
    # print(control_robot.right_arm.move_group.get_current_joint_values())
