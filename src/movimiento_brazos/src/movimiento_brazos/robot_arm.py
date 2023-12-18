from moveit_commander import MoveGroupCommander
from control_msgs.msg import GripperCommandActionGoal
import rospy
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from math import pi


class RobotArm:
    def __init__(self, robot_id: str, delay: int) -> None:
        self.move_group = MoveGroupCommander(robot_id)
        self.delay = delay

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
            time.sleep(self.delay)
