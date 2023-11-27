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
            f"/{robot_id}/sequence_move_group",
            # f"/{robot_id}/scaled_pos_joint_traj_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
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
        punto_traj.positions = list(conf_actual.joint_state.position[12:17])+[angle]
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
                result = self.rotate_clamp(angulo, 3)
            
            print(result)
            time.sleep(2)
 

class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("robot_controller", anonymous=True)
        rospy.sleep(2)
 
        self.left_arm = RobotArm("robot_205")
        self.right_arm = RobotArm("robot_206")
 
        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()
 
        pose_suelo = PoseStamped()
        pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        pose_suelo.pose.position.z = -0.011
        self.planning_scene.add_box("suelo", pose_suelo, (3, 3, 0.02))

    def recoger_cubo(self):
        self.left_arm.execute_secuence(
            [
                [100, 3],
                [0.40443098545074463, -0.9778804940036316, 0.8503282705890101, -1.4449669879725953, -1.564664665852682, 0.357255756855011],
                [0.404244065284729, -0.9792840641788025, 1.090346638356344, -1.6836816273131312, -1.5647137800799769, 0.358335018157959],
                [30, 40],
                [0.40443098545074463, -0.9778804940036316, 0.8503282705890101, -1.4449669879725953, -1.564664665852682, 0.357255756855011],
                [0.46085554361343384, -0.7338349980166932, 1.6405442396747034, -4.082707067529196, -2.000608269368307, 0.006899356842041016],
                [0.44173359870910645, -0.27442999303851323, 0.6156037489520472, -3.466997285882467, -2.019162956868307, 4.89973826915957e-05]
            ]
        )

    def mover_abajo(self):
        self.right_arm.execute_secuence(
            [
                [1.058468222618103, -1.8820544681944789, 0.8497050444232386, -0.5394669336131592, -1.5684707800494593, -2.1022632757769983],
                [0.06274263560771942, -1.2139890950969239, 0.6285613218890589, -0.9864318531802674, -1.5671222845660608, -3.009000841771261],
                [0.47338834404945374, -0.6877903503230591, 0.6417611281024378, 0.039393706912658644, -1.105957333241598, -3.1479156653033655],
                [0.5036637187004089, -0.7917526525310059, 0.8734982649432581, -0.0368078512004395, -1.0687573591815394, 1.9073486328125e-06],
            ]
        )

    def mover_arriba(self):
        self.right_arm.execute_secuence(
            [
                [1.058468222618103, -1.8820544681944789, 0.8497050444232386, -0.5394669336131592, -1.5684707800494593, -2.1022632757769983],
                [1.3109047412872314, -1.2469163698009034, 0.204935375844137, -0.5298386377147217, -1.56775409380068, -1.8200209776507776],
                [1.313001275062561, -1.337524102335312, 0.4287937323199671, -0.6633060735515137, -1.5677087942706507, -1.817707363759176],
            ]
        )

    def volver_centro_arriba(self):
        self.right_arm.execute_secuence(
            [
                [1.3109047412872314, -1.2469163698009034, 0.204935375844137, -0.5298386377147217, -1.56775409380068, -1.8200209776507776],
                [1.058468222618103, -1.8820544681944789, 0.8497050444232386, -0.5394669336131592, -1.5684707800494593, -2.1022632757769983],
            ]
        )

    def volver_centro_abajo(self):
        self.right_arm.execute_secuence(
            [
                [0.47338834404945374, -0.6877903503230591, 0.6417611281024378, 0.039393706912658644, -1.105957333241598, -3.1479156653033655],
                [0.06274263560771942, -1.2139890950969239, 0.6285613218890589, -0.9864318531802674, -1.5671222845660608, -3.009000841771261],
                [1.058468222618103, -1.8820544681944789, 0.8497050444232386, -0.5394669336131592, -1.5684707800494593, -2.1022632757769983],
            ]
        )

    def hacer_giro(self, is_inverted, is_double):
        angulo = pi if is_double else pi/2
        angulo = angulo * is_inverted

        self.right_arm.execute_secuence(
            [
                [30, 40],
                [angulo],
                [100, 3]
            ]
        )

    def rotar_caras(self, actual, posicion):
        angulo = (posicion - actual) * pi/2
    
        self.left_arm.execute_secuence([[100, 3]])
    
        self.right_arm.execute_secuence(
            [
                [30, 40],
                [angulo],
                [100, 3]
            ]
        ) 
    
        self.left_arm.execute_secuence([[30, 40]])

    def invertir_orientacion(self):
        self.right_arm.execute_secuence(
            [
                [30, 40],
                [pi],
                [100, 3] 
            ]
        )

 
    def move_secuence(self, solve_path):
        orientacion = 1
        actual_pos = 0

        self.recoger_cubo()

        for i in solve_path.split(" "):
            position = i[0]
            is_inverted = int(i.find("'") > -1)
            is_double = i.find("2") > -1
        
            print(f"{position=} {is_inverted=} {is_double=}")
        
            if position in ["L", "F", "R", "B"]:
                position_index = ["L", "F", "R", "B"].index(position)
                
                if position_index != actual_pos:
                        self.mover_abajo()
                        self.rotar_caras(position_index, actual_pos)
                        actual_pos = actual_pos
                        self.volver_centro_abajo()
        
                self.mover_arriba()
                self.hacer_giro(is_inverted * orientacion, is_double)
                self.volver_centro_arriba()
        

            if position in ["U", "D"]:
                position_index = 1 if ["U", "D"].index(position) == 0 else -1

                if position_index != orientacion:
                        self.mover_arriba()
                        self.invertir_orientacion()
                        self.volver_centro_arriba()
                        orientacion = position_index
        
                self.mover_abajo()  
                self.hacer_giro(is_inverted, is_double)
                self.volver_centro_abajo()
    



if __name__ == '__main__':
    control_robot = ControlRobot()
    control_robot.move_secuence("D2 R' D' F2 B D R2 D2 R' F2 D' F2 U' B2 L2 U2 D R2 U")
