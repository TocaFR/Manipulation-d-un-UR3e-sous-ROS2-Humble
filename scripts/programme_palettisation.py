#!/usr/bin/env python3
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner

# This is an example of how to interface the robot without any additional ROS components. For
# real-life applications, we do recommend to use something like MoveIt!

import time
import tty
import sys
import termios
import subprocess
from typing import List
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray

from builtin_interfaces.msg import Duration as ROSDuration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


orig_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)
pi = 3.14159

def deg_to_rad(deg):
    s = deg*(pi/180)
    return s

class Duration:
    def __init__(self, sec: int, nanosec: int):
        self.sec = sec
        self.nanosec = nanosec

class Position:
    def __init__(self, joint_angles: List[float]):
        self.joint_angles = [deg_to_rad(angle) for angle in joint_angles]  # Convertir les angles en radians

    def __repr__(self):
        return f"Position({self.joint_angles})"
    
class Trajectoire:
    def __init__(self, nom: str):
        self.nom = nom
        self.points = []

    def ajouter_point(self, positions: Position, velocities: List[float], time_from_start: Duration, gripper_status: bool):
        point = {
            "positions": positions,
            "velocities": velocities,
            "time_from_start": time_from_start,
            "gripper_status": gripper_status,
        }
        self.points.append(point)

    def __repr__(self):
        return f"Trajectoire(nom={self.nom}, points={self.points})"

def charger_trajectoires(fichier_xml: str) -> dict:
    tree = ET.parse(fichier_xml)
    root = tree.getroot()

    trajectoires = {}

    for traj_elem in root.findall('trajectory'):
        nom = traj_elem.get('name')
        trajectoire = Trajectoire(nom)

        for point_elem in traj_elem.findall('point'):
            # Récupérer les positions et les convertir
            positions = [float(pos_elem.get('value')) for pos_elem in point_elem.find('positions').findall('position')]
            
            # Récupérer les vitesses
            velocities = [float(vel_elem.get('value')) for vel_elem in point_elem.find('velocities').findall('velocity')]
            
            # Récupérer le temps
            time_elem = point_elem.find('time_from_start')
            time_from_start = Duration(sec=float(time_elem.get('sec')), nanosec=float(time_elem.get('nanosec')))
            
            # Récupérer l'état du gripper
            gripper_status = point_elem.find('gripper_status').text.lower() == 'true'
            
            # Ajouter le point à la trajectoire
            trajectoire.ajouter_point(
                positions=Position(positions),
                velocities=velocities,
                time_from_start=time_from_start,
                gripper_status=gripper_status
            )

        trajectoires[nom] = trajectoire

    return trajectoires

TRAJECTORIES = charger_trajectoires("trajectoires.xml")


class JTCClient(rclpy.node.Node):
    """Small test client for the jtc."""

    def __init__(self, num_cycles=1):
        super().__init__("jtc_client")
        self.num_cycles = num_cycles
        self.current_cycle = 0
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        
        self.parse_trajectories()
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        self.execute_next_trajectory()

        self.create_subscription(
            Float64MultiArray,
            '/move_robot',
            self.move_robot_callback,
            10
        )

    def gripper_state_change(self, state):
        if state == False:
            #self.get_logger().info("Opening gripper...")
            subprocess.run(["python3", "gripper_control.py", "open"])
        else:
            #self.get_logger().info("Closing gripper...")
            subprocess.run(["python3", "gripper_control.py", "close"])

    def parse_trajectories(self):
        self.goals = {}
        for traj_name, trajectoire in TRAJECTORIES.items():
            goal = JointTrajectory()
            goal.joint_names = self.joints

            # Iterate over the points in the Trajectoire object
            for pt in trajectoire.points:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"].joint_angles  # Access joint angles from the Position object
                point.velocities = pt["velocities"]
                point.time_from_start = ROSDuration(
                    sec=int(pt["time_from_start"].sec),
                    nanosec=int(pt["time_from_start"].nanosec),
                )
                goal.points.append(point)

            self.goals[traj_name] = goal

    def move_robot_callback(self, msg):
        """
        Cette fonction sera appelée lorsqu'un message avec les coordonnées est reçu.
        Elle prend les coordonnées dans msg.data et génère une nouvelle trajectoire.
        """
        # Récupérer les positions envoyées (en radians)
        positions = msg.data
        if len(positions) != len(self.joints):
            self.get_logger().error(f"Le nombre de positions ne correspond pas aux joints du robot.")
            return

        # Créer une nouvelle trajectoire avec ces positions
        point = JointTrajectoryPoint()
        point.positions = positions  # Remplacer les positions des joints
        point.velocities = [0.0] * len(self.joints)  # Vitesse nulle par défaut
        point.time_from_start = ROSDuration(sec=2, nanosec=0)  # Temps d'exécution de la trajectoire

        # Créer un objet JointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joints
        goal.trajectory.points.append(point)

        # Envoyer la commande au contrôleur
        self.get_logger().info(f"Exécution de la trajectoire avec les nouvelles positions : {positions}")
        self.execute_trajectory(goal)        
        

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.current_cycle += 1
            if self.current_cycle >= self.num_cycles:
                self.get_logger().info("Done with all trajectories and cycles")
                raise SystemExit
            self.i = 0 
            self.get_logger().info(f"Starting cycle {self.current_cycle + 1} of {self.num_cycles}")

        traj_name = list(self.goals)[self.i]
        self.i += 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        trajectoire = TRAJECTORIES.get(traj_name)
        if not trajectoire:
            self.get_logger().error(f"Trajectory {traj_name} not found.")
            return

        for point in trajectoire.points:
            self.gripper_state_change(point["gripper_status"])

        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]
        goal.goal_time_tolerance = ROSDuration(sec=0, nanosec=500000000)  # Use ROS Duration here
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=joint)
            for joint in self.joints
        ]

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(2)
            self.execute_next_trajectory()
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Done with result: {self.error_code_to_str(result.error_code)}"
                )
            raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def main(args=None):
    rclpy.init(args=args)
    try:
        # Prompt user for the number of cycles
        num_cycles = int(input("Enter the number of cycles to execute: "))
        
        jtc_client = JTCClient(num_cycles=num_cycles)
        rclpy.spin(jtc_client)
    except RuntimeError as err:
        rclpy.logging.get_logger("jtc_client").error(str(err))
    except KeyboardInterrupt:
        rclpy.logging.get_logger("jtc_client").info("Shutting down gracefully.")
    finally:
        rclpy.shutdown()

        

if __name__ == "__main__":
        main()
