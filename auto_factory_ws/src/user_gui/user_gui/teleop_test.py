#!/usr/bin/env python

from concurrent.futures import ThreadPoolExecutor
from math import exp
import os
import select
import sys
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Timer
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import math
import numpy as np
import time
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

debug = True
task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian
path_time = 2.0  # second

r1 = 130
r2 = 124
r3 = 126


th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5*math.pi - th1_offset




# author : karl.kwon (mrthinks@gmail.com)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J0 to J2
def solv2(r1, r2, r3):
  d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
  d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

  s1 = math.acos(d1 / r1)
  s2 = math.acos(d2 / r2)

  return s1, s2

# author : karl.kwon (mrthinks@gmail.com)
# x, y, z : relational position from J0 (joint 0)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J2 to J3
# sr1 : angle between z-axis to J0->J1
# sr2 : angle between J0->J1 to J1->J2
# sr3 : angle between J1->J2 to J2->J3 (maybe always parallel)
def solv_robot_arm2(x, y, z, r1, r2, r3):
  Rt = math.sqrt(x**2 + y**2 + z**2)
  Rxy = math.sqrt(x**2 + y**2)
  St = math.asin(z / Rt)
#   Sxy = math.acos(x / Rxy)
  Sxy = math.atan2(y, x)

  s1, s2 = solv2(r1, r2, Rt)

  sr1 = math.pi/2 - (s1 + St)
  sr2 = s1 + s2
  sr2_ = sr1 + sr2
  sr3 = math.pi - sr2_

  J0 = (0, 0, 0)
  J1 = (J0[0] + r1 * math.sin(sr1)  * math.cos(Sxy),
        J0[1] + r1 * math.sin(sr1)  * math.sin(Sxy),
        J0[2] + r1 * math.cos(sr1))
  J2 = (J1[0] + r2 * math.sin(sr1 + sr2) * math.cos(Sxy),
        J1[1] + r2 * math.sin(sr1 + sr2) * math.sin(Sxy),
        J1[2] + r2 * math.cos(sr1 + sr2))
  J3 = (J2[0] + r3 * math.sin(sr1 + sr2 + sr3) * math.cos(Sxy),
        J2[1] + r3 * math.sin(sr1 + sr2 + sr3) * math.sin(Sxy),
        J2[2] + r3 * math.cos(sr1 + sr2 + sr3))

  return J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt



e = """
Communications Failed
"""


class TeleopKeyboardModi(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard_modi')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription
        
        # ROS2 구독자 설정
        self.position_subscription = self.create_subscription(
            PointStamped,
            'processed_positions',
            self.position_callback,
            10
        )

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()
        self.all_done = self.create_publisher(Bool,'last_order', 10)
        self.put_Box = self.create_publisher(Bool,'put_box', 10)
        # 물체 좌표 저장 리스트
        self.object_positions = []
        self.moving = False  # 이동 중 상태 플래그

    def position_callback(self, msg):
        """YOLO 좌표를 구독하고 리스트에 저장"""
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        target_position = (x, y, 70)
        self.object_positions.append(target_position)
        self.get_logger().info(
            f"감지된 물체 좌표 추가: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
    
    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[4]]
        self.tool_control_req.path_time = 5.0

        try:
            future = self.tool_control.call_async(self.tool_control_req)
            future.add_done_callback(self.tool_control_response)
        except Exception as e:
            self.get_logger().error(f"Service call to 'goal_tool_control' failed: {e}")

    def tool_control_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Tool control succeeded: {result}')
        except Exception as e:
            self.get_logger().error(f'Tool control failed: {e}')

    def move_to_positions(self):
        """object_positions 리스트에 저장된 좌표로 순서대로 이동"""
        if not self.object_positions:  # 리스트가 비어 있는 경우
            msga=Bool()
            msga.data = True
            self.all_done.publish(msga)
            self.get_logger().info("모든 작업이 완료되었습니다.")
            return
        
        if self.moving or not self.object_positions:
            return  # 이미 이동 중이거나 리스트가 비어 있는 경우 종료
        

        self.moving = True  # 이동 시작
        while self.object_positions:
            position = self.object_positions.pop(0)  # 리스트에서 첫 번째 좌표 꺼냄
            tutorial_x, tutorial_y, tutorial_z = position

            # 이동 계산
            try:


                ###받은 위치로 이동
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(
                    tutorial_x, tutorial_y, 60, r1, r2, r3)

                # 목표 관절 각도 설정
                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 3.0

                self.send_goal_joint_space()  # 이동 명령
                self.get_logger().info(
                    f"목표 좌표로 이동 중: X={tutorial_x}, Y={tutorial_y}, Z={tutorial_z}")
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기
                
                time.sleep(3)


                ####내리기
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(
                    tutorial_x-10, tutorial_y+10, 45, r1, r2, r3)

                # 목표 관절 각도 설정
                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 3.0

                self.send_goal_joint_space()  # 이동 명령
                self.get_logger().info(
                    f"목표 좌표로 이동 중: X={tutorial_x}, Y={tutorial_y}, Z={tutorial_z}")
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기


                time.sleep(2)



                ####그리퍼 닫아 큐브 잡기
                goal_joint_angle[4] = 0.01
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                self.send_tool_control_request()  # 이동 명령
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기



                time.sleep(2)

                
                ###받은 위치로 이동
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(
                    tutorial_x, tutorial_y, 60, r1, r2, r3)

                # 목표 관절 각도 설정
                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 3.0

                self.send_goal_joint_space()  # 이동 명령
                self.get_logger().info(
                    f"목표 좌표로 이동 중: X={tutorial_x}, Y={tutorial_y}, Z={tutorial_z}")
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기
                
                
                
                time.sleep(2)

                #### 들어올리기
                goal_joint_angle[0] = 0.78
                goal_joint_angle[1] = 0.0
                goal_joint_angle[2] = 0.0
                goal_joint_angle[3] = 1.57
                goal_joint_angle[4] = 0.0
                path_time = 3.0
                self.send_goal_joint_space()  # 이동 명령
                self.get_logger().info(
                    f"목표 좌표로 이동 중: X={tutorial_x}, Y={tutorial_y}, Z={tutorial_z}")
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기


                time.sleep(2)
                

                ####컨베이어에 놓기
                goal_joint_angle[0] = 1.575
                goal_joint_angle[1] = 1.75
                goal_joint_angle[2] = 0.0
                goal_joint_angle[3] = 0.0
                goal_joint_angle[4] = 0.0
                path_time = 3.0
                self.send_goal_joint_space()  # 이동 명령
                self.get_logger().info(
                    f"목표 좌표로 이동 중: X={tutorial_x}, Y={tutorial_y}, Z={tutorial_z}")
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기
                

                time.sleep(2)


                time.sleep(1)
                ####그리퍼 열기
                goal_joint_angle[4] = -0.01
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                self.send_tool_control_request()  # 이동 명령
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기


                time.sleep(2)


                #### 다시 들어올리기
                goal_joint_angle[0] = 0.78
                goal_joint_angle[1] = 0.0
                goal_joint_angle[2] = 0.0
                goal_joint_angle[3] = 1.57
                goal_joint_angle[4] = 0.0
                path_time = 3.0
                self.send_goal_joint_space()  # 이동 명령
                self.get_logger().info(
                    f"목표 좌표로 이동 중: X={tutorial_x}, Y={tutorial_y}, Z={tutorial_z}")
                rclpy.spin_once(self, timeout_sec=path_time)  # 이동 시간 동안 대기
                
                time.sleep(2)
                msg = Bool()
                msg.data = True
                self.put_Box.publish(msg)
            except Exception as e:
                self.get_logger().error(f"좌표 이동 중 오류 발생: {e}")

        self.moving = False  # 이동 완료

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [
            goal_joint_angle[0],
            goal_joint_angle[1] + th1_offset,
            goal_joint_angle[2] + th2_offset,
            goal_joint_angle[3],
            goal_joint_angle[4]
        ]
        self.goal_joint_space_req.path_time = 3.0

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[4]]
        self.tool_control_req.path_time = 5.0

        try:
            future = self.tool_control.call_async(self.tool_control_req)
            future.add_done_callback(self.tool_control_response)
        except Exception as e:
            self.get_logger().error(f"Service call to 'goal_tool_control' failed: {e}")

    def tool_control_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Tool control succeeded: {result}')
        except Exception as e:
            self.get_logger().error(f'Tool control failed: {e}')

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        # Ensure that there are at least 5 joint positions in the message
        if len(msg.position) >= 5:
            present_joint_angle[0] = msg.position[0]
            present_joint_angle[1] = msg.position[1] - th1_offset
            present_joint_angle[2] = msg.position[2] - th2_offset
            present_joint_angle[3] = msg.position[3]
            present_joint_angle[4] = msg.position[4]
        else:
            self.get_logger().warn('Received JointState message with insufficient joint positions.')

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = None
    tutorial_x = 150
    tutorial_y = 0.000
    tutorial_z = 150

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboardModi()        # 초기 위치로 이동
        goal_joint_angle[0] = 0.0
        goal_joint_angle[1] = -1.0723
        goal_joint_angle[2] = 1.5659
        goal_joint_angle[3] = 2.0003
        goal_joint_angle[4] = 0.0
        path_time = 3.0
        teleop_keyboard.send_goal_joint_space()
        teleop_keyboard.get_logger().info("초기 위치로 이동 중...")

        # 초기 이동 완료를 위해 잠시 대기
        rclpy.spin_once(teleop_keyboard, timeout_sec=path_time)
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin_once(teleop_keyboard)

            print('tutorial_position: [{:.3f}, {:.3f}, {:.3f}] - {:.3f}'.format(tutorial_x, tutorial_y, tutorial_z, math.atan2(tutorial_y, tutorial_x)))

            # object_positions에 좌표가 있으면 이동
            teleop_keyboard.move_to_positions()

            # 사용자 입력 처리
            key_value = get_key(settings)
            if key_value == '\x03':  # Ctrl+C로 종료
                break
            else:
                    for index in range(0, 7):
                        prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
                    for index in range(0, 5):
                        prev_goal_joint_angle[index] = goal_joint_angle[index]
                
    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
