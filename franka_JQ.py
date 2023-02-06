from __future__ import print_function
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
import time
import math
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import rospy
from scipy.spatial.transform import Rotation as R
import numpy as np
import sys
import serial
from binascii import *
import crcmod
import json

ser = serial.Serial()

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


# END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def home(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0.00012409422975050778
        joint_goal[1] = -0.7851760762197928
        joint_goal[2] = -0.000308854746172659
        joint_goal[3] = -2.357726806912310
        joint_goal[4] = -0.00011798564528483742
        joint_goal[5] = 1.570464383098814
        joint_goal[6] = 0.7852387161304554
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def end(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = -0.2462336559787131
        joint_goal[1] = 0.5533778210810982
        joint_goal[2] = -0.570754861965514
        joint_goal[3] = -1.2474257309211134
        joint_goal[4] = 0.1992099539931909
        joint_goal[5] = 1.644834751637865
        joint_goal[6] = 0.9432959114237601
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)


def rpy_mat(rpy):
    num = [[0, 0, 0, rpy[0]], [0, 0, 0, rpy[1]],
           [0, 0, 0, rpy[2]], [0, 0, 0, 1]]
    rotation = rpy[3:6]
    r = R.from_euler('xyz', [rotation])
    rotation = r.as_dcm().reshape(-1)
    rotation = np.array(rotation)
    num = [[rotation[0], rotation[1], rotation[2], rpy[0]],
           [rotation[3], rotation[4], rotation[5], rpy[1]],
           [rotation[6], rotation[7], rotation[8], rpy[2]],
           [0, 0, 0, 1]]
    # print(num)
    num = np.array(num)
    return num


def transf(goal):
    mat = [goal[3], goal[4], goal[5], goal[6]]
    goal = rpy_mat(goal)
    grasper = [0, 0, -0.05, 0, 0, 0.25]
    grasper = np.array(grasper)
    grasper = rpy_mat(grasper)
    grasper = np.linalg.inv(grasper)
    pose = goal @ grasper
    goal = [pose[0][3]-0.05, pose[1][3], pose[2]
            [3], mat[0], mat[1], mat[2], mat[3]]
    return goal


def port_open():
    ser.port = "/dev/ttyUSB0"  # 设置端口号
    ser.baudrate = 115200  # 设置波特率
    ser.bytesize = 8  # 设置数据位
    ser.stopbits = 1  # 设置停止位
    ser.parity = "N"  # 设置校验位
    ser.open()  # 打开串口,要找到对的串口号才会成功
    if (ser.isOpen()):
        print("打开成功")
    else:
        print("打开失败")


def port_close():
    ser.close()
    if (ser.isOpen()):
        print("关闭失败")
    else:
        print("关闭成功")


def send(send_data):
    if (ser.isOpen()):
        ser.write(send_data)  # Hex发送
        print("发送成功", send_data)
    else:
        print("发送失败")


def crc16Add(read):
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    data = read.replace(" ", "")  # 消除空格
    readcrcout = hex(crc16(unhexlify(data))).upper()
    str_list = list(readcrcout)
    # print(str_list)
    if len(str_list) == 5:
        str_list.insert(2, '0')  # 位数不足补0，因为一般最少是5个
    crc_data = "".join(str_list)  # 用""把数组的每一位结合起来  组成新的字符串
    # print(crc_data)
    read = read.strip() + ' ' + crc_data[4:] + \
        ' ' + crc_data[2:4]  # 把源代码和crc校验码连接起来
    # print('CRC16校验:', crc_data[4:] + ' ' + crc_data[2:4])
    # print(read)
    return read


# 手掌开关数据帧 03EB 位置 速度 力  0-100 0-100  0-100
def SetFingerPosition(hand):
    for i in range(len(hand)):
        hand[i] = round(2.55*hand[i])  # 转为整数
        hand[i] = hex(hand[i])
        hand[i] = hand[i][2:]
        hand[i] = hand[i].upper()
        i += 1
    if int(hand[0], 16) < 10:
        hand[0] = "0"+hand[0]
    print(hand[0])
    hand = hand[0] + " " + hand[1] + " " + hand[2] + " " + "00"
    str = "09 10 03 EB 00 02 04 "
    hand = str + hand
    print(hand)
    hand = crc16Add(hand)
    hand = hand.split(' ')
    for i in range(len(hand)):
        hand[i] = '0x' + hand[i]
        hand[i] = hand[i].replace(" ", "")
        hand[i] = hand[i].replace(",", "")
        hand[i] = int(hand[i], 16)
        # hand[i] = hex(hand[i])
        i += 1
    return hand


# 手掌旋转数据帧 03ED
def SetHandRotation(revolve):
    for i in range(len(revolve)):
        revolve[i] = round(2.55*revolve[i])  # 转为整数
        revolve[i] = hex(revolve[i])
        revolve[i] = revolve[i][2:]
        revolve[i] = revolve[i].upper()
        i += 1
    if int(revolve[0], 16) < 10:
        revolve[0] = "0"+revolve[0]
    revolve = revolve[0] + " " + revolve[1] + " " + revolve[2] + " " + "00"
    str = "09 10 03 ED 00 02 04 "
    revolve = str + revolve
    revolve = crc16Add(revolve)
    revolve = revolve.split(' ')
    for i in range(len(revolve)):
        revolve[i] = '0x' + revolve[i]
        revolve[i] = revolve[i].replace(" ", "")
        revolve[i] = revolve[i].replace(",", "")
        revolve[i] = int(revolve[i], 16)
        # hand[i] = hex(hand[i])
        i += 1
    return revolve
# #左指开合数据帧 03EF


def LeftHand(left):
    for i in range(len(left)):
        left[i] = round(2.55*left[i])  # 转为整数
        left[i] = hex(left[i])
        left[i] = left[i][2:]
        left[i] = left[i].upper()
        i += 1
    if int(left[0], 16) < 10:
        left[0] = "0"+left[0]
    left = left[0] + " " + left[1] + " " + left[2] + " " + "00"
    str = "09 10 03 EF 00 02 04 "
    left = str + left
    left = crc16Add(left)
    left = left.split(' ')
    for i in range(len(left)):
        left[i] = '0x' + left[i]
        left[i] = left[i].replace(" ", "")
        left[i] = left[i].replace(",", "")
        left[i] = int(left[i], 16)
        # hand[i] = hex(hand[i])
        i += 1
    return left
# #右手开合数据帧 03F3


def RightHand(right):
    for i in range(len(right)):
        right[i] = round(2.55*right[i])  # 转为整数
        right[i] = hex(right[i])
        right[i] = right[i][2:]
        right[i] = right[i].upper()
        i += 1
    if int(right[0], 16) < 10:
        right[0] = "0"+right[0]
    right = right[0] + " " + right[1] + " " + right[2] + " " + "00"
    str = "09 10 03 F3 00 02 04 "
    right = str + right
    right = crc16Add(right)
    right = right.split(' ')
    for i in range(len(right)):
        right[i] = '0x' + right[i]
        right[i] = right[i].replace(" ", "")
        right[i] = right[i].replace(",", "")
        right[i] = int(right[i], 16)
        # hand[i] = hex(hand[i])
        i += 1
    return right
# #中指开合数据帧 03F1


def CenterHand(center):
    for i in range(len(center)):
        center[i] = round(2.55*center[i])  # 转为整数
        center[i] = hex(center[i])
        center[i] = center[i][2:]
        center[i] = center[i].upper()
        i += 1
    if int(center[0], 16) < 10:
        center[0] = "0"+center[0]
    center = center[0] + " " + center[1] + " " + center[2] + " " + "00"
    str = "09 10 03 F1 00 02 04 "
    center = str + center
    center = crc16Add(center)
    center = center.split(' ')
    for i in range(len(center)):
        center[i] = '0x' + center[i]
        center[i] = center[i].replace(" ", "")
        center[i] = center[i].replace(",", "")
        center[i] = int(center[i], 16)
        # hand[i] = hex(hand[i])
        i += 1
    return center


def HandOpen():
    print("Opening the hand")
    open = [0, 50, 43]
    open = SetFingerPosition(open)
    send(open)
    time.sleep(2)
    print("Opening the hand... Done...")


def HandClose():
    print("Closing the hand")
    close = [90, 50, 43]
    close = SetFingerPosition(close)
    send(close)
    time.sleep(2)
    print("Closing the hand... Done...")


def jq_init():
    # activation 09 06 03 E8 00 FF 48 B2
    act = [0x09, 0x06, 0x03, 0xE8, 0x00, 0xFF, 0x48, 0xB2]
    send(act)
    # sleeep(10)
    # # Read 09 03 07 D0 00 01 85 CF
    # rea = [0x09, 0x03, 0x07, 0xD0, 0x00, 0x01,0x85, 0xCF]
    # send(rea)


zero_position = [0, 50, 43]

if __name__ == "__main__":
    # parse JSON
    f = open('./diffgrasp_output/0008.json', 'r')
    content = f.read()
    a = json.loads(content)
    f.close()

    port_open()  # open the Modbus port

    jq_init()  # Initialize the hand
    time.sleep(5)

    # revolve= [100 - int(a['0'][2][1] * 100 / 3.1416) ,50,43]
    # # revolve=[100,50,43]
    # revolve=SetHandRotation(revolve)
    # send(revolve)
    # left = [int(a['0'][2][3] * 100 / 2.44) ,50,43]
    # # left = [0,50,43]
    # left = LeftHand(left)
    # send(left)
    # right = [int(a['0'][2][2] * 100 / 2.44) ,50,43]
    # right = RightHand(right)
    # send(right)
    # center = [int(a['0'][2][0] * 100 / 2.44) ,50,43]
    # center = CenterHand(center)
    # send(center)

    demo = MoveGroupPythonInterfaceTutorial()

    for x in a:
        if x[0] == []:
            continue
        quaternion = x[0]
        pose = x[1]
        grasp = x[2]
        print('---------四元数-wxyz----------')
        print(quaternion)
        print('--------pose-xyz---------------')
        print(pose)
        print('---------grasp-relove/left/center/right-----------')
        print(grasp)
        # port_open()
        # jq_init()

        # control the hand to the correct position
        revolve = [100 - int(grasp[1] * 100 / 3.1416), 50, 43]
        # revolve=[100,50,43]
        revolve = SetHandRotation(revolve)
        send(revolve)
        left = [int(grasp[3] * 100 / 2.44), 50, 43]
        # left = [0,50,43]
        left = LeftHand(left)
        send(left)
        right = [int(grasp[2] * 100 / 2.44), 50, 43]
        right = RightHand(right)
        send(right)
        center = [int(grasp[0] * 100 / 2.44), 50, 43]
        center = CenterHand(center)
        send(center)

        demo.home()

        # x y z rx ry rz w
        goal = np.array([pose[0], pose[1], pose[2], quaternion[1],
                        quaternion[2], quaternion[3], quaternion[0]])
        goal = transf(goal)
        goal[0] -= 0.11
        goal[2] -= 0
        # if goal[0] >= 0.68: continue
        demo.go_to_pose_goal(goal)

        HandClose()

        demo.home()

        demo.end()
        
        HandOpen()

        demo.home()
