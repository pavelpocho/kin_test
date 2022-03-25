#! /usr/bin/env python

from mimetypes import init
from re import I
import rospy
import math
from robotics_cswk_kin.srv import IKinMsg, IKinMsgRequest
from geometry_msgs.msg import Point

from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

from cube_locator.msg import RealCubeArray, RealCube

rospy.init_node("ikintest")
proxy = rospy.ServiceProxy("/inv_kin", IKinMsg)
rospy.wait_for_service("/inv_kin")

i = 0

scanned = False
executed = False
initialized = False
done = False
measure = False

x_red = []
y_red = []
z_red = []

x_yellow = []
y_yellow = []
z_yellow = []

def real_cube_data_handler(msg):

    global i 
    i += 1
    if i % 1 != 0:
        return

    global x
    global y
    global z

    global scanned
    global executed
    global initialized
    global done
    global measure

    if not initialized or done or not measure:
        return

    for c in msg.cubes:
        if c.color.data == "yellow":
            x_yellow.append(c.position.x)
            y_yellow.append(c.position.y)
            z_yellow.append(c.position.z)
        elif c.color.data == "red":
            print("adding to red")
            x_red.append(c.position.x)
            y_red.append(c.position.y)
            z_red.append(c.position.z)

    # if len(x) > 0 and len(msg.cubes) > 0:
    #     rospy.loginfo(sum(x) / len(x))
    #     rospy.loginfo(sum(y) / len(y))
    #     rospy.loginfo(sum(z) / len(z))

def move_to_position(x, y, z, angle, time):
    request = IKinMsgRequest()
    request.position = Point()
    request.position.x = x
    request.position.y = y
    request.position.z = z
    request.angle.data = angle

    response = proxy(request)
    r = [x.data for x in response.joint_positions]
    print(r)

    if not response.success.data:
        print('Failed inv kin')
    else:
        setPose = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

        rospy.wait_for_service('/goal_joint_space_path')

        jointRequest=JointPosition()
        jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
        jointRequest.position=[x.data for x in response.joint_positions]
        setPose(str(), jointRequest, time)

    r = rospy.Rate(1 / (time * 1.3))
    r.sleep()

def open_gripper():
    # Open the gripper
    gripperRequest=JointPosition()
    gripperRequest.joint_name=["gripper"] 
    gripperRequest.position=[0.01]# 0.01 represents open
    setGripper(str(),gripperRequest,1.0)

    rospy.sleep(1) # Wait for the gripper to open

def close_gripper():
    # Close the gripper
    gripperRequest=JointPosition()
    gripperRequest.joint_name=["gripper"]  
    gripperRequest.position=[-0.0065]# -0.01 represents closed
    setGripper(str(),gripperRequest,1.5)

    rospy.sleep(1) # Wait for the gripper to close

real_cubes_sub = rospy.Subscriber('/real_cubes', RealCubeArray, real_cube_data_handler)
setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)

move_to_position(0.15, 0, 0.1, math.pi / 4, 1.7)

initialized = True

for k in range(2):
    if len(x_yellow) > 1 and len(x_red) > 1:
        break
    print("Going for position", k)
    angle = math.pi / 2
    if k > 0:
        angle = math.pi / 4
    move_to_position(0.05 + 0.08 * k, 0, 0.1, angle, 1.0)
    s = rospy.Rate(3)
    measure = True
    s.sleep()
    measure = False

scanned = True

while not (scanned and not executed and len(x_yellow) > 0 and len(x_red) > 0):
    sl = rospy.Rate(1)
    sl.sleep()

executed = True
angle = math.pi / 12
z = -0.04
if math.sqrt((sum(x_yellow) / len(x_yellow)) ** 2 + (sum(y_yellow) / len(y_yellow)) ** 2) < 0.2:
    angle = math.pi / 2
    z = -0.05
elif math.sqrt((sum(x_yellow) / len(x_yellow)) ** 2 + (sum(y_yellow) / len(y_yellow)) ** 2) < 0.3:
    angle = math.pi / 4
    z = -0.045
move_to_position(sum(x_yellow) / len(x_yellow), sum(y_yellow) / len(y_yellow), z + 0.05, angle, 2.0)
move_to_position(sum(x_yellow) / len(x_yellow), sum(y_yellow) / len(y_yellow), z, angle, 2.0)
close_gripper()
move_to_position(sum(x_yellow) / len(x_yellow), sum(y_yellow) / len(y_yellow), z + 0.1, angle, 2.0)
move_to_position(sum(x_red) / len(x_red) - 0.005, sum(y_red) / len(y_red), z + 0.1, angle, 2.0)
move_to_position(sum(x_red) / len(x_red) - 0.005, sum(y_red) / len(y_red), z + 0.04, angle, 2.0)
open_gripper()

move_to_position(0.15, 0, 0.1, math.pi / 4, 3.0)

done = True
