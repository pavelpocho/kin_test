#! /usr/bin/env python

import rospy
import math
from robotics_cswk_kin.srv import IKinMsg, IKinMsgRequest
from geometry_msgs.msg import Point

from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

from cube_locator.msg import RealCubeArray, RealCube

CUBE_POSITIONS_TOPIC_NAME = '/real_cubes'

class CubeInfo:

    def __init__(self, point, id, color):
        self.id = id
        self.point = point
        self.alpha_correct = False
        self.r_correct = False
        self.color = color

    def alpha_corrected(self):
        self.alpha_correct = True
    
    def r_corrected(self):
        self.r_correct = True

class DistanceZone:

    def __init__(self, r_min, r_max, beta_min, beta_max):
        self.r_min = r_min
        self.r_max = r_max
        self.beta_min = beta_min
        self.beta_max = beta_max

    def get_mid_beta(self):
        return (self.beta_min + self.beta_max) / 2

    def get_mid_r(self):
        return (self.r_min + self.r_max) / 2


class MainProgram:

    def __init__(self):

        # -1. Setup everything that's needed
        self.sequence_complete = False
        self.searching = False
        # colors excluded from search
        self.excluded_colors = []
        self.move_time = 1.4
        self.search_rate = 3
        self.search_z = 0.1

        self.inv_kin_proxy
        self.set_pose_proxy

        self.yellow_cube_locations = []
        self.red_cube_locations = []
        self.blue_cube_locations = []

        self.cube_infos = []

        self.setup()

        # 0. Specify distance zones and approach angles
        self.distance_zones = []
        self.specify_distance_zones()

        # 1. Sub to cube information and read it
        self.cube_position_subscriber
        self.sub_to_cube_info()

        # 2. Search (move around), detect cube positions and average them
        self.search()

        # 3. We now have a map of cubes
        # 4. Assess the situation
        self.assess_situation()

        pass

    def setup(self):
        # Initialize inverse kinematics service proxy and wait for the service
        self.inv_kin_proxy = rospy.ServiceProxy("/inv_kin", IKinMsg)
        rospy.wait_for_service("/inv_kin")
        self.set_pose_proxy = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        rospy.wait_for_service('/goal_joint_space_path')
        self.set_gripper = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        rospy.wait_for_service('/goal_tool_control')

    def specify_distance_zones(self):
        for i in range(10):
            # edit this when you test it on the robot to make it as fast as possible
            # for now it starts at six centimeters, we'll see how close we can reach
            self.distance_zones.append(DistanceZone(
                3 * (i + 2) / 100,              # every zone is 3 centimeters
                3 * (i + 3) / 100,              # it ends after the 3 centimeters
                90 - (i + 2) * 6),              # say the range is 12 degrees in those 3 centimeters
                90 - i * 6                      # there will always be a 6 degree overlap
            )

    def sub_to_cube_info(self):
        self.cube_position_subscriber = rospy.Subscriber(CUBE_POSITIONS_TOPIC_NAME, RealCubeArray, self.cube_info_handler)

    def cube_info_handler(self, msg):
        if self.sequence_complete or not self.searching:
            return

        for cube in msg.cubes:
            if cube.color.data == 'yellow' and not 'yellow' in self.excluded_colors:
                self.yellow_cube_locations.append(cube.position)
            elif cube.color.data == 'red' and not 'red' in self.excluded_colors:
                self.red_cube_locations.append(cube.position)
            elif cube.color.data == 'blue' and not 'blue' in self.excluded_colors:
                self.blue_cube_locations.append(cube.position)

    def move_to_position(self, x, y, z, beta):
        request = IKinMsgRequest()
        request.position = Point()
        request.position.x = x
        request.position.y = y
        request.position.z = z
        request.angle.data = beta

        response = self.inv_kin_proxy(request)
        r = [x.data for x in response.joint_positions]
        print(r)

        if not response.success.data:
            print('Failed inverse kinematics. Where the heck did you send it?')
        else:
            jointRequest=JointPosition()
            jointRequest.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']  
            jointRequest.position = [x.data for x in response.joint_positions]
            self.set_pose_proxy(str(), jointRequest, self.move_time)
            r = rospy.Rate(1 / (self.move_time * 1.3))
            r.sleep()

    def move_to_position_by_r_and_alpha(self, r, alpha, z, beta):
        return self.move_to_position(r * math.cos(alpha), r * math.sin(alpha), z, beta)

    def search(self, excluded_colors = []):
        # Move to initial position
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)

        # Always run through all the positions so as to spot
        # all the cubes that could possibly be somewhere
        for zone_index in range(3):
            for alpha_index in range(3):
                # scan every third zone for now, if they are
                # e.g. bigger, change this
                r = self.distance_zones[zone_index * 3].get_mid_r()
                alpha = (alpha_index - 1) * 20
                beta = self.distance_zones[zone_index * 3].get_mid_beta()
                
                self.move_to_position_by_r_and_alpha(r, alpha, self.search_z, beta)
                s = rospy.Rate(self.search_rate)
                self.searching = True
                self.excluded_colors = excluded_colors
                s.sleep()
                self.searching = False
                self.excluded_colors = []

    def get_average(self, numbers):
        return sum(numbers) / len(numbers)

    def add_final_cube_location(self, locations, id, color):
        p = Point()
        p.x = self.get_average(map(lambda loc: loc.x, locations))
        p.y = self.get_average(map(lambda loc: loc.y, locations))
        p.z = self.get_average(map(lambda loc: loc.z, locations))
        self.cube_infos.append(CubeInfo(p, id, color))

    def get_r(self, point):
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def get_alpha(self, point):
        return -math.asin(point.y / self.get_r(point))

    def assess_situation(self):
        # Step 1: Check how many cubes we have
        if len(self.yellow_cube_locations) > 10:
            self.add_final_cube_location(self.yellow_cube_locations, 1, 'yellow')
        if len(self.red_cube_locations) > 10:
            self.add_final_cube_location(self.red_cube_locations, 2, 'red')
        if len(self.blue_cube_locations) > 10:
            self.add_final_cube_location(self.blue_cube_locations, 3, 'blue')
        
        # The sequence is now as follows:
        # 1. Order them by r distance
        self.cube_infos.sort(lambda c: self.get_r(c.point))

        # 2a. Always move in to a smaller r position and approach from the back
        # 2b. Move each to its own alpha section
        alpha_sections = [
            [-math.pi/4, -math.pi/12],
            [-math.pi/12, math.pi/12],
            [math.pi/12, math.pi/4]
        ]
        colors_to_exclude = []
        for cube_info in self.cube_infos:
            free_section = self.find_empty_alpha_section_for_cube(cube_info)
            self.move_cube_on_alpha(cube_info, self.get_average(free_section))
            # Have to look through all of the other cubes again here
            colors_to_exclude.append(cube_info.color)
            self.search(colors_to_exclude)

        # 3. Move each to the right r section
        
        # 4. Stack

    def find_empty_alpha_section_for_cube(self, cube_info, alpha_sections):
        for section in alpha_sections:
            section_full = False
            for other_cube_info in self.cube_infos:
                if cube_info.id == other_cube_info.id:
                    continue
                if section[0] < self.get_alpha(other_cube_info.point) < section[1]:
                    section_full = True
            
            if not section_full:
                return section
                    

    def open_gripper(self):
        # Open the gripper
        grip_request = JointPosition()
        grip_request.joint_name = ["gripper"] 
        grip_request.position = [0.01] # 0.01 represents open
        self.set_gripper(str(), grip_request, 1.0)
        rospy.sleep(1) # Wait for the gripper to open

    def close_gripper(self):
        # Close the gripper
        grip_request = JointPosition()
        grip_request.joint_name = ["gripper"]  
        grip_request.position = [-0.0065] # -0.01 represents closed
        self.set_gripper(str(), grip_request, 1.0)
        rospy.sleep(1) # Wait for the gripper to close

    def grab_at_coords(self, r, alpha):
        # TODO: Change this beta value according to r-zone!
        beta = math.pi / 2

        # Move 6 cm in front of the cube (where we know there is empty space)
        # Note: This does decrease the available workspace
        self.move_to_position_by_r_and_alpha(r - 0.06, alpha, -0.04, beta)
        # Move to cube, grip it and get out (directly up)
        self.move_to_position_by_r_and_alpha(r, alpha, -0.04, beta)
        self.close_gripper()
        # TODO: This might limit the space. If too far away, make it go back as well or something
        self.move_to_position_by_r_and_alpha(r, alpha, -0.04 + 0.01, beta)

    def place_at_coords(self, r, alpha, stacking = False):
        # TODO: Change this beta value according to r-zone!
        beta = math.pi / 2

        # TODO: Check that this goes far above enough to clear the cube below if it's stacking
        self.move_to_position_by_r_and_alpha(r, alpha, -0.04 + 0.07, beta)

        # Move to point and release
        self.move_to_position_by_r_and_alpha(r, alpha, -0.04 + (0.04 if stacking else 0), beta)
        self.open_gripper()

        # Move 6 cm in front of the cube (where we know there is empty space)
        # Note: This does decrease the available workspace
        self.move_to_position_by_r_and_alpha(r - 0.06, alpha, -0.04 + (0.04 if stacking else 0), beta)

    def move_cube_on_alpha(self, cube_info, new_alpha):
        self.grab_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))
        self.place_at_coords(self.get_r(cube_info.point), new_alpha)

    # def move_cube_on_table(self, point_from, point_to):
    #     self.grab_at_coords(point_from)
    #     self.place_at_coords(point_to)

    # def stack_cube(self, point_from, point_to):
    #     self.grab_at_coords(point_from)
    #     self.place_at_coords(point_to, True)
    




if __name__ == "__main__":
    rospy.init_node("main_program")
    try:
        fwd_kin = MainProgram()
    except rospy.ROSInterruptException:
        pass


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
