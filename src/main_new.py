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

    def get_r(self, point):
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def get_alpha(self, point):
        return math.asin(point.y / self.get_r(point))

    def get_xy(self, r, alpha):
        return [r * math.cos(alpha), r * math.sin(alpha)]

    def set_alpha(self, new_alpha):
        r = self.get_r(self.point)
        self.point.x = self.get_xy(r, new_alpha)[0]
        self.point.y = self.get_xy(r, new_alpha)[1]

    def set_r(self, new_r):
        alpha = self.get_alpha(self.point)
        self.point.x = self.get_xy(new_r, alpha)[0]
        self.point.y = self.get_xy(new_r, alpha)[1]

class DistanceZone:

    def __init__(self, index, r_min, r_max, beta_min, beta_max):
        self.index = index
        self.r_min = r_min
        self.r_max = r_max
        self.beta_min = beta_min
        self.beta_max = beta_max

    def get_mid_beta(self):
        print((self.beta_min + self.beta_max) / 2)
        return (self.beta_min + self.beta_max) / 2

    def get_mid_r(self):
        print(self.r_min)
        print(self.r_max)
        return (self.r_min + self.r_max) / 2


class MainProgram:

    def __init__(self):

        # -1. Setup everything that's needed
        self.sequence_complete = False
        self.searching = False
        # colors excluded from search
        self.excluded_colors = []
        self.move_time = 0.9
        self.search_rate = 2
        self.search_z = 0.05

        self.yellow_cube_locations = []
        self.red_cube_locations = []
        self.blue_cube_locations = []

        self.cube_infos = []

        self.setup()

        # 0. Specify distance zones and approach angles
        self.distance_zones = []
        self.specify_distance_zones()

        # 1. Sub to cube information and read it
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

    def get_distance_zone_angle(self, r):
        for z in self.distance_zones:
            if z.r_min < r < z.r_max:
                return z.get_mid_beta()
        return math.pi / 2

    def specify_distance_zones(self):
        for i in range(9):
            # edit this when you test it on the robot to make it as fast as possible
            # for now it starts at six centimeters, we'll see how close we can reach
            self.distance_zones.append(DistanceZone(
                i,
                3.0 * (i + 3) / 100.0,              # every zone is 3 centimeters
                3.0 * (i + 4) / 100.0,              # it ends after the 3 centimeters
                (90.0 - (i + 2) * 10.0) / 180.0 * math.pi,              # say the range is 12 degrees in those 3 centimeters
                (90.0 - i * 10.0) / 180.0 * math.pi                   # there will always be a 6 degree overlap
            ))

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
        request.position.x = float(x)
        request.position.y = float(y)
        request.position.z = float(z)
        request.angle.data = float(beta)

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
            r = rospy.Rate(1 / (self.move_time * 1.2))
            r.sleep()

    def move_to_position_by_r_and_alpha(self, r, alpha, z, beta):
        return self.move_to_position(r * math.cos(alpha), r * math.sin(alpha), z, beta)

    def search(self, excluded_colors = []):
        # Move to initial position
        print("Searching")
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)

        # Always run through all the positions so as to spot
        # all the cubes that could possibly be somewhere
        for zone_index in range(4):
            for alpha_index in range(1):
                # scan every third zone for now, if they are
                # e.g. bigger, change this
                r = self.distance_zones[zone_index].get_mid_r()
                alpha = ((alpha_index) * 25) / 180.0 * math.pi
                beta = self.distance_zones[zone_index].get_mid_beta()
                
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
        return math.asin(point.y / self.get_r(point))

    def assess_situation(self):
        # Step 1: Check how many cubes we have
        if len(self.yellow_cube_locations) > 5:
            self.add_final_cube_location(self.yellow_cube_locations, 1, 'yellow')
        if len(self.red_cube_locations) > 5:
            self.add_final_cube_location(self.red_cube_locations, 2, 'red')
        if len(self.blue_cube_locations) > 5:
            self.add_final_cube_location(self.blue_cube_locations, 3, 'blue')
        
        # The sequence is now as follows:
        # 1. Order them by r distance
        self.cube_infos.sort(key=lambda c: math.sqrt(c.point.x ** 2 + c.point.y ** 2))

        print("Cube infos")
        print(self.cube_infos)

        # 2a. Always move in to a smaller r position and approach from the back
        # 2b. Move each to its own alpha section
        alpha_sections = [
            [-90.0 / 180.0 * math.pi, -40.0 / 180.0 * math.pi],
            [-40.0 / 180.0 * math.pi, 10.0 / 180.0 * math.pi],
            [10.0 / 180.0 * math.pi, 60.0 / 180.0 * math.pi]
        ]
        colors_to_exclude = []
        for cube_info in self.cube_infos:
            free_section = self.find_empty_alpha_section_for_cube(cube_info, alpha_sections)
            self.move_cube_on_alpha(cube_info, self.get_average(free_section))
            cube_info.set_alpha(self.get_average(free_section))
            # Have to look through all of the other cubes again here
            colors_to_exclude.append(cube_info.color)
            if (len(self.cube_infos) == len(colors_to_exclude)):
                break
            self.search(colors_to_exclude)

        # 3. Move each to the right r section
        # Have to update their position with camera
        # for cube_info in self.cube_infos:
        #     self.grab_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))
        #     self.place_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))

        cube_zones = []

        for cube_info in self.cube_infos:
            for z in self.distance_zones:
                if z.r_min < self.get_r(cube_info.point) < z.r_max:
                    cube_zones.append(z)
                    break

        mid_index = round(sum(map(lambda z: z.index, cube_zones)) / len(cube_zones))

        for cube_info in self.cube_infos:
            zone_index = None
            for z in self.distance_zones:
                if z.r_min < self.get_r(cube_info.point) < z.r_max:
                    zone_index = z.index
                    break
            
            if zone_index is None:
                continue

            if zone_index > mid_index:
                while zone_index > mid_index:
                    zone_index -= 1
                    self.move_cube_on_r(cube_info, self.distance_zones[zone_index].get_mid_r())
                    cube_info.set_r(self.distance_zones[zone_index].get_mid_r())
            elif zone_index < mid_index:
                while zone_index < mid_index:
                    zone_index += 1
                    self.move_cube_on_r(cube_info, self.distance_zones[zone_index].get_mid_r())
                    cube_info.set_r(self.distance_zones[zone_index].get_mid_r())
            elif zone_index == mid_index:
                self.move_cube_on_r(cube_info, self.distance_zones[zone_index].get_mid_r())
                cube_info.set_r(self.distance_zones[zone_index].get_mid_r())
        
        # 4. Stack

        for i in range(len(self.cube_infos) - 1):
            cube_info = self.cube_infos[i + 1 if i == 1 else i]
            next_cube_info = self.cube_infos[i if i == 1 else i + 1]
            self.move_cube_on_alpha(cube_info, self.get_alpha(next_cube_info.point), True, i == 1)

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
        self.move_to_position_by_r_and_alpha(r - 0.08, alpha, -0.06, self.get_distance_zone_angle(r - 0.06))
        # Move to cube, grip it and get out (directly up)
        self.move_to_position_by_r_and_alpha(r, alpha, -0.06, self.get_distance_zone_angle(r))
        self.close_gripper()
        # TODO: This might limit the space. If too far away, make it go back as well or something
        self.move_to_position_by_r_and_alpha(r, alpha, -0.06 + 0.14, self.get_distance_zone_angle(r))

    def place_at_coords(self, r, alpha, stacking = False, second_stacking = False):
        # TODO: Change this beta value according to r-zone!
        beta = math.pi / 2

        # TODO: Check that this goes far above enough to clear the cube below if it's stacking
        self.move_to_position_by_r_and_alpha(r, alpha, -0.06 + 0.14, self.get_distance_zone_angle(r))

        # Move to point and release
        self.move_to_position_by_r_and_alpha(r, alpha, -0.06 + (0.08 if second_stacking else 0.04 if stacking else 0), self.get_distance_zone_angle(r))
        self.open_gripper()

        # Move 6 cm in front of the cube (where we know there is empty space)
        # Note: This does decrease the available workspace
        self.move_to_position_by_r_and_alpha(r - 0.08, alpha, -0.06 + (0.14), self.get_distance_zone_angle(r - 0.06))

    def move_cube_on_alpha(self, cube_info, new_alpha, stacking = False, second_stacking = False):
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)
        self.grab_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))
        self.place_at_coords(self.get_r(cube_info.point), new_alpha, stacking, second_stacking)
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)

    def move_cube_on_r(self, cube_info, new_r):
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)
        self.grab_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))
        self.place_at_coords(new_r, self.get_alpha(cube_info.point))
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)

    # def move_cube_on_table(self, point_from, point_to):
    #     self.grab_at_coords(point_from)
    #     self.place_at_coords(point_to)

    # def stack_cube(self, point_from, point_to):
    #     self.grab_at_coords(point_from)
    #     self.place_at_coords(point_to, True)
    




if __name__ == "__main__":
    rospy.init_node("main_program")
    try:
        main = MainProgram()
    except rospy.ROSInterruptException:
        pass


# i = 0

# scanned = False
# executed = False
# initialized = False
# done = False
# measure = False

# x_red = []
# y_red = []
# z_red = []

# x_yellow = []
# y_yellow = []
# z_yellow = []

# def real_cube_data_handler(msg):

#     global i 
#     i += 1
#     if i % 1 != 0:
#         return

#     global x
#     global y
#     global z

#     global scanned
#     global executed
#     global initialized
#     global done
#     global measure

#     if not initialized or done or not measure:
#         return

#     for c in msg.cubes:
#         if c.color.data == "yellow":
#             x_yellow.append(c.position.x)
#             y_yellow.append(c.position.y)
#             z_yellow.append(c.position.z)
#         elif c.color.data == "red":
#             print("adding to red")
#             x_red.append(c.position.x)
#             y_red.append(c.position.y)
#             z_red.append(c.position.z)

#     # if len(x) > 0 and len(msg.cubes) > 0:
#     #     rospy.loginfo(sum(x) / len(x))
#     #     rospy.loginfo(sum(y) / len(y))
#     #     rospy.loginfo(sum(z) / len(z))

# def move_to_position(x, y, z, angle, time):
#     request = IKinMsgRequest()
#     request.position = Point()
#     request.position.x = x
#     request.position.y = y
#     request.position.z = z
#     request.angle.data = angle

#     response = proxy(request)
#     r = [x.data for x in response.joint_positions]
#     print(r)

#     if not response.success.data:
#         print('Failed inv kin')
#     else:
#         setPose = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

#         rospy.wait_for_service('/goal_joint_space_path')

#         jointRequest=JointPosition()
#         jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
#         jointRequest.position=[x.data for x in response.joint_positions]
#         setPose(str(), jointRequest, time)

#     r = rospy.Rate(1 / (time * 1.3))
#     r.sleep()

# def open_gripper():
#     # Open the gripper
#     gripperRequest=JointPosition()
#     gripperRequest.joint_name=["gripper"] 
#     gripperRequest.position=[0.01]# 0.01 represents open
#     setGripper(str(),gripperRequest,1.0)

#     rospy.sleep(1) # Wait for the gripper to open

# def close_gripper():
#     # Close the gripper
#     gripperRequest=JointPosition()
#     gripperRequest.joint_name=["gripper"]  
#     gripperRequest.position=[-0.0065]# -0.01 represents closed
#     setGripper(str(),gripperRequest,1.5)

#     rospy.sleep(1) # Wait for the gripper to close

# real_cubes_sub = rospy.Subscriber('/real_cubes', RealCubeArray, real_cube_data_handler)
# setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)

# move_to_position(0.15, 0, 0.1, math.pi / 4, 1.7)

# initialized = True

# for k in range(2):
#     if len(x_yellow) > 1 and len(x_red) > 1:
#         break
#     print("Going for position", k)
#     angle = math.pi / 2
#     if k > 0:
#         angle = math.pi / 4
#     move_to_position(0.05 + 0.08 * k, 0, 0.1, angle, 1.0)
#     s = rospy.Rate(3)
#     measure = True
#     s.sleep()
#     measure = False

# scanned = True

# while not (scanned and not executed and len(x_yellow) > 0 and len(x_red) > 0):
#     sl = rospy.Rate(1)
#     sl.sleep()

# executed = True
# angle = math.pi / 12
# z = -0.04
# if math.sqrt((sum(x_yellow) / len(x_yellow)) ** 2 + (sum(y_yellow) / len(y_yellow)) ** 2) < 0.2:
#     angle = math.pi / 2
#     z = -0.05
# elif math.sqrt((sum(x_yellow) / len(x_yellow)) ** 2 + (sum(y_yellow) / len(y_yellow)) ** 2) < 0.3:
#     angle = math.pi / 4
#     z = -0.045
# move_to_position(sum(x_yellow) / len(x_yellow), sum(y_yellow) / len(y_yellow), z + 0.05, angle, 2.0)
# move_to_position(sum(x_yellow) / len(x_yellow), sum(y_yellow) / len(y_yellow), z, angle, 2.0)
# close_gripper()
# move_to_position(sum(x_yellow) / len(x_yellow), sum(y_yellow) / len(y_yellow), z + 0.1, angle, 2.0)
# move_to_position(sum(x_red) / len(x_red) - 0.005, sum(y_red) / len(y_red), z + 0.1, angle, 2.0)
# move_to_position(sum(x_red) / len(x_red) - 0.005, sum(y_red) / len(y_red), z + 0.04, angle, 2.0)
# open_gripper()

# move_to_position(0.15, 0, 0.1, math.pi / 4, 3.0)

# done = True
