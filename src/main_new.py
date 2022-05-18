#! /usr/bin/env python

import rospy
import math
from robotics_cswk_kin.srv import IKinMsg, IKinMsgRequest
from geometry_msgs.msg import Point

from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

from cube_spotter.msg import Cube_co_Array, cube_co

CUBE_POSITIONS_TOPIC_NAME = '/Cordinates'

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
        self.move_time = 0.85
        self.search_move_time_short = 0.5
        self.search_move_time_long = 0.9
        self.search_rate = 1.8
        self.search_z = 0.035

        self.yellow_cube_locations = []
        self.red_cube_locations = []
        self.blue_cube_locations = []

        self.cube_infos = []

        print("Setting up")
        self.setup()

        # 0. Specify distance zones and approach angles
        self.distance_zones = []
        self.specify_distance_zones()

        # 1. Sub to cube information and read it
        print("Subscribing to cube position info")
        self.sub_to_cube_info()

        # 2. Search (move around), detect cube positions and average them
        print("Searching...")
        self.search()

        # 3. We now have a map of cubes
        # 4. Assess the situation
        print("Moving cubes...")
        self.assess_situation()

    def setup(self):
        # Initialize inverse kinematics service proxy and wait for the service
        self.inv_kin_proxy = rospy.ServiceProxy("/inv_kin", IKinMsg)
        rospy.wait_for_service("/inv_kin")
        self.set_pose_proxy = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        rospy.wait_for_service('/goal_joint_space_path')
        self.set_gripper = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        rospy.wait_for_service('/goal_tool_control')
        self.open_gripper()

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
                (90.0 - (i + 2) * 7.0) / 180.0 * math.pi,              # say the range is 12 degrees in those 3 centimeters
                (90.0 - i * 7.0) / 180.0 * math.pi                   # there will always be a 6 degree overlap
            ))

    def sub_to_cube_info(self):
        # This format:
        self.cube_position_subscriber = rospy.Subscriber(CUBE_POSITIONS_TOPIC_NAME, Cube_co_Array, self.cube_info_handler)
        pass
        

    def cube_info_handler(self, msg):
        if self.sequence_complete or not self.searching:
            return

        for cube in msg.Position:
            if cube.CubeColour.data == 'yellow' and not 'yellow' in self.excluded_colors:
                self.yellow_cube_locations.append(cube)
            elif cube.CubeColour.data == 'red' and not 'red' in self.excluded_colors:
                self.red_cube_locations.append(cube)
            elif cube.CubeColour.data == 'blue' and not 'blue' in self.excluded_colors:
                self.blue_cube_locations.append(cube)

    def move_to_position(self, x, y, z, beta, stacking = False, short_search = False, long_search = False):
        request = IKinMsgRequest()
        request.position = Point()
        request.position.x = float(x)
        request.position.y = float(y)
        request.position.z = float(z)
        request.angle.data = float(beta)

        response = self.inv_kin_proxy(request)
        r = [x.data for x in response.joint_positions]

        if not response.success.data or request.position.x < 0.02:
            print('Failed inverse kinematics. Where the heck did you send it?')
        else:
            jointRequest=JointPosition()
            jointRequest.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']  
            jointRequest.position = [x.data for x in response.joint_positions]
            self.set_pose_proxy(str(), jointRequest, self.move_time * (3 if stacking else 1) * (1 if short_search else 1))
            r = rospy.Rate(1 / (self.move_time * (3 if stacking else 1) * (1 if short_search else 1) * 1.05))
            r.sleep()

    def move_to_position_by_r_and_alpha(self, r, alpha, z, beta, stacking = False, short_search = False, long_search = False):
        return self.move_to_position(r * math.cos(alpha), r * math.sin(alpha), z, beta, stacking, short_search, long_search)

    def search(self, excluded_colors = []):
        # Move to initial position
        self.move_to_position(0.15, 0, 0.1, math.pi / 4)

        # Always run through all the positions so as to spot
        # all the cubes that could possibly be somewhere
        for zone_index in range(4):
            print("Distance zone" + str(zone_index))
            for alpha_index in range(5):
                print("Angle zone" + str(alpha_index))
                # scan every third zone for now, if they are
                # e.g. bigger, change this
                r = self.distance_zones[zone_index].get_mid_r()
                alpha = ((alpha_index - 2) * 14) / 180.0 * math.pi
                beta = self.distance_zones[zone_index].get_mid_beta()
                
                self.move_to_position_by_r_and_alpha(r, alpha, self.search_z, beta, short_search=(alpha_index != 0), long_search=(alpha_index == 0))
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
        p.x = self.get_average(map(lambda loc: loc.x_co.data, locations))
        p.y = self.get_average(map(lambda loc: -loc.y_co.data, locations))
        p.z = -0.04
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
        
        if len(self.cube_infos) < 2:
            raise Exception("Nothing to stack")
        else:
            print("Found " + str(len(self.cube_infos)) + " cubes.")

        # The sequence is now as follows:
        # 1. Order them by r distance
        self.cube_infos.sort(key=lambda c: math.sqrt(c.point.x ** 2 + c.point.y ** 2))

        # 2a. Always move in to a smaller r position and approach from the back
        # 2b. Move each to its own alpha section
        alpha_sections = [
            [10.0 / 180.0 * math.pi, 60.0 / 180.0 * math.pi],
            [-40.0 / 180.0 * math.pi, 10.0 / 180.0 * math.pi],
            [-85.0 / 180.0 * math.pi, -40.0 / 180.0 * math.pi]
        ]
        colors_to_exclude = []
        print("Separating cubes...")
        for cube_info in self.cube_infos:
            free_section = self.find_empty_alpha_section_for_cube(cube_info, alpha_sections)
            self.move_cube_on_alpha(cube_info, self.get_average(free_section))
            cube_info.set_alpha(self.get_average(free_section))
            # Have to look through all of the other cubes again here
            colors_to_exclude.append(cube_info.color)
            if (len(self.cube_infos) == len(colors_to_exclude)):
                break
            # assume it doesn't knock anything out of it's place
            # when enabling this, make sure to delete the original position of the cubes
            # so that in case they do move, it actually works
            # self.search(colors_to_exclude)

        # 3. Move each to the right r section
        # Have to update their position with camera
        # for cube_info in self.cube_infos:
        #     self.grab_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))
        #     self.place_at_coords(self.get_r(cube_info.point), self.get_alpha(cube_info.point))

        
        cube_zones = []

        self.cube_infos.sort(key=lambda c: math.sqrt(c.point.x ** 2 + c.point.y ** 2), reverse=True)

        for cube_info in self.cube_infos:
            for z in self.distance_zones:
                if z.r_min < self.get_r(cube_info.point) < z.r_max:
                    cube_zones.append(z)
                    break

        mid_index = 3 #round(sum(map(lambda z: z.index, cube_zones)) / len(cube_zones))
        # forced to be 3 for consistency

        print("Moving to same distance...")

        for cube_info in self.cube_infos:
            zone_index = None
            for z in self.distance_zones:
                if z.r_min < self.get_r(cube_info.point) < z.r_max:
                    zone_index = z.index
                    break
            
            if zone_index is None:
                continue

            no_move_yet = True

            if zone_index > mid_index:
                while zone_index > mid_index:
                    zone_index -= 1
                    self.move_cube_on_r(cube_info, self.distance_zones[zone_index].get_mid_r(), start=no_move_yet, end=zone_index == mid_index)
                    cube_info.set_r(self.distance_zones[zone_index].get_mid_r())
                    no_move_yet = False
            elif zone_index < mid_index:
                while zone_index < mid_index:
                    zone_index += 1
                    self.move_cube_on_r(cube_info, self.distance_zones[zone_index].get_mid_r(), start=no_move_yet, end=zone_index == mid_index)
                    cube_info.set_r(self.distance_zones[zone_index].get_mid_r())
                    no_move_yet = False
            elif zone_index == mid_index:
                self.move_cube_on_r(cube_info, self.distance_zones[zone_index].get_mid_r(), start=True, end=True)
                cube_info.set_r(self.distance_zones[zone_index].get_mid_r())
        
        # 4. Stack

        print("Stacking...")

        self.cube_infos.sort(key=lambda c: c.get_alpha(c.point))

        for i in range(len(self.cube_infos) - 1):
            cube_info = self.cube_infos[i + 1 if i == 1 else i]
            next_cube_info = self.cube_infos[i if i == 1 else i + 1]
            self.move_cube_on_alpha(cube_info, self.get_alpha(next_cube_info.point), True, i == 1)

        self.move_to_position(0.15, 0, 0.1, math.pi / 4)
        print("Done.")

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
        grip_request.position = [-0.0067] # -0.01 represents closed
        self.set_gripper(str(), grip_request, 1.0)
        rospy.sleep(1) # Wait for the gripper to close

    def grab_at_coords(self, r, r_to_get_angle_from, alpha, no_up = False, no_down = False, stacking = False, second_stacking = False):
        # TODO: Change this beta value according to r-zone!
        beta = math.pi / 2

        # Move 6 cm in front of the cube (where we know there is empty space)
        # Note: This does decrease the available workspace
        if not no_down:
            self.move_to_position_by_r_and_alpha(r - 0.08, alpha, self.search_z, self.get_distance_zone_angle(r_to_get_angle_from - 0.08))
            self.move_to_position_by_r_and_alpha(r - 0.08, alpha, -0.06, self.get_distance_zone_angle(r_to_get_angle_from - 0.08))

        self.move_to_position_by_r_and_alpha(r, alpha, -0.06, self.get_distance_zone_angle(r_to_get_angle_from))
        self.close_gripper()
        # Move to cube, grip it and get out (directly up)
        # TODO: This might limit the space. If too far away, make it go back as well or something
        if not no_up:
            self.move_to_position_by_r_and_alpha(r, alpha, (0.101 if second_stacking else 0.063 if stacking else self.search_z), self.get_distance_zone_angle(r_to_get_angle_from))

    def place_at_coords(self, r, r_to_get_angle_from, alpha, stacking = False, second_stacking = False, no_up = False, no_down = False):
        # TODO: Change this beta value according to r-zone!
        beta = math.pi / 2

        # TODO: Check that this goes far above enough to clear the cube below if it's stacking
        if not no_down:
            self.move_to_position_by_r_and_alpha(r, alpha, (0.101 if second_stacking else 0.063 if stacking else 0), self.get_distance_zone_angle(r_to_get_angle_from))

        # Move to point and release
        # The offset - 0.035 at 0, 0 at 0.35
        # y = 0.35 - r / 10
        if stacking or second_stacking:
            self.move_to_position_by_r_and_alpha(r, alpha, -0.06 + (0.101 if second_stacking else 0.063 if stacking else 0), self.get_distance_zone_angle(r_to_get_angle_from), stacking or second_stacking)
        self.move_to_position_by_r_and_alpha(r, alpha, -0.06 + (0.076 if second_stacking else 0.038 if stacking else 0), self.get_distance_zone_angle(r_to_get_angle_from), stacking or second_stacking)

        self.open_gripper()
        # Move 6 cm in front of the cube (where we know there is empty space)
        # Note: This does decrease the available workspace
        if not no_up:
            self.move_to_position_by_r_and_alpha(r, alpha, (0.101 if second_stacking else 0.063 if stacking else self.search_z), self.get_distance_zone_angle(r_to_get_angle_from))

    def move_cube_on_alpha(self, cube_info, new_alpha, stacking = False, second_stacking = False):
        self.grab_at_coords(self.get_r(cube_info.point), self.get_r(cube_info.point), self.get_alpha(cube_info.point), stacking=stacking, second_stacking=second_stacking)
        self.place_at_coords(self.get_r(cube_info.point), self.get_r(cube_info.point), new_alpha, stacking=stacking, second_stacking=second_stacking)


    def move_cube_on_r(self, cube_info, new_r, start = False, end = False):
        self.grab_at_coords(self.get_r(cube_info.point), self.get_r(cube_info.point), self.get_alpha(cube_info.point), no_up=True, no_down=not start)
        self.place_at_coords(new_r, self.get_r(cube_info.point), self.get_alpha(cube_info.point), no_up=not end, no_down=True)

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
