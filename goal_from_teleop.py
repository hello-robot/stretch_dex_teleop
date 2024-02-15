import numpy as np
import webcam_teleop_interface as wt
import simple_ik as si
import argparse
import pprint as pp
import loop_timer as lt
import dex_teleop_parameters as dt
from multiprocessing import shared_memory
import os


class GoalFromMarkers:
    def __init__(self, teleop_origin, initial_center_wrist_position, slide_lift_range=False):

        print('GoalFromMarkers: teleop_origin =', teleop_origin)
        print('GoalFromMarkers: initial_center_wrist_position =', initial_center_wrist_position)

        self.slide_lift_range = slide_lift_range
        
        self.grip_pose_marker_name = 'tongs'
        self.grip_width_marker_name = 'tongs'
        self.teleop_origin = teleop_origin
        self.initial_center_wrist_position = np.array(initial_center_wrist_position)
        self.center_wrist_position = self.initial_center_wrist_position.copy()
        
        # Regions at the top and bottom of the allowable tongs range
        # are reserved for changing the range over which the lift is
        # operating. This sliding region enables a user to use the
        # lift's full range without restarting the code.
        self.sliding_region_height = dt.lift_sliding_region_height

        # The tongs are ignored when they are outside of this range.
        self.tongs_min_dist_from_camera = dt.min_dist_from_camera_to_tongs - self.sliding_region_height
        self.tongs_max_dist_from_camera = dt.max_dist_from_camera_to_tongs + self.sliding_region_height
        
        print('GoalFromMarkers: self.tongs_min_dist_from_camera = {:.2f} cm'.format(self.tongs_min_dist_from_camera * 100.0))
        print('GoalFromMarkers: self.tongs_max_dist_from_camera = {:.2f} cm'.format(self.tongs_max_dist_from_camera * 100.0))
        print('GoalFromMarkers: self.sliding_region_height = {:.2f} cm'.format(self.sliding_region_height * 100.0))
        

        # These values determine when the lift range should be slid up
        # or down.
        self.slide_lift_range_down = self.tongs_min_dist_from_camera + self.sliding_region_height
        self.slide_lift_range_up = self.tongs_max_dist_from_camera - self.sliding_region_height

        # The tongs can be moved over a range of distances from the
        # camera to actively control the lift. This is the height of
        # this region when ignoring the lift's joint limits.
        self.tongs_lift_range = self.slide_lift_range_up - self.slide_lift_range_down
        print('GoalFromMarkers: self.tongs_lift_range = {:.2f} cm'.format(self.tongs_lift_range * 100.0))

        # The maximum and minimum goal_wrist_position z values do not
        # need to be perfect due to joint limit checking performed by
        # the SimpleIK based on the specialized URDF joint
        # limits. They are specified with respect to the robot's
        # coordinate system.
        self.max_goal_wrist_position_z = dt.goal_max_position_z
        self.min_goal_wrist_position_z = dt.goal_min_position_z
        
        self.max_lift_range_offset = (((self.max_goal_wrist_position_z - self.center_wrist_position[2]) +
                                       self.teleop_origin[2]) -
                                      (self.tongs_max_dist_from_camera - self.sliding_region_height)
                                      )

        self.min_lift_range_offset = (((self.min_goal_wrist_position_z - self.center_wrist_position[2]) +
                                       self.teleop_origin[2]) -
                                      (self.tongs_min_dist_from_camera + self.sliding_region_height)
                                      )
        
        print('self.min_lift_range_offset = {:.2f} cm'.format(self.min_lift_range_offset * 100.0))
        print('self.max_lift_range_offset = {:.2f} cm'.format(self.max_lift_range_offset * 100.0))

        # Initialized the offset.
        self.lift_range_offset = 0.0

        # Set how fast the lift will be translated when being slid.
        self.lift_range_offset_change_per_timestep = dt.lift_range_offset_change_per_timestep

        self.in_sliding_region = False

        self.min_finger_width = dt.tongs_closed_grip_width
        self.max_finger_width = dt.tongs_open_grip_width

    def get_goal_dict(self, markers): 

        if markers:
            grip_pose_marker = markers.get(self.grip_pose_marker_name, None)
            grip_width_marker = markers.get(self.grip_width_marker_name, None)

            # If the marker (real or virtual) that specifies the
            # gripper pose goal has been observed and its z-axis
            # hasn't changed too much, proceed.
            if grip_pose_marker is not None: 

                # Transform the gripper pose marker (real or
                # virtual) to a goal position for the wrist.

                # The wrist position goal is defined with respect
                # to the world frame, which has its origin where
                # the mobile base's rotational axis intersects
                # with the ground. The world frame's z-axis points
                # up. When the robot's mobile base has angle 0,
                # the world frame's x-axis points in front of the
                # robot and its y-axis points to the left of the
                # robot in the direction opposite to arm extension

                teleop_marker_position_in_camera_frame = grip_pose_marker['pos']

                goal = None

                dist_from_camera = teleop_marker_position_in_camera_frame[2]

                tongs_at_valid_distance_from_camera = ((dist_from_camera > self.tongs_min_dist_from_camera) and
                                                       (dist_from_camera < self.tongs_max_dist_from_camera))

                if tongs_at_valid_distance_from_camera: 
                    
                    command_to_slide_lift_range = ((dist_from_camera < self.slide_lift_range_down) or
                                                   (dist_from_camera > self.slide_lift_range_up))

                    if self.slide_lift_range and command_to_slide_lift_range:
                        if not self.in_sliding_region:
                            os.system("/usr/bin/canberra-gtk-play --id='bell'")
                            self.in_sliding_region = True
                        if dist_from_camera < self.slide_lift_range_down:
                            self.lift_range_offset = self.lift_range_offset - self.lift_range_offset_change_per_timestep
                            teleop_marker_position_in_camera_frame[2] = self.slide_lift_range_down
                        elif dist_from_camera > self.slide_lift_range_up:
                            self.lift_range_offset = self.lift_range_offset + self.lift_range_offset_change_per_timestep
                            teleop_marker_position_in_camera_frame[2] = self.slide_lift_range_up
                        self.lift_range_offset = min(self.max_lift_range_offset, self.lift_range_offset)
                        self.lift_range_offset = max(self.min_lift_range_offset, self.lift_range_offset)

                        print('self.lift_range_offset = {:.2f} cm'.format(self.lift_range_offset * 100.0))
                    else:
                        if self.in_sliding_region:
                            os.system("/usr/bin/canberra-gtk-play --id='bell'")
                            self.in_sliding_region = False
                            

                    goal_wrist_position = ((teleop_marker_position_in_camera_frame - self.teleop_origin) +
                                           self.center_wrist_position)
                    goal_wrist_position[2] = goal_wrist_position[2] + self.lift_range_offset


                    # If the gripper width marker (virtual or real)
                    # has been observed, use it to command the robot's
                    # gripper.
                    goal_grip_width = None
                    if grip_width_marker is not None:
                        grip_width = grip_width_marker['info']['grip_width']
                        # convert to value between 0.0 and 1.0
                        goal_grip_width = (np.clip(grip_width, self.min_finger_width, self.max_finger_width) - self.min_finger_width)/ (self.max_finger_width - self.min_finger_width)
                        # convert to value between -100.0 and 100.0

                    goal_x_axis = grip_pose_marker['x_axis']
                    goal_y_axis = grip_pose_marker['y_axis']
                    goal_z_axis = grip_pose_marker['z_axis']

                    goal = {'grip_width': goal_grip_width,
                            'wrist_position': goal_wrist_position,
                            'gripper_x_axis': goal_x_axis,
                            'gripper_y_axis': goal_y_axis,
                            'gripper_z_axis': goal_z_axis}
                    
                return goal
            else:
                return None
       
            
    def get_goal_array(self, markers):
        dict_out = self.get_goal_dict(markers)
        return dt.goal_dict_to_array(dict_out)
            
        
if __name__ == '__main__':

    args = dt.get_arg_parser().parse_args()
    use_fastest_mode = args.fast
    manipulate_on_ground = args.ground
    left_handed = args.left
    using_stretch_2 = args.stretch_2
    use_multiprocessing = args.multiprocessing
    slide_lift_range = args.slide_lift_range

    # When False, the robot should only move to its initial position
    # and not move in response to ArUco markers. This is helpful when
    # first trying new code and interface objects.
    robot_allowed_to_move = True

    # The 'default', 'slow', 'fast', and 'max' options are defined by
    # Hello Robot. The 'fastest_stretch_2' option has been specially tuned for
    # this application.
    #
    # WARNING: 'fastest_stretch_2' has velocities and accelerations that exceed
    # the factory 'max' values defined by Hello Robot.
    if use_fastest_mode:
        if using_stretch_2:
            robot_speed = 'fastest_stretch_2'
        else: 
            robot_speed = 'fastest_stretch_3'
    else:
        robot_speed = 'slow'
    print('running with robot_speed =', robot_speed)
    
    lift_middle = dt.get_lift_middle(manipulate_on_ground)
    center_configuration = dt.get_center_configuration(lift_middle)
    starting_configuration = dt.get_starting_configuration(lift_middle)

    if left_handed: 
        webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='left', visualize_detections=False)
    else:
        webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='right', visualize_detections=False)
    
    # Initialize IK
    simple_ik = si.SimpleIK()

    # Define the center position for the wrist that corresponds with
    #the teleop origin.
    center_wrist_position = simple_ik.fk_rotary_base(center_configuration)

    goal_from_markers = GoalFromMarkers(dt.teleop_origin, center_wrist_position, slide_lift_range=slide_lift_range)
    
    if use_multiprocessing:

        goal_array = dt.get_example_goal_array()
        
        shm = shared_memory.SharedMemory(name=dt.shared_memory_name, create=True, size=goal_array.nbytes)
        shared_memory_goal_array = np.ndarray(goal_array.shape, dtype=goal_array.dtype, buffer=shm.buf)

        shared_memory_goal_array[:] = dt.get_do_nothing_goal_array()[:]

    loop_timer = lt.LoopTimer()
    print_timing = True
    first_goal_sent = False
    
    try: 
        while True:
            loop_timer.start_of_iteration()
            markers = webcam_aruco_detector.process_next_frame()
            if use_multiprocessing: 
                goal_array = goal_from_markers.get_goal_array(markers)
                if goal_array is not None: 
                    shared_memory_goal_array[:] = goal_array[:]
                    if not first_goal_sent:
                        loop_timer.reset()
                        first_goal_sent = True
                print('goal_array =')
                pp.pprint(goal_array)

            else: 
                goal = goal_from_markers.get_goal_dict(markers)
                print('goal =')
                pp.pprint(goal)
                if goal is not None:
                    if not first_goal_sent:
                        loop_timer.reset()
                        first_goal_sent = True

            loop_timer.end_of_iteration()
            if print_timing: 
                loop_timer.pretty_print()
    finally:
        if use_multiprocessing:
            print('cleaning up shared memory multiprocessing')
            shm.close()
            shm.unlink()
            
##############################################################
## NOTES
##############################################################

#######################################
#
# Overview
#
# Dexterous teleoperation uses a marker dictionary representing either
# a real or virtual ArUco marker specified with respect to the
# camera's frame of reference. The marker's position controls the
# robot's wrist position via inverse kinematics (IK). The marker's
# orientation directly controls the joints of the robot's dexterous
# wrist.
#
#######################################

#######################################
#
# The following coordinate systems are important to this teleoperation
# code
#
#######################################

#######################################
# ArUco Coordinate System
#
# Origin in the middle of the ArUco marker.
#
# x-axis
# right side when looking at marker is pos
# left side when looking at marker is neg

# y-axis
# top of marker is pos
# bottom of marker is neg

# z-axis
# normal to marker surface is pos
# pointing into the marker surface is neg
#
#######################################

#######################################
# Camera Coordinate System
#
# Camera on the floor looking with the top of the camer facing away
# from the person.
#
# This configuration matches the world frame's coordinate system with
# a different origin that is mostly just translated along the x and y
# axes.
#
# Origin likely at the optical cemter of a pinhole
# model of the camera.
#
# The descriptions below describe when the robot's mobile base is at
# theta = 0 deg.
#
# x-axis
# human left is pos / robot forward is pos
# human right is neg / robot backward is neg

# y-axis
# human arm extended is neg / robot arm extended is neg
# human arm retracted is pos / robot arm retracted is pos

# z-axis
# up is positive for person and the robot
# down is negative for person and the robot
#
#######################################

#######################################
# IK World Frame Coordinate System
#
# Origin at the axis of rotation of the mobile
# base on the floor.
#
# x-axis
# human/robot left is pos
# human/robot right is neg

# y-axis
# human/robot forward is neg
# human/robot backward is pos

# z-axis
# human/robot up is pos
# human/robot down is neg
#
#######################################

#######################################
# Robot Wrist Control

# wrist yaw
#     - : deployed direction
#     0 : straight out parallel to the telescoping arm
#     + : stowed direction

# wrist pitch
#     - : up
#     0 : horizontal
#     + : down

# wrist roll
#     - : 
#     0 : horizontal
#     + :
#
#######################################

##############################################################
