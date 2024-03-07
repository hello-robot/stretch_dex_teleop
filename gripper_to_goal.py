import stretch_body.robot as rb
import numpy as np
import math
import time
from functools import partial
from scipy.spatial.transform import Rotation
from stretch_body.robot_params import RobotParams
from hello_helpers import hello_misc as hm
import urchin as urdf_loader
import os
import simple_ik as si
import argparse
import loop_timer as lt
import dex_teleop_parameters as dt
from multiprocessing import shared_memory
import pprint as pp
import robot_move as rm


def load_urdf(file_name):
    if not os.path.isfile(file_name):
        print()
        print('*****************************')
        print('ERROR: ' + file_name + ' was not found. Simple IK requires a specialized URDF saved with this file name. prepare_base_rotation_ik_urdf.py can be used to generate this specialized URDF.')
        print('*****************************')
        print()
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), file_name)
    urdf = urdf_loader.URDF.load(file_name, lazy_load_meshes=True)
    return urdf


def nan_in_configuration(configuration):
    for k, v in configuration.items():
        if math.isnan(v) or np.isnan(v):
            return True
    return False


                

class GripperToGoal:
    def __init__(self, robot_speed, starting_configuration, robot_allowed_to_move, using_stretch_2):
        if using_stretch_2:
            self.grip_range = dt.dex_wrist_grip_range
        else:
            self.grip_range = dt.dex_wrist_3_grip_range
            
        self.using_stretch_2 = using_stretch_2
                
        self.joints_allowed_to_move = ['aloha_gripper', 'joint_arm_l0', 'joint_lift', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_mobile_base_rotate_by']

        # Get Wrist URDF joint limits
        rotary_urdf_file_name = './stretch_base_rotation_ik_with_fixed_wrist.urdf'
        rotary_urdf = load_urdf(rotary_urdf_file_name)
        wrist_joints = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        self.wrist_joint_limits = {}
        for joint_name in wrist_joints: 
            joint = rotary_urdf.joint_map.get(joint_name, None)
            if joint is not None: 
                lower = float(joint.limit.lower)
                upper = float(joint.limit.upper)
                self.wrist_joint_limits[joint.name] = (lower, upper)

        self.robot_allowed_to_move = robot_allowed_to_move
        self.drop_extreme_wrist_orientation_change = True
            
        # Initialize the filtered wrist orientation that is used to
        # command the robot. Simple exponential smoothing is used to
        # filter wrist orientation values coming from the interface
        # objects.
        self.filtered_wrist_orientation = np.array([0.0, 0.0, 0.0])

        # Initialize the filtered wrist position that is used to command
        # the robot. Simple exponential smoothing is used to filter wrist
        # position values coming from the interface objects.
        self.filtered_wrist_position_configuration = np.array([
            starting_configuration['joint_mobile_base_rotate_by'],
            starting_configuration['joint_lift'],
            starting_configuration['joint_arm_l0']
        ])
    
        self.prev_commanded_wrist_orientation = {'joint_wrist_yaw':None,
                                                 'joint_wrist_pitch':None,
                                                 'joint_wrist_roll':None}

        
        # This is the weight multiplied by the current wrist angle command when performing exponential smoothing.
        # 0.5 with 'max' robot speed was too noisy on the wrist
        self.wrist_orientation_filter = dt.exponential_smoothing_for_orientation

        # This is the weight multiplied by the current wrist position command when performing exponential smoothing.
        # commands before sending them to the robot
        self.wrist_position_filter = dt.exponential_smoothing_for_position 

        # Initialize IK
        self.simple_ik = si.SimpleIK()
        
        ##########################################################
        # Prepare the robot last to avoid errors due to blocking calls
        # associated with other aspects of setting things up, such as
        # initializing SimpleIK.
        self.robot = rb.Robot()
        self.robot.startup()

        print('stretch_body file imported =', rb.__file__)
        transport_version = self.robot.arm.motor.transport.version
        print('stretch_body using transport version =', transport_version)

        self.robot_move = rm.RobotMove(self.robot, speed=robot_speed)
        self.robot_move.print_settings()

        self.robot_move.to_configuration(starting_configuration, speed='default')
        self.robot.push_command()
        self.robot.wait_command()

        # Set the current mobile base angle to be 0.0 radians.
        self.robot.base.reset_odometry()
        ##########################################################

        self.print_robot_status_thread_timing = False
        self.debug_wrist_orientation = False

        self.max_allowed_wrist_yaw_change = dt.max_allowed_wrist_yaw_change
        self.max_allowed_wrist_roll_change = dt.max_allowed_wrist_roll_change
    

    def __del__(self):
        print('GripperToGoal.__del__: stopping the robot')
        self.robot.stop()
        
        
    def update_goal(self, grip_width, wrist_position, gripper_x_axis, gripper_y_axis, gripper_z_axis):

        ###############################
        #INPUT: wrist_position

        # Use Simple IK to find configurations for the
        # mobile base angle, lift distance, and arm
        # distance to achieve the goal wrist position in
        # the world frame.
        new_goal_configuration = self.simple_ik.ik_rotary_base(wrist_position)
        if new_goal_configuration is None:
            print(f'WARNING: SimpleIK failed to find a valid new_goal_configuration so skipping this iteration by continuing the loop. Input to IK: wrist_position = {wrist_position}, Output from IK: new_goal_configuration = {new_goal_configuration}')
        else: 

            new_wrist_position_configuration = np.array([new_goal_configuration['joint_mobile_base_rotation'],
                                                         new_goal_configuration['joint_lift'],
                                                         new_goal_configuration['joint_arm_l0']])

            # Use exponential smoothing to filter the wrist
            # position configuration used to command the
            # robot.
            self.filtered_wrist_position_configuration = (((1.0 - self.wrist_position_filter) * self.filtered_wrist_position_configuration) +
                                                          (self.wrist_position_filter * new_wrist_position_configuration))

            new_goal_configuration['joint_lift'] = self.filtered_wrist_position_configuration[1]
            new_goal_configuration['joint_arm_l0'] = self.filtered_wrist_position_configuration[2]


            self.simple_ik.clip_with_joint_limits(new_goal_configuration)

            #################################

            #################################
            # INPUT: grip_width between 0.0 and 1.0
        
            if (grip_width is not None) and (grip_width > -1000.0): 
                new_goal_configuration['aloha_gripper'] = self.grip_range * (grip_width - 0.5)                    


            ##################################################
            # INPUT: x_axis, y_axis, z_axis

            # Use the gripper pose marker's orientation to directly control the robot's wrist yaw, pitch, and roll. 
            aruco_rotation = np.zeros((3, 3))

            aruco_rotation[:,0] = gripper_x_axis
            aruco_rotation[:,1] = gripper_y_axis
            aruco_rotation[:,2] = gripper_z_axis

            r = Rotation.from_matrix(aruco_rotation)
            # capital letters represent intrinsic
            # rotations, lowercase letters represent
            # extrinsic rotations
            ypr = r.as_euler('ZXY', degrees=False)
            
            wrist_yaw = hm.angle_diff_rad(ypr[0] + np.pi, 0.0)
            lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_yaw']
            if (wrist_yaw < lower_limit):
                wrist_yaw = wrist_yaw + (2.0*np.pi)
                
            wrist_pitch = ypr[1]

            wrist_roll = hm.angle_diff_rad(ypr[2] + np.pi, 0.0)

            if self.debug_wrist_orientation: 
                print('___________')
                print('wrist_yaw, wrist_pitch, wrist_roll = {:.2f}, {:.2f}, {:.2f} deg'.format((180.0 * (wrist_yaw/np.pi)),
                                                                                               (180.0 * (wrist_pitch/np.pi)),
                                                                                               (180.0 * (wrist_roll/np.pi))))

            limits_violated = False
            lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_yaw']
            if (wrist_yaw < lower_limit) or (wrist_yaw > upper_limit):
                limits_violated = True
            lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_pitch']
            if (wrist_pitch < lower_limit) or (wrist_pitch > upper_limit):
                limits_violated = True
            lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_roll']
            if (wrist_roll < lower_limit) or (wrist_roll > upper_limit):
                limits_violated = True


            ################################################################
            # DROP GRIPPER ORIENTATION GOALS WITH LARGE JOINT ANGLE CHANGES
            #
            # Dropping goals that result in extreme changes in joint
            # angles over a single time step avoids the nearly 360
            # degree rotation in an opposite direction of motion that
            # can occur when a goal jumps across a joint limit for a
            # joint with a large range of motion like the roll joint.
            #
            # This also reduces the potential for unexpected wrist
            # motions near gimbal lock when the yaw and roll axes are
            # aligned (i.e., the gripper is pointed down to the
            # ground). Goals representing slow motions that traverse
            # near this gimbal lock region can still result in the
            # gripper approximately going upside down in a manner
            # similar to a pendulum, but this results in large yaw
            # joint motions and is prevented at high speeds due to
            # joint angles that differ significantly between time
            # steps. Inverting this motion must also be performed at
            # low speeds or the gripper will become stuck and need to
            # traverse a trajectory around the gimbal lock region.
            #
            extreme_difference_violated = False
            if self.drop_extreme_wrist_orientation_change:
                prev_wrist_yaw = self.prev_commanded_wrist_orientation['joint_wrist_yaw']
                if prev_wrist_yaw is not None:
                    diff = abs(wrist_yaw - prev_wrist_yaw)
                    if diff > self.max_allowed_wrist_yaw_change:
                        print('extreme wrist_yaw change of {:.2f} deg'.format((180.0 * (diff/np.pi))))
                        extreme_difference_violated = True
                prev_wrist_roll = self.prev_commanded_wrist_orientation['joint_wrist_roll']
                if prev_wrist_roll is not None:
                    diff = abs(wrist_roll - prev_wrist_roll)
                    if diff > self.max_allowed_wrist_roll_change:
                        print('extreme wrist_roll change of {:.2f} deg'.format((180.0 * (diff/np.pi))))
                        extreme_difference_violated = True
            #
            ################################################################
                        
            if self.debug_wrist_orientation:
                if limits_violated: 
                    print('The wrist angle limits were violated.')
                    
            if (not extreme_difference_violated) and (not limits_violated):
                new_wrist_orientation = np.array([wrist_yaw, wrist_pitch, wrist_roll])
                    
                # Use exponential smoothing to filter the wrist
                # orientation configuration used to command the
                # robot.
                self.filtered_wrist_orientation = (((1.0 - self.wrist_orientation_filter) * self.filtered_wrist_orientation) +
                                                   (self.wrist_orientation_filter * new_wrist_orientation))
                
                new_goal_configuration['joint_wrist_yaw'] = self.filtered_wrist_orientation[0]
                new_goal_configuration['joint_wrist_pitch'] = self.filtered_wrist_orientation[1]
                new_goal_configuration['joint_wrist_roll'] = self.filtered_wrist_orientation[2]

                self.prev_commanded_wrist_orientation = {'joint_wrist_yaw':self.filtered_wrist_orientation[0],
                                                         'joint_wrist_pitch':self.filtered_wrist_orientation[1],
                                                         'joint_wrist_roll':self.filtered_wrist_orientation[2]}

            # Convert from the absolute goal for the mobile
            # base to an incremental move to be performed
            # using rotate_by. This should be performed just
            # before sending the commands to make sure it's
            # using the most rececnt mobile base angle
            # estimate to reduce overshoot and other issues.

            # convert base odometry angle to be in the range -pi to pi
            # negative is to the robot's right side (counterclockwise)
            # positive is to the robot's left side (clockwise)
            base_odom_theta = hm.angle_diff_rad(self.robot.base.status['theta'], 0.0)
            current_mobile_base_angle = base_odom_theta

            new_goal_configuration['joint_mobile_base_rotation'] = self.filtered_wrist_position_configuration[0]
            new_goal_configuration['joint_mobile_base_rotate_by'] = new_goal_configuration['joint_mobile_base_rotation'] - current_mobile_base_angle
            # remove virtual joint and approximate motion with rotate_by using joint_mobile_base_rotate_by
            del new_goal_configuration['joint_mobile_base_rotation']


            # If motion allowed, command the robot to move to the target configuration
            if self.robot_allowed_to_move:
                if nan_in_configuration(new_goal_configuration): 
                    print()
                    print('******************************************************************')
                    print('WARNING: dex_teleop: new_goal_configuration has a nan, so skipping execution on the robot')
                    print()
                    print('     new_goal_configuration =', new_goal_configuration)
                    print()
                    print('******************************************************************')
                    print()
                else: 
                    self.robot_move.to_configuration(new_goal_configuration, self.joints_allowed_to_move)
                    self.robot.push_command()

            # Print robot status timing stats, if desired.
            if self.print_robot_status_thread_timing: 
                self.robot.non_dxl_thread.stats.pretty_print()
                print()
                self.robot.dxl_end_of_arm_thread.stats.pretty_print()
                print()
                self.robot.dxl_head_thread.stats.pretty_print()
                print()
                self.robot.sys_thread.stats.pretty_print()

            #####################################################


        
if __name__ == '__main__':

    
    args = dt.get_arg_parser().parse_args()
    use_fastest_mode = args.fast
    manipulate_on_ground = args.ground
    left_handed = args.left
    using_stretch_2 = args.stretch_2
    use_multiprocessing = args.multiprocessing

    
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
    
    gripper_to_goal = GripperToGoal(robot_speed, starting_configuration, robot_allowed_to_move, using_stretch_2)
    
    if use_multiprocessing:
        shm = shared_memory.SharedMemory(name=dt.shared_memory_name, create=False)
        example_goal_array = dt.get_example_goal_array()
        received_goal_array = np.ndarray(example_goal_array.shape, dtype=example_goal_array.dtype, buffer=shm.buf)
    else:
        
        goal_grip_width = 1.0
        goal_wrist_position = np.array([-0.03, -0.4, 0.9])
        goal_x_axis = np.array([ 1.0,  0.0, 0.0])
        goal_y_axis = np.array([ 0.0, -1.0, 0.0])
        goal_z_axis = np.array([ 0.0, 0.0, -1.0])
        goal_dict = {'grip_width': goal_grip_width,
                     'wrist_position': goal_wrist_position,
                     'gripper_x_axis': goal_x_axis,
                     'gripper_y_axis': goal_y_axis,
                     'gripper_z_axis': goal_z_axis}

    loop_timer = lt.LoopTimer()
    print_timing = True
    first_goal_received = False
    
    try:
        while True:
            loop_timer.start_of_iteration()
            if use_multiprocessing: 
                if not dt.is_a_do_nothing_goal_array(received_goal_array):
                    goal_dict = dt.goal_array_to_dict(received_goal_array)
                    gripper_to_goal.update_goal(**goal_dict)
                    print('goal_dict =')
                    pp.pprint(goal_dict)
                    if not first_goal_received:
                        loop_timer.reset()
                        first_goal_received = True
                else:
                    print('received do nothing goal array')
            else: 
                gripper_to_goal.update_goal(**goal_dict)
                if not first_goal_received:
                    loop_timer.reset()
                    first_goal_received = True

            time.sleep(0.03)
            loop_timer.end_of_iteration()
            if print_timing:
                loop_timer.pretty_print()
    finally:
        print('cleaning up shared memory multiprocessing')
        shm.close()
        


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
