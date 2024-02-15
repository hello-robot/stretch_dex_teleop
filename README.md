# Dex Teleop for Stretch

| Single Robot Teleoperation                     | Bimanual Teleoperation                      |
| ---------------------------------------------- | ------------------------------------------- |
| ![](/gifs/single_arm_dishes_short_318x360.gif) | ![](/gifs/two_arm_dishes_start_289x360.gif) |

This repository provides code for dexterous teleoperation of the [Stretch 3](https://hello-robot.com/stretch-3-product) mobile manipulator from [Hello Robot](https://hello-robot.com/).

Dex Teleop supports bimanual dexterous teleoperation with Stretch mobile manipulators. As shown in the GIFs above, you can use it to perform efficient multijoint movement with one or two Stretch robots. 

The human operator uses modified kitchen tongs with attached ArUco markers to control the pose of the robot's end effector. A webcam looking up from a stand placed on the ground observes the tongs to estimate the tongs' position, orientation, and grip width. A ring light around the webcam ensures that the ArUco markers can be detected during fast motions by reducing motion blur. 

The system could be adapted to use other interfaces that can provide a six degree of freedom (6 DOF) target pose and grip width at a high rate (e.g., >= 15 Hz). The position of this target pose controls the end of the robot's telescoping arm via inverse kinematics (IK). The orientation of the target pose directly controls the robot's dexterous wrist joints and thereby controls the orientation of the robot's end effector. The grip width commands the robot's gripper.

## Motivation

| Single Robot Teleoperation                     | Bimanual Teleoperation                      |
| ---------------------------------------------- | ------------------------------------------- |
| ![](/gifs/play_with_dog.gif) | ![](/gifs/fold_shirt_cropped_372x270.gif) |

Hello Robot provided code to teleoperate the first version of Stretch (the Stretch RE1) using a gamepad and a web interface. Both types of teleoperation have improved over the years with work from the Stretch community and Hello Robot. Gamepad teleoperation is simple and portable. Web-based teleoperation can be used over great distances and has been made accessible for people with disabilities. A notable disadvantage of these approaches is that they tend to favor slow single joint motions with a single arm.

As shown in the GIFs above, Stretch Dex Teleop enables efficient bimanual multijoint motions. However, compared to gamepad and web-based teleoperation, it is more complex, less accessible, and currently only supports manipulation not navigation.

You can see more bimanual teleoperation examples in a real home with ![this YouTube video](https://www.youtube.com/watch?v=QtG8nJ78x2M).

## Setting Up Dex Teleop

You should start by cloning this repository. All of the commands below should be run from the command line in the root directory of the repository on your robot's computer. 

### Buy or Build Your Interface

You can [buy tongs, a camera, a ring light, and a stand from Hello Robot Inc.](https://hello-robot.com/stretch-dex-teleop-kit)

Alternatively, you can buy the components and build your own by following the [Stretch Dex Teleop Hardware Guide](https://docs.google.com/document/d/1Pom3P8vVNRhchLK_CTduoqQJ3y_TS0MYIIAxRl94ktU/edit?usp=sharing).

You will need a camera, a ring light, and a stand as shown in the following photo. 

<img src="/images/camera_ring_light_and_stand.jpg" width="30%">

For a single robot, you will need a pair of tongs like those shown in the following two photos.

<img src="/images/right_tongs_held_and_open.jpg" width="40%">
<img src="/images/right_tongs_held_and_closed.jpg" width="40%">

For bimanual manipulation, you will need two camera, two ring lights, two stands, left-hand tongs and right-hand tongs.

<img src="/images/left_and_right_tongs.jpg" width="40%">


### Run the Installation Script

After cloning the repository, run the following installation script found in the repository's root directory. 

```
install_dexterous_teleop.sh
```

The installation script sets up a udev rule for a Logitech Webcam C930e, so that the camera can be reset each time you run dexterous teleoperation. This is a workaround to avoid low frame rates and errors in the camera settings.

Next, the installation script installs v4l2 utilities, if necessary.

### Generate Specialized URDFs

To run Dex Teleop, you need to generate specialized URDF files. Dex Teleop uses forward kinematic (FK) and inverse kinematic (IK) models of the robot. These models use specialized URDFs generated from the calibrated URDF on your robot. 

```
python3 prepare_specialized_urdfs.py
```

### Set Up the Camera, Ring Light and Stand

As shown in the photo above, the camera stand should be placed on the ground, and the camera should be pointed straight up. The stand should be at its minimum height. 

The camera should be plugged into the robot's trunk using a USB extension cable. The ring light should not be plugged into the robot's trunk as it requires too much power - it can either be plugged into the robot's head, or externally.

When using the camera, the top of the camera should be pointed away from you. With respect to the robot, the top of the camera points in the direction of arm extension, the lens of the camera looks in the direction of the lift moving up, and the left of the camera points in the direction of the robot's mobile base moving forward. 

### Calibrate the Logitech C930e Webcam

After setting up your camera, you need to calibrate it. 


First, generate a calibration board using the following command: 

```
python3 webcam_calibration_create_board.py
```

This should result in the following PNG file. 

```
webcam_aruco_calibration_board.png
```

Print this image out without scaling it. The resulting printout should match the dimensions specified in the PNG file. 

Mount the resulting printout on a flat surface that you can move around the camera to capture calibration images **with the ring light turned on**. 

Install v4l2 with the following command.

```
sudo apt-get install v4l-utils
```

Use the following command and your calibration pattern to collect calibration images for your Logitech C930e webcam. The entire calibration board should be visible and not too far away, or else the calibration images can lead to errors.

```
python3 webcam_calibration_collect_images.py
```

The images will be stored in the following directory. 

```
./webcam_calibration_images/Logitech Webcam C930e/1920x1080
```

Once you've collected the calibration images, run the following command to process the images. 

```
python3 webcam_calibration_process_images.py
```

Processing the images will generate a YAML calibration file similar to the following file.

```
./webcam_calibration_images/Logitech Webcam C930e/1920x1080/camera_calibration_results_20231211211703.yaml
```

### Test the Camera

To make sure that your camera detects the ArUco markers on your tongs, **turn on the ring light** and run the following code.

```
python3 webcam_teleop_interface.py
```

You should see images from the camera with green boxes drawn around detected ArUco markers. 

## Running Dex Teleop

After you've gotten everything setup, you can try out Dex Teleop. Make sure to start with slow motions, to test your system, gain experience, and warm up. 

### Start with Slow Motions!

After setting everything up, run the following command without any command line arguments. **This will result in the robot moving at the slowest available speed while you ensure that everything is working properly and get used to using the teleoperation system.**

```
python3 dexterous_teleop.py
```

### When You're Ready, Try Fast Motions

Once you are confident that you have the system correctly configured and have learned to use it at the slowest speed, you can run the following command to try it at the fastest available speed. **The robot will move fast, so be very careful!**

```
python3 dexterous_teleop.py --fast
```

### Advanced: Multiprocessing with Shared Memory

To achieve better performance, you can run Dexterous Teleoperation using two processes that communicate via shared memory.

First, run the interface process in a terminal. This process observes ArUco markers with the webcam to create goals for the robot's gripper.

```
python3 goal_from_teleop.py --multiprocessing
```

Second, run the robot process in a different terminal. This process receives gripper goals and attempts to achieve them by controlling the robot. 

```
python3 gripper_to_goal.py --multiprocessing --fast
```



## Acknowledgment

Blaine Matulevich has been extremely helpful throughout the development of Dex Teleop, including testing, providing feedback, discussing the system, and contributing ideas. The entire Hello Robot team provided essential support throughout, including helping with early versions of Stretch 3, which the entire company worked on intensely.
