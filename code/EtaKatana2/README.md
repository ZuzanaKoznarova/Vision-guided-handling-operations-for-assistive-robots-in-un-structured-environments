# EtaKatana

The theoretical part of the thesis concentrates on the topic of assistive robots in health and social care and illustrates possible applications for mobile manipulators in the given area. In the practical part, it processes the proposal of an assistive robot in the form of a vision-guided mobile manipulator. The implementation is simplified on the pick and place task of a detected object by the camera using the Katana 400 manipulator written in ROS.

## Content of the project
* folder /thesis: There is the text of the thesis.
* folder /code: There are two subfolders EtaKatana2 where there is the
program of the whole project described in Doxygen documentation and
README.md. The ROS based implementation is written in C++ and
consists of vision and robot package for better modularity. The folder
models includes the proposal for the mobile manipulator, in a form of
the 3D model and calibration the ball. 
* folder /attachment/videos: There are the example videos of the pick and
place task for a box with medicaments and a can.

## Getting started

### Needed instalations:
Please follow these tutorials: 
* ROS instalation EtaBot/Dokumentation/html/a00036.html
* [Katana packages instalation](http://wiki.ros.org/katana_driver)
* Kinect packages instalation Etabot/Dokumentation/html/a00132.html

The kinect bridge can need additionally arguments

```
rosrun kinect2_bridge kinect2_bridge _depth_method:=cpu _reg_method:=cpu
```
Moreover kinect need to be connected in USB3. 
* [Opencv instalation](https://docs.opencv.org/3.3.1/d7/d9f/tutorial_linux_install.html)  programmed on OpenCV 3.3.1 
* [Darknet instalation](https://pjreddie.com/darknet/install/) for recognition of general objects 

### Needed configurations:

For connecting Katana robot using a USB cable with a computer is needed to set the computer IP address to 192.168.0.1. Katana will be found on IP address 192.168.1.1 or 192.168.168.232. In order to connect Katana robot using an Ethernet cable it is needed to set the computer IP address to 192.168.168.200. Also, Katana will be found on address 192.168.168.232.
Then set Katana type by typing following line in comand line.
```
export KATANA_TYPE="katana_450_6m180"

```
## Compilation
For compilation of the program EtaKatana go to the folder /EtaKatana2 and
type
```
catkin_make
```
If the compilation is unsuccessful because of missing header files, comment compilation of all nodes in CMakeLists.txt in package vision_part and also in package robot_part and compile the project again using the following command.
```
catkin_make install
```
Then uncomment compilation of all nodes and compile again using
```
catkin_make
```
For compilation of the robot_part package documentation go to the folder
/EtaKatana2/src/robot_part and type the following command.
```
rosdoc_lite .
```
Repeat the same procedure for compilation of the vision_part package documentation.
## Example of running parts of implemetation
### Pick and place
The pick and place task works for grasping boxes and cylindrical objects. Cylindrical objects are grasped horizontally and boxes vertically to the surface. It includes multiple subparts of the vision and also robot part. In order to run this task it is necessary to launch all of the folowing comands, if the files do not run already. Moreover, make sure that roscore was run as the firstone. When you run calibration after pick and place or backwards, please run the go_home before to avoid collision.

```
roscore
rosrun kinect2_bridge kinect2_bridge _depth_method:=cpu _reg_method:=cpu
roslaunch katana katana.launch
rosrun robot_part my_inverse_kinematic
rosrun robot_part my_forward_kinematic
rosrun vision_part depth_on_RGB_coordinates
rosrun vision_part recognize_and_show_object
rosrun robot_part pick_and_place circles 1050 867 # example for horizontal grasping, run this or next line not both together
rosrun robot_part pick_and_place ibuflam 715 960 #example for vertical grasping
```

### Calibration
Do the calibration again every time the position of the robot or the position of the camera are changed, or when it seems that the robot is less precise then before.In case of minimum changes in the position of the robot and camera no code changes will be necessary. In case of a different camera robot position it will be necessary to change the calibration points in the file calibration.cpp (variable cv::Mat_<double> end_effector_position_matrix) to be in the camera FoV. When you run calibration after pick and place or backwards, please run the go_home before to avoid collision.

```
roscore
rosrun kinect2_bridge kinect2_bridge _depth_method:=cpu _reg_method:=cpu
roslaunch katana katana.launch
rosrun robot_part my_inverse_kinematic
rosrun robot_part my_forward_kinematic
rosrun vision_part recognize_and_show_object circles
rosrun vision_part calibration 1 #testing
rosrun vision_part calibration #training
```
### Detecting an object
Commands to detect an object.
```
roscore
rosrun kinect2_bridge kinect2_bridge _depth_method:=cpu _reg_method:=cpu
rosrun vision_part recognize_and_show_object ibuflam #if without argument then circles otherwise it will choose the detection method depen
```

### Go home
Commands to send the robot to the home position, for example to have a good point of view.

```
roscore
roslaunch katana katana.launch
rosrun robot_part my_inverse_kinematic
rosrun robot_part go_home
```

## Adding new object in database

To add a new picture it is needed to make a photo of the object, downscale it to about ten per cent of the camera resolution and insert it in folder relative to the project EtaKatana2 /data/user_defined_objects. After that it is necessary to write the name of the object to /data/user_ defined_objects/user_defined_objects.names and then write the object size in z-axis in centimetres into the file measurements.ext in the form name: double_number. Then it is possible to recognise the object and also grasp the object if it will lay up with the photographed side.

## Outlook
### Object detection
A beneficial next step can be recalculation of the depth map data for the occupancy grid. That step enables calculating a trajectory with the collisions avoidance, for example by using rapidly-exploring random tree (RRT) algorithm.
### Calibration
Other variables that can be tested and can bring further accuracy improvement are definition of calibration points in a way that they covered the expected robot working area as much as possible (distribute more calibration points near the surface). Next possibility for further testing can be to gradually increase the number of uniformly distributed calibration points and to monitor when the number of points means improvement which is significant enough in comparison to the time necessary to perform the calibration task. The third possible improvement can be calculation of the internal camera calibration.
### Pick and place
The first point that can be solved is the automatic finding of the object size in z-axis. At first sight, it seems to be enough to read the depth from the depth map in the surroundings of the object. However, it can be enough only if the object is the only one on a flat surface. If we also consider other objects lying next to the detected object, it is probable that the object z size will be detected incorrectly. The next idea can be the saved value of the distance from the basic surface (table, ground) or taking of the most common non-zero value from the captured image and setting it as the basic surface value. That can be highly problematic when there are objects lying on other objects. The situation is even more complicated for the convex and concave objects. The first mentioned situation would result in collision with the surface during the placing, which can have harmful consequences for the robot, surface and also for the object. In the second case, the object will be dropped from height, which can have adverse consequences especially for the object6. For both of the above mentioned reasons, a fixed set value was used for each object. Two possible solutions are offered as an outlook for further developers.
* Katana can grasp the object and lift it in safe distance from the surface and other objects and rotate the object and then reconstruct a 3D model from measured data. Moreover, the 3D model information can also be saved and used to learn effective object grasping. This solution can be dangerous for liquids.
* Another possible implementation is to use tactile and force sensors and with them to detect the change of force that acts on the object and the gripper.

The next and colossal step can be to create an automatic prediction of the grasping positions for any shape of the object.
### User interface
Next task cane be to program the user interface using communication based on ROS and using Bluetooth. Another task is to implement a sound output with possible extension to a chatbot.


### Where to find more detailed informations
Master Thesis: Vision-guided handling operations for assistive robots in unstructured environments (Author: Zuzana Kožnarová, Supervisors: Prof. Dr.-Ing. Hartmut Bruhm and doc. Ing. Tomáš Svoboda Ph.D.).

## Authors (contact emails):
Zuzana Kožnarová (zuzanazofka@gmail.com)

## Acknowledgments
I would like to express my gratitude to my supervisors Prof. Dr.-Ing. Hartmut Bruhm and doc. Ing. Tomáš Svoboda Ph.D. for relevant feedback to my master thesis. Many thanks belong to my family that supports me not only during the last five years at the university, but also in the whole of my life. I would like to thank my partner Jakub Havlícek and my friends for their support.

