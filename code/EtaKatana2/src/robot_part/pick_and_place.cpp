/** @file pick_and_place.cpp
 *  @brief Pick the user defined object or cylindrical object on recognized position with recognized orientation and recognized size and place it on by the user defined position. Example can: rosrun robot_part pick_and_place circles 1050 867 Example medicaments: rosrun robot_part pick_and_place ibuflam 715 960
 * @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include <ros/ros.h>
#include <robot_part/service_end_effector_position_with_orientation.h>
#include <vision_part/get_depth.h>
#include <cstdlib>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include "control_msgs/GripperCommandActionGoal.h"
#include "katana_msgs/JointMovementActionGoal.h"
#include "katana_msgs/JointMovementActionResult.h"
#include <vision_part/hough_circle_position.h>
#include <vision_part/hough_circle_position_with_diameter.h>
#include "control_msgs/GripperCommandActionResult.h"
#include "control_msgs/JointTrajectoryActionGoal.h"
#include "control_msgs/JointTrajectoryActionResult.h"
#include <time.h>
#include <std_msgs/Float64.h>
#include "../vision_part/include/vision_part/camera_parameters.h"
#include <vision_part/area_of_recognized_object.h>
#include "vision_part/object_to_be_recognised.h"
#include <stdlib.h>


#define _USE_MATH_DEFINES

#define PENDING 0
#define ACTIVE 1
#define PREEMPTED 2
#define SUCCEEDED 3
#define ABORTED 4
#define REJECTED 5
#define PREEMPTING 6
#define RECALLING 7
#define RECALLED 8
#define LOST 9


//gripper coeficients definitions
#define A_GRIPPER 0.002003343
#define B_GRIPPER 0.05169325
#define C_GRIPPER -0.5733983
//define gripper range
#define MAX_GRIPPER 0.3
#define MIN_GRIPPER -0.44

int circles;
double high_of_the_object;



cv::Mat vision_position_vector;
void change_of_position(cv::Mat_<double> object_in_robot_coordinate_system, ros::NodeHandle nh, double loop_rate_movements);
void print_status_info(int status);
bool colision_avoidance(cv::Mat_<double> object_in_robot_coordinate_system);

/**
 * @brief The trajectory struct is structure of preparing position, position and after position with value of gripper on preposition and after approaching position.
 */
struct trajectory {
    cv::Mat_<double> preposition;
    cv::Mat_<double> position;
    cv::Mat_<double> afterposition;
    double gripper_pre;
    double gripper_after;
} ;

/**
 * @brief distance Calculate distance in 3D between point_A and point_B. Expected format of points cv::Mat_<double>.
 * @param A_point point in 3D
 * @param B_point point in 3D
 * @return distance in 3D
 */
double distance(cv::Mat_<double> A_point,cv::Mat_<double> B_point){
    double dist=sqrt(pow(B_point.at<double>(cv::Point(0, 0))-A_point.at<double>(cv::Point(0, 0)),2)+pow(B_point.at<double>(cv::Point(0, 1))-A_point.at<double>(cv::Point(0, 1)),2)+pow(B_point.at<double>(cv::Point(0, 2))-A_point.at<double>(cv::Point(0, 2)),2));
    return dist;
}

/**
 * @brief distance_points Calculate distance in 2D between point a and b. Expected format of points cv::Point_<float>.
 * @param a 2D point a
 * @param b 2D point b
 * @return distance in 2D
 */
float distance_points(cv::Point_<float> a,cv::Point_<float> b){
    float d=(float)sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    return d;
}

/**
 * @brief move_linearly Move between point A and point B using small steps, currently not used, can be modified with calculation of speed and accelerations. Now for linear movement is used move_on_trajectory.
 * @param A_point start point
 * @param B_point end point
 * @param step_size size of the step
 * @param nh node handle
 */
void move_linearly(cv::Mat_<double> A_point,cv::Mat_<double> B_point,double step_size, ros::NodeHandle nh){
    double dist= distance(A_point,B_point);
    int number_of_steps=(int)(dist/step_size);
    cv::Mat_<double> tmp_position=A_point.clone();
    for(int i=1; i<number_of_steps;i++){
        tmp_position.at<double>(cv::Point(0, 0))=A_point.at<double>(cv::Point(0, 0))+(B_point.at<double>(cv::Point(0, 0))-A_point.at<double>(cv::Point(0, 0)))*i/number_of_steps;
        tmp_position.at<double>(cv::Point(0, 1))=A_point.at<double>(cv::Point(0, 1))+(B_point.at<double>(cv::Point(0, 1))-A_point.at<double>(cv::Point(0, 1)))*i/number_of_steps;
        tmp_position.at<double>(cv::Point(0, 2))=A_point.at<double>(cv::Point(0, 2))+(B_point.at<double>(cv::Point(0, 2))-A_point.at<double>(cv::Point(0, 2)))*i/number_of_steps;
        change_of_position(tmp_position, nh,30);
    }
    change_of_position(B_point, nh,3);
    return;
}
/**
 * @brief cm_to_gripper Recalculate the distance between gripper fingers to the gripper value, by using the gripper charakteristic.
 * @param x distance in cm
 * @return gripper value
 */
double cm_to_gripper(double x){
    double y=A_GRIPPER*pow(x,2)+B_GRIPPER*x+C_GRIPPER;
    if(y<MIN_GRIPPER || y>MAX_GRIPPER){
        ROS_WARN("Calculated gripper value is out of gripper range, the lower limit is: %lf, calculated value is: %lf, upper limit is: %lf.", MIN_GRIPPER, y, MAX_GRIPPER);
        return -100;
    }else{
        ROS_INFO("Gripper will be open on value: %lf, that corresponds to the value %lf cm.",y,x);
    }
    return y;
}

/**
 * @brief move_on_trajectory Moving on the trajectory from point A to point B.
 * Now the movement is linear, because only the points in 3D space are set. Can be modified to the Cubic movement when be defined also the velocities or to the Quitic movement by adding velocities and also accelerations.
 * @param A_point start point
 * @param B_point end point
 * @param step_size size of the steps between subpoints, currently not used.
 * @param nh node handle
 */
void move_on_trajectory(cv::Mat_<double> A_point,cv::Mat_<double> B_point,double step_size, ros::NodeHandle nh){
    double dist= distance(A_point,B_point);
    int number_of_steps=1;//(int)(dist/step_size);
    cv::Mat_<double> object_in_robot_coordinate_system=A_point.clone();
    control_msgs::JointTrajectoryActionGoal msg3;
    ros::Publisher chatter_pub_joint;
    int count = 0;
    chatter_pub_joint = nh.advertise<control_msgs::JointTrajectoryActionGoal>("/katana_arm_controller/joint_trajectory_action/goal", 10000);
    msg3.goal.trajectory.joint_names.push_back("katana_motor1_pan_joint");
    msg3.goal.trajectory.joint_names.push_back("katana_motor2_lift_joint");
    msg3.goal.trajectory.joint_names.push_back("katana_motor3_lift_joint");
    msg3.goal.trajectory.joint_names.push_back("katana_motor4_lift_joint");
    msg3.goal.trajectory.joint_names.push_back("katana_motor5_wrist_roll_joint");
    for(int i=0; i<number_of_steps+1;i++){
        object_in_robot_coordinate_system.at<double>(cv::Point(0, 0))=A_point.at<double>(cv::Point(0, 0))+(B_point.at<double>(cv::Point(0, 0))-A_point.at<double>(cv::Point(0, 0)))*i/number_of_steps;
        object_in_robot_coordinate_system.at<double>(cv::Point(0, 1))=A_point.at<double>(cv::Point(0, 1))+(B_point.at<double>(cv::Point(0, 1))-A_point.at<double>(cv::Point(0, 1)))*i/number_of_steps;
        object_in_robot_coordinate_system.at<double>(cv::Point(0, 2))=A_point.at<double>(cv::Point(0, 2))+(B_point.at<double>(cv::Point(0, 2))-A_point.at<double>(cv::Point(0, 2)))*i/number_of_steps;
        if(colision_avoidance(object_in_robot_coordinate_system)){
            ros::ServiceClient client = nh.serviceClient<robot_part::service_end_effector_position_with_orientation>("robot_part/service_end_effector_position_with_orientation");
            robot_part::service_end_effector_position_with_orientation srv;
            srv.request.x = object_in_robot_coordinate_system.at<double>(cv::Point(0, 0));
            srv.request.y = object_in_robot_coordinate_system.at<double>(cv::Point(0, 1));
            srv.request.z = object_in_robot_coordinate_system.at<double>(cv::Point(0, 2));
            srv.request.roll =  object_in_robot_coordinate_system.at<double>(cv::Point(0, 3));
            srv.request.pitch = object_in_robot_coordinate_system.at<double>(cv::Point(0, 4));
            srv.request.yaw = object_in_robot_coordinate_system.at<double>(cv::Point(0, 5));
            if (client.call(srv)){
                if (srv.response.katana_motor3_lift_joint.size()!=0){
                    bool position_change=true;
                    if(position_change){
                        //Robot
                        double eff =0.1;
                        double vel =0.1;
                        double aclr =0.1;
                        trajectory_msgs::JointTrajectoryPoint point;
                        point.positions.push_back(srv.response.katana_motor1_pan_joint.at(0)); // before were r
                        point.positions.push_back(srv.response.katana_motor2_lift_joint.at(0));
                        point.positions.push_back(srv.response.katana_motor3_lift_joint.at(0));
                        point.positions.push_back(srv.response.katana_motor4_lift_joint.at(0));
                        point.positions.push_back(srv.response.katana_motor5_wrist_roll_joint.at(0));
                        for(int j=0; j<5;j++){
                            //point.effort.push_back(eff);
                            if(i>=(double)number_of_steps/2.0){
                                //point.velocities.push_back(1.0-((double)i+1.0)/(double)(number_of_steps));
                            }else{
                                //point.velocities.push_back((double)i/(double)(number_of_steps));
                            }
                        }
                        point.time_from_start=ros::Duration(((double)i));
                        msg3.goal.trajectory.points.push_back(point);
                    }else{
                        ROS_INFO("Robot is already on questioned position, because of that robot is not moving.");
                    }
                }else{
                    ROS_WARN("Out of robot range");
                }
            }else{
                ROS_WARN("Failed to call service my_inverse_kinematic");
            }
        }else{
            ROS_WARN_STREAM("Colision denger on the position: "<< object_in_robot_coordinate_system.at<double>(cv::Point(0, 0))<<", "<<object_in_robot_coordinate_system.at<double>(cv::Point(0, 1))<<", "<<object_in_robot_coordinate_system.at<double>(cv::Point(0, 2)));
        }
    }
    while (ros::ok()){
        ros::Rate poll_rate(1000); //to make sure that the message will be published once

        const clock_t begin_time = clock();
        while(chatter_pub_joint.getNumSubscribers() == 0){
            poll_rate.sleep();
            // do something
            float elapset_time =  float( clock () - begin_time ) /  CLOCKS_PER_SEC;
            if(elapset_time>10){
                ROS_WARN("The waiting for the subcribes take more then 10 seconds, the changing of the robot position was canceled, please make sure, that the robot and roscore running.");
                return;
            }
        }
        chatter_pub_joint.publish(msg3);
        ROS_INFO("Trajectory was published.");

        //TODO: change for the trajectory
        boost::shared_ptr<control_msgs::JointTrajectoryActionResult const> result_of_the_movement_pointer;
        result_of_the_movement_pointer = ros::topic::waitForMessage<control_msgs::JointTrajectoryActionResult>("/katana_arm_controller/joint_trajectory_action/result", nh,ros::Duration(10,0)); // could be problem, wenn the robot is already in the final point
        if(!result_of_the_movement_pointer){
            ROS_WARN("No status trajectory");
        }else{
            control_msgs::JointTrajectoryActionResult  result_of_the_movement_object= *result_of_the_movement_pointer;
            uint8_t status=result_of_the_movement_object.status.status;
            print_status_info(status);
        }
        double loop_rate_movements=0.5;
        ros::Rate loop_rate(loop_rate_movements);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if(count>0){
            break;
        }
    }
}
/**
 * @brief colision_avoidance Check if the robot is not to close to the surface.
 * @param object_in_robot_coordinate_system coordinates of the end effector position
 * @return If true then the movement position is without collision.
 */
bool colision_avoidance(cv::Mat_<double> object_in_robot_coordinate_system){
    //basic avoidance only control the z axes
    bool avoidance= false;
    if( object_in_robot_coordinate_system.at<double>(cv::Point(0, 2))>0.04){
        avoidance=true;
    }
    return avoidance;
}

/**
 * @brief print_status_info Print a status after finishing a movement or if some problems have occurred during movement, for example colision.
 * @param status
 */
void print_status_info(int status){
    if(status==SUCCEEDED){
        ROS_INFO("Movement was sucessfully finished.");
    }else{
        switch (status) {
        case PENDING:
            ROS_WARN("Movement was pending, check if the robotic arm is active.");
            break;
        case ACTIVE:
            ROS_WARN("Movement is not finished yet.");
            break;
        case PREEMPTED:
            ROS_WARN("movement is preempted, can be a problem with time stamp or with other publisher for same topic.");
            break;
        case ABORTED:
            ROS_WARN("Movement was aborted, check if there was no collision.");
            break;
        case REJECTED:
            ROS_WARN("Movement was rejected.");
            break;
        case  PREEMPTING:
            ROS_WARN("Movement finished with problematic status preempting, can be a problem with time stemp of the message.");
            break;
        case  RECALLING:
            ROS_WARN("Movement finished with problematic status recalling, check if there are not sent same messages more then once.");
            break;
        case RECALLED:
            ROS_WARN("Movement finished with problematic status recalled.");
            break;
        case  LOST:
            ROS_WARN("Movement finished with problematic status lost.");
            break;
        default:
            ROS_WARN("Movement finished with unknown status, for more information check message katana_msgs/JointMovementActionResult.");
            break;
        }
    }
}

/**
 * @brief control_position Control the colision avoidance and then calculate if exist solution in range of the robot joints.
 * @param object_in_robot_coordinate_system end effector position with orientation
 * @param nh node handle
 * @return rotations of the joints
 */
int control_position(cv::Mat_<double> object_in_robot_coordinate_system, ros::NodeHandle nh){
    if(colision_avoidance(object_in_robot_coordinate_system)){
        ros::ServiceClient client = nh.serviceClient<robot_part::service_end_effector_position_with_orientation>("robot_part/service_end_effector_position_with_orientation");
        robot_part::service_end_effector_position_with_orientation srv;
        srv.request.x = object_in_robot_coordinate_system.at<double>(cv::Point(0, 0));
        srv.request.y = object_in_robot_coordinate_system.at<double>(cv::Point(0, 1));
        srv.request.z = object_in_robot_coordinate_system.at<double>(cv::Point(0, 2));
        srv.request.roll =  object_in_robot_coordinate_system.at<double>(cv::Point(0, 3));
        srv.request.pitch = object_in_robot_coordinate_system.at<double>(cv::Point(0, 4));
        srv.request.yaw = object_in_robot_coordinate_system.at<double>(cv::Point(0, 5));
        if (client.call(srv))
        {
            ros::Publisher chatter_pub_joint = nh.advertise<katana_msgs::JointMovementActionGoal>("/katana_arm_controller/joint_movement_action/goal", 10000);
            if (srv.response.katana_motor3_lift_joint.size()!=0){
                return 0;
            }else{
                ROS_WARN("Out of robot range");
                return -1;
            }
        }else{
            ROS_ERROR("Failed to call service my_inverse_kinematic, please check if the node my_inverse_kinematic in package robot_part is running. If not, type in commandline rosrun robot_part my_inverse_kinematic .");
            return -2;
        }
    }else{
        ROS_WARN_STREAM("Colision denger on the position: "<< object_in_robot_coordinate_system.at<double>(cv::Point(0, 0))<<", "<<object_in_robot_coordinate_system.at<double>(cv::Point(0, 1))<<", "<<object_in_robot_coordinate_system.at<double>(cv::Point(0, 2)));
        return -3;
    }
    return 0;
}

/**
 * @brief change_of_position Control the colision avoidance and then calculate if exist solution in range of the robot joints then send the robot to the position.
 * @param object_in_robot_coordinate_system object position in robot coordinate system
 * @param nh node handle
 * @param loop_rate_movements Currently not used, can be used for waiting until will be the movement finished, currently used waiting for result message katana_arm_controller/joint_movement_action/result.
 */
void change_of_position(cv::Mat_<double> object_in_robot_coordinate_system, ros::NodeHandle nh, double loop_rate_movements){
    if(colision_avoidance(object_in_robot_coordinate_system)){
        ros::ServiceClient client = nh.serviceClient<robot_part::service_end_effector_position_with_orientation>("robot_part/service_end_effector_position_with_orientation");
        robot_part::service_end_effector_position_with_orientation srv;
        srv.request.x = object_in_robot_coordinate_system.at<double>(cv::Point(0, 0));
        srv.request.y = object_in_robot_coordinate_system.at<double>(cv::Point(0, 1));
        srv.request.z = object_in_robot_coordinate_system.at<double>(cv::Point(0, 2));
        srv.request.roll =  object_in_robot_coordinate_system.at<double>(cv::Point(0, 3));
        srv.request.pitch = object_in_robot_coordinate_system.at<double>(cv::Point(0, 4));
        srv.request.yaw = object_in_robot_coordinate_system.at<double>(cv::Point(0, 5));
        if (client.call(srv))
        {
            ros::Publisher chatter_pub_joint = nh.advertise<katana_msgs::JointMovementActionGoal>("/katana_arm_controller/joint_movement_action/goal", 10000);
            if (srv.response.katana_motor3_lift_joint.size()!=0){
                bool position_change=true;
                if(position_change){
                    int count = 0;
                    while (ros::ok())
                    {
                        //Robot
                        double eff =0.1;
                        double vel =0.1;
                        katana_msgs::JointMovementActionGoal msg3;
                        //ROS_INFO("Moving to the next position.");
                        msg3.goal.jointGoal.name.push_back("katana_motor1_pan_joint");
                        msg3.goal.jointGoal.position.push_back(srv.response.katana_motor1_pan_joint.at(0)); // before were r
                        msg3.goal.jointGoal.effort.push_back(eff);
                        msg3.goal.jointGoal.velocity.push_back(vel);
                        msg3.goal.jointGoal.name.push_back("katana_motor2_lift_joint");
                        msg3.goal.jointGoal.position.push_back(srv.response.katana_motor2_lift_joint.at(0));
                        msg3.goal.jointGoal.effort.push_back(eff);
                        msg3.goal.jointGoal.velocity.push_back(vel);
                        msg3.goal.jointGoal.name.push_back("katana_motor3_lift_joint");
                        msg3.goal.jointGoal.position.push_back(srv.response.katana_motor3_lift_joint.at(0));
                        msg3.goal.jointGoal.effort.push_back(eff);
                        msg3.goal.jointGoal.velocity.push_back(vel);
                        msg3.goal.jointGoal.name.push_back("katana_motor4_lift_joint");
                        msg3.goal.jointGoal.position.push_back(srv.response.katana_motor4_lift_joint.at(0));
                        msg3.goal.jointGoal.effort.push_back(eff);
                        msg3.goal.jointGoal.velocity.push_back(vel);
                        msg3.goal.jointGoal.name.push_back("katana_motor5_wrist_roll_joint");
                        msg3.goal.jointGoal.position.push_back(srv.response.katana_motor5_wrist_roll_joint.at(0));
                        msg3.goal.jointGoal.effort.push_back(eff);
                        msg3.goal.jointGoal.velocity.push_back(vel);
                        //ros::Rate poll_rate(100); //to make sure that the message will be published once

                        const clock_t begin_time = clock();
                        while(chatter_pub_joint.getNumSubscribers() == 0){
                            //        poll_rate.sleep();
                            float elapset_time =  float( clock () - begin_time ) /  CLOCKS_PER_SEC;
                            if(elapset_time>10){
                                ROS_WARN("The waiting for the subcribes take more then 10 seconds, the changing of the robot position was canceled, please make sure, that the robot and roscore running.");
                                return;
                            }
                        }
                        chatter_pub_joint.publish(msg3);
                        boost::shared_ptr<katana_msgs::JointMovementActionResult const> result_of_the_movement_pointer;
                        result_of_the_movement_pointer = ros::topic::waitForMessage<katana_msgs::JointMovementActionResult>("katana_arm_controller/joint_movement_action/result", nh,ros::Duration(10,0)); // could be problem, wenn the robot is already in the final point
                        if(!result_of_the_movement_pointer){
                            ROS_WARN("No status joint");
                        }else{
                            katana_msgs::JointMovementActionResult  result_of_the_movement_object= *result_of_the_movement_pointer;
                            uint8_t status=result_of_the_movement_object.status.status;
                            print_status_info(status);
                        }
                        ros::spinOnce();
                        ++count;
                        if(count>0){
                            break;
                        }
                    }
                }else{
                    ROS_INFO("Robot is already on questioned position, because of that robot is not moving.");
                }
            }else{
                ROS_WARN("Out of robot range");
            }
        }else{
            ROS_ERROR("Failed to call service my_inverse_kinematic, check if the service my_inverse_kinematic from package robot_part is running and if there exist a solution for the end effector possition.");
            return;
        }
        return;
    }else{
        ROS_WARN_STREAM("Colision denger on the position: "<< object_in_robot_coordinate_system.at<double>(cv::Point(0, 0))<<", "<<object_in_robot_coordinate_system.at<double>(cv::Point(0, 1))<<", "<<object_in_robot_coordinate_system.at<double>(cv::Point(0, 2)));
    }
}

/**
 * @brief change_of_gripper_setting Change of the gripper value, open or close gripper on some value.
 * @param gripper_position gripper value already in the gripper units
 * @param nh node handle
 */
void change_of_gripper_setting(double gripper_position, ros::NodeHandle nh){ //range: [-0.440000, 0.300000]
    ros::Publisher chatter_pub = nh.advertise<control_msgs::GripperCommandActionGoal>("/gripper_grasp_posture_controller/goal", 1000);
    ros::Rate loop_rate(1);
    int count=0;
    while (ros::ok())
    {
        control_msgs::GripperCommandActionGoal msg2;
        msg2.goal.command.position = gripper_position; //-0.35 for grasping can
        msg2.goal.command.max_effort = 0.9;
        ros::Rate poll_rate(100);
        const clock_t begin_time = clock();
        while(chatter_pub.getNumSubscribers() == 0){
            poll_rate.sleep();
            float elapset_time =  float( clock () - begin_time ) /  CLOCKS_PER_SEC;
            if(elapset_time>10){
                ROS_WARN("The waiting for the subcribes take more then 10 seconds, the changing of the griper position was canceled, please make sure, that the robot and roscore running.");
                return;
            }
        }
        chatter_pub.publish(msg2);
        boost::shared_ptr<control_msgs::GripperCommandActionResult const> result_of_the_movement_pointer;
        result_of_the_movement_pointer = ros::topic::waitForMessage<control_msgs::GripperCommandActionResult>("/gripper_grasp_posture_controller/result", nh,ros::Duration(10,0)); // could be problem, wenn the robot is already in the final point
        if(!result_of_the_movement_pointer){
            ROS_WARN("No status gripper, control if the computer is connected with Katana controlbox");
        }else{
            control_msgs::GripperCommandActionResult  result_of_the_movement_object= *result_of_the_movement_pointer;
            uint8_t status=result_of_the_movement_object.status.status;
            print_status_info(status);
        }
        ros::spinOnce();
        ++count;
        if(count>0){
            break;
        }
    }
    return;
}

/**
 * @brief find_depth_for_grasping_positions Not finished yet. Is planned to calculate the z size of the object.
 * @param a left upper corner of detected area
 * @param b right upper corner of detected area
 * @param c right lower corner of detected area
 * @param d left lower corner of detected area
 */
void find_depth_for_grasping_positions(cv::Point2d a,cv::Point2d b,cv::Point2d c,cv::Point2d d){
    double d1=(distance_points(a,b)+distance_points(c,d))/2;
    double d2=(distance_points(b,c)+distance_points(d,a))/2;
    cv::Point2d e;
    cv::Point2d f;
    cv::Point2d g;
    cv::Point2d h;
    if(d1>=d2){
        e = a;
        f = b;
        g = c;
        h = d;
    }else{
        e = b;
        f = c;
        g = d;
        h = a;
    }
    cv::Point2d middle_e_f=(e+f)/2;

    // posible calculation of the depth of the object, if we expected that the object lay directly on the surface without other objects close next to it
    // calculate the perpendicular line to the e-f that starts in the middle_e_f
    // ask on the depth on the position on the perpendicular line at some milimiters far away from the middle
    // repeat for the g-h
    // choose lower value
    // substract the depth found in the middle of the object and next to the object
    // with that I get the depth of the object
    // choose in which depth of the object I want to grasp it
}

/**
 * @brief calculate_pick_positions Calculate the pregrasping and aftergrasping position from the grasping position.
 * @param object_in_robot_coordinate_system Coordinates with rotations of the endeffector.
 * @param object_size object size in cm.
 * @return Structure with the pick trajectory. Pregrasping point, aftergrasping point, grasping point, gripper value before grasping and gripper value after grasping.
 */
trajectory calculate_pick_positions(cv::Mat_<double> object_in_robot_coordinate_system,double object_size){
    trajectory pick_trajectory;
    double gripper_close=cm_to_gripper(object_size*100-1.9); //y -0.33
    if(gripper_close==-100){
        ROS_ERROR("Object can not be grasped because gripper is value is out of range. Program ends.");
        return pick_trajectory;
    }
    double gripper_open=MAX_GRIPPER;//maximal open, for grasping on dificult position, need to be also set dinamically
    //Calculate for 3D diagonal depending on the input angle and not only the 2D diagonal
    cv::Mat_<double> pre_grasping_in_robot_coordinate_system=object_in_robot_coordinate_system.clone();
    double object_diameter_xyz=object_size;//possibly will be taken from hough circles and sendet to this function
    double object_diameter_z=std::sin(object_in_robot_coordinate_system.at<double>(cv::Point(0, 4)))*object_diameter_xyz;
    double object_diameter_xy=std::cos(object_in_robot_coordinate_system.at<double>(cv::Point(0, 4)))*object_diameter_xyz;
    double old_distance= sqrt(pow(object_in_robot_coordinate_system.at<double>(cv::Point(0, 0)),2)+pow(object_in_robot_coordinate_system.at<double>(cv::Point(0, 1)),2)); //distance between gripper and origin of robot coordinate system
    double new_distance=old_distance-object_diameter_xy;
    double alpha=atan2(object_in_robot_coordinate_system.at<double>(cv::Point(0, 1)),object_in_robot_coordinate_system.at<double>(cv::Point(0, 0)));//yaw angle
    pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0, 0))=new_distance*cos(alpha);
    pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,1))=new_distance*sin(alpha);
    double z=pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,2));
    if(circles==1){
        pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))=pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))+object_diameter_z-high_of_the_object/3; //0.03 for savety
        object_in_robot_coordinate_system.at<double>(cv::Point(0,2))=object_in_robot_coordinate_system.at<double>(cv::Point(0,2))-high_of_the_object/3; //0.03 for savety
    }else{
        pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))=pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))+object_diameter_z+0.005; //0.03 for savety
        object_in_robot_coordinate_system.at<double>(cv::Point(0,2))=object_in_robot_coordinate_system.at<double>(cv::Point(0,2))+0.005;
    }
    if(circles!=1){
        pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,3))=pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,3))+alpha;//alpha;
    }
    object_in_robot_coordinate_system.at<double>(cv::Point(0,3))=pre_grasping_in_robot_coordinate_system.at<double>(cv::Point(0,3));//alpha;
    cv::Mat_<double> after_grasping_in_robot_coordinate_system=object_in_robot_coordinate_system.clone();
    after_grasping_in_robot_coordinate_system.at<double>(cv::Point(0, 2))=after_grasping_in_robot_coordinate_system.at<double>(cv::Point(0, 2))+0.02;
    pick_trajectory.preposition=pre_grasping_in_robot_coordinate_system;
    pick_trajectory.position=object_in_robot_coordinate_system;
    pick_trajectory.afterposition=after_grasping_in_robot_coordinate_system;
    pick_trajectory.gripper_pre=gripper_open;
    pick_trajectory.gripper_after=gripper_close;
    return pick_trajectory;
}

/**
 * @brief go_on_position Change gripper value on gripper_pre value, move point to point to the pre position, move linearly to the position, change the gripper value on gripper_after position, move linearly to the after position.
 * @param tr trajectory
 * @param nh node handle
 * @return not used now plan to return if the performing of trajectory was succesful
 */
bool go_on_position(trajectory tr, ros::NodeHandle nh){
    change_of_gripper_setting(tr.gripper_pre, nh);
    change_of_position(tr.preposition, nh,0.5);
    move_on_trajectory(tr.preposition,tr.position,0.01,nh); //space for improvement
    change_of_position(tr.position, nh,0.5);
    //move_linearly(tr.preposition,tr.position,0.003,nh); //space for improvement
    //change_of_position(tr.position, nh,0.5);
    change_of_gripper_setting(tr.gripper_after, nh);
    //change_of_position( tr.afterposition, nh,0.5);
    move_on_trajectory(tr.position,tr.afterposition,0.01,nh); //space for improvement
    return true;
}

/**
 * @brief calculate_place_positions Calculate the preplaceing and afterplaceing position from the placeing position.
 * @param object_in_robot_coordinate_system Coordinates with rotations of the endeffector.
 * @param object_size object size in cm.
 * @return Structure with the place trajectory. Preplaceing point, afterplaceing point, placeing point, gripper value before placeing -1 to be same as for picking and gripper value after placeing.
 */
trajectory calculate_place_positions(cv::Mat_<double> object_in_robot_coordinate_system, double object_size){
    //higher position
    cv::Mat_<double> pre_ungrasping_in_robot_coordinate_system=object_in_robot_coordinate_system.clone();
    pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0, 2))=pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0, 2))+0.02;
    //after_ungrasping position
    cv::Mat_<double> after_ungrasping_in_robot_coordinate_system=object_in_robot_coordinate_system.clone();
    double object_diameter=object_size;//possibly will be taken from hough circles and sendet to this function
    double old_distance= sqrt(pow(object_in_robot_coordinate_system.at<double>(cv::Point(0, 0)),2)+pow(object_in_robot_coordinate_system.at<double>(cv::Point(0, 1)),2)); //distance between gripper and origin of robot coordinate system
    double object_diameter_xyz=object_size;//possibly will be taken from hough circles and sendet to this function
    double object_diameter_z=std::sin(object_in_robot_coordinate_system.at<double>(cv::Point(0, 4)))*object_diameter_xyz;
    double object_diameter_xy=std::cos(object_in_robot_coordinate_system.at<double>(cv::Point(0, 4)))*object_diameter_xyz;
    double new_distance=old_distance-object_diameter_xy;
    double alpha=atan2(object_in_robot_coordinate_system.at<double>(cv::Point(0, 1)),object_in_robot_coordinate_system.at<double>(cv::Point(0, 0)));
    if(circles!=1){
        pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,3))=pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,3))+alpha;//alpha;
        object_in_robot_coordinate_system.at<double>(cv::Point(0,3))=object_in_robot_coordinate_system.at<double>(cv::Point(0,3))+alpha;//alpha;
        after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,3))=after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,3))+alpha;//alpha;
    }
    after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0, 0))=new_distance*cos(alpha);
    after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,1))=new_distance*sin(alpha);
    if(circles==1){
        after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))=after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))+high_of_the_object/3*2;
        pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))=pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))+high_of_the_object/3*2;
        object_in_robot_coordinate_system.at<double>(cv::Point(0,2))=object_in_robot_coordinate_system.at<double>(cv::Point(0,2))+high_of_the_object/3*2;
    }else{
        after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))=after_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))+high_of_the_object+0.005+object_diameter_z;//+object_diameter_z;
        pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))=pre_ungrasping_in_robot_coordinate_system.at<double>(cv::Point(0,2))+high_of_the_object+0.005;
        object_in_robot_coordinate_system.at<double>(cv::Point(0,2))=object_in_robot_coordinate_system.at<double>(cv::Point(0,2))+high_of_the_object+0.005;
    }
    double gripper_open=MAX_GRIPPER;
    trajectory place_trajectory;
    place_trajectory.preposition=pre_ungrasping_in_robot_coordinate_system;
    place_trajectory.position=object_in_robot_coordinate_system;
    place_trajectory.afterposition=after_ungrasping_in_robot_coordinate_system;
    place_trajectory.gripper_pre=-1;
    place_trajectory.gripper_after=gripper_open;
    return place_trajectory;
}

/**
 * @brief visionCallback actualize the vision position vector dependent on actually recognized object
 * @param msg hough_circle_position
 */
void visionCallback(const vision_part::hough_circle_position& msg)
{
    vision_position_vector=(cv::Mat_<double>(3,1) << msg.x, msg.y, msg.z);
    return;
}


/**
 * @brief calculate_diameter_from_pixels Recalculate diameter in pixels to the diameter in meters.
 * @param d_in_pxl diameter in pixel
 * @param distance_object z in meters
 * @return  diameter in meters.
 */
double calculate_diameter_from_pixels(double d_in_pxl, double distance_object){
    double focal_length=FOCAL_LENGTH;
    double diameter=(d_in_pxl)*(distance_object*1000/focal_length-1);
    return diameter/1000;
}

/**
 * @brief load_calibration_matrix Load calibration matrix from file data/robot_camera_calibration/calibration_transformation.ext
 * @return calibration matrix
 */
cv::Mat load_calibration_matrix(){
    cv::Mat calibration_matrix;
    try
    {
        cv::FileStorage storage("data/robot_camera_calibration/calibration_transformation.ext", cv::FileStorage::READ);
        storage["transformation"] >> calibration_matrix;
        storage.release();
        ROS_INFO_STREAM("Calibration matrix was loaded: \n"<<calibration_matrix);
    }catch (int err_code){
        ROS_ERROR("Error during loading the calibration matrix.");
        return calibration_matrix;
    }
    if(calibration_matrix.empty()){
        ROS_ERROR("Error during loading the calibration matrix, no file calibration_transformation.ext exist. Please check and generate it using the file calibration.cpp in package vision part. Or nothing stored in the file calibration_transformation.ext under name transformation");
        return calibration_matrix;
    }
    return calibration_matrix;
}

/**
 * @brief calibrate_to_robot_coordinate_system Recalculate from the vector in the camera coordinate system (pxl, pxl, meters) to the vector in the robot coordinate system.
 * @param calibration_matrix transformation matrix between coordinate systems when both are in meters
 * @param v_position_tmp vector in camera coordinate system (pxl, pxl, meters)
 * @return vector in robot coordinate syestem
 */
cv::Mat calibrate_to_robot_coordinate_system(cv::Mat calibration_matrix, cv::Mat v_position_tmp){
    double z=(double)v_position_tmp.at<double>(0,2);//+0.08;//+0.04;
    double tmp_scale=TMP_SCALE;
    double tmp_x=X_MOVE;
    double tmp_y=Y_MOVE;
    cv::Mat D;
    if(circles==1){
        D =(cv::Mat_<double>(4,1)<< ((double)v_position_tmp.at<double>(0,0)-tmp_x)*(z)*tmp_scale,((double)v_position_tmp.at<double>(0,1)-tmp_y)*z*tmp_scale,z,(double)1); //change same as in the calibration
    }else{
        D =(cv::Mat_<double>(4,1)<< ((double)v_position_tmp.at<double>(0,0)-tmp_x)*(z)*tmp_scale,((double)v_position_tmp.at<double>(0,1)-tmp_y)*(z)*tmp_scale,z-0.015,(double)1); //change same as in the calibration
    }
    cv::Mat robot_position_vector= calibration_matrix*D;
    return robot_position_vector;

}

/**
 * @brief angle_using_cosin_law Calculate the angle by the point c.
 * @param a triangle top
 * @param b triangle top
 * @param c trinagle top on them will be the angle calculated
 * @return Angle by the point c.
 */
float angle_using_cosin_law(cv::Point_<float> a,cv::Point_<float> b,cv::Point_<float> c){
    float side_a=distance_points(b,c);
    float side_b=distance_points(a,c);
    float side_c=distance_points(b,a);
    float gamma= acos((pow(side_a,2)+pow(side_b,2)-pow(side_c,2))/(2*side_a*side_b));
    return gamma;
}

/**
 * @brief control_trajectory Control if all points in trajectory are valid. In robot joint range and without collision dangerous. If there will be a complication print a warning.
 * @param trajectory
 * @param nh node handle
 * @param pick True, if it is a picking trajectory.
 * @return Return true if all points are valid.
 */
bool control_trajectory(trajectory trajectory,ros::NodeHandle nh, bool pick){
    bool ok=true;
    if(trajectory.afterposition.empty()){
        return false;
    }
    std::string approach="is out of robot range, it can be caused by the angle under which the grasping position is required to be approach. ";
    std::string z_axes="is out of robot range, it can be caused by to high number in z axes in dependency on position and yaw angle. ";
    std::string one;
    std::string three;
    std::string type;
    if(pick){
        one = approach;
        three = z_axes;
        type="picking";
    }else{
        one = z_axes;
        three = approach;
        type="placing";
    }
    int check_preposition=control_position(trajectory.preposition,nh);
    int check_position=control_position(trajectory.position,nh);
    int check_afterposition=control_position(trajectory.afterposition,nh);
    if(check_preposition==-1){
        ROS_WARN_STREAM("Preparation position by "<< type<<" "<<one<<"With preposition value: \n"<<trajectory.preposition<<".");
        ok=false;
    }
    if(check_position==-1){
        ROS_WARN_STREAM("Position by "<< type<<" is out of robot range. With position value: \n"<<trajectory.position<<".");
        ok=false;
    }if(check_afterposition==-1){
        ROS_WARN_STREAM("Afterposition by "<< type<<" "<<three<<"With afterposition value: \n"<<trajectory.preposition<<".");
        ok=false;
    }
    if(check_preposition<0 || check_position<0 || check_afterposition<0){
        ok=false;
    }
    return ok;

}

/**
 * @brief calculate_grasping_angle Calculate the angle of the object in comparison to the pattern image for the rotation around the middle of the object.
 * @param area_of_recognized_object_object Information about the recognized object. in our case are used the coordinates of the corners and middle.
 * @param calibration_matrix Transformation matrix between camera and robot coordinate systems.
 * @return Angle between pattern and recognized object around z axes in the middle of the object.
 */
double calculate_grasping_angle(vision_part::area_of_recognized_object  area_of_recognized_object_object, cv::Mat calibration_matrix){
    //recalculate all points to robot coordinate system
    cv::Point2f left_top;
    cv::Point2f right_top;
    cv::Point2f right_down;
    cv::Point2f left_down;
    cv::Point2f center;
    cv::Mat tmp_position=(cv::Mat_<double>(3,1) << area_of_recognized_object_object.left_top_x[0], area_of_recognized_object_object.left_top_y[0], area_of_recognized_object_object.center_z[0]); //calculated for unique z
    cv::Mat robot_vector_tmp=calibrate_to_robot_coordinate_system(calibration_matrix, tmp_position);
    left_top.x=robot_vector_tmp.at<double>(0,0);
    left_top.y=robot_vector_tmp.at<double>(1,0);
    tmp_position=(cv::Mat_<double>(3,1) << area_of_recognized_object_object.right_top_x[0], area_of_recognized_object_object.right_top_y[0], area_of_recognized_object_object.center_z[0]); //calculated for unique z
    robot_vector_tmp=calibrate_to_robot_coordinate_system(calibration_matrix, tmp_position);
    right_top.x=robot_vector_tmp.at<double>(0,0);
    right_top.y=robot_vector_tmp.at<double>(1,0);
    tmp_position=(cv::Mat_<double>(3,1) << area_of_recognized_object_object.right_down_x[0], area_of_recognized_object_object.right_down_y[0], area_of_recognized_object_object.center_z[0]); //calculated for unique z
    robot_vector_tmp=calibrate_to_robot_coordinate_system(calibration_matrix, tmp_position);
    right_down.x=robot_vector_tmp.at<double>(0,0);
    right_down.y=robot_vector_tmp.at<double>(1,0);
    tmp_position=(cv::Mat_<double>(3,1) << area_of_recognized_object_object.left_down_x[0], area_of_recognized_object_object.left_down_y[0], area_of_recognized_object_object.center_z[0]); //calculated for unique z
    robot_vector_tmp=calibrate_to_robot_coordinate_system(calibration_matrix, tmp_position);
    left_down.x=robot_vector_tmp.at<double>(0,0);
    left_down.y=robot_vector_tmp.at<double>(1,0);
    tmp_position=(cv::Mat_<double>(3,1) << area_of_recognized_object_object.center_x[0], area_of_recognized_object_object.center_y[0], area_of_recognized_object_object.center_z[0]); //calculated for unique z
    robot_vector_tmp=calibrate_to_robot_coordinate_system(calibration_matrix, tmp_position);
    center.x=robot_vector_tmp.at<double>(0,0);
    center.y=robot_vector_tmp.at<double>(1,0);

    cv::Point2f length;
    double z=(double)area_of_recognized_object_object.center_z[0];//+0.08;//+0.04;
    double tmp_scale=TMP_SCALE;
    length.x=area_of_recognized_object_object.length_x[0]*(z)*tmp_scale;
    length.y=area_of_recognized_object_object.length_y[0]*(z)*tmp_scale;
    double rot_z=angle_using_cosin_law(left_top,-length/2 + center,center);
    cv::Point_<float> a=left_top;
    cv::Point_<float> c=center;
//    float angle_tmp=atan2(a.y-c.y,a.x-c.x)+3*M_PI/4;
//    if(angle_tmp<0||angle_tmp>M_PI){
//        rot_z=2*M_PI-rot_z;
//    }
    return -(rot_z-M_PI);//+M_PI/8;
}

/**
 * @brief pick_and_place_same Construct place trajectory from the pick trajectory as an inverse procedure. Used if the user do not choose place position. (pregrasping=aftergrasping, placeing=grasping, aftergrasping=preplacing)
 * @param pick_trajectory grasping trajectory
 * @return placing trajectory
 */
trajectory pick_and_place_same(trajectory pick_trajectory){
    trajectory place_trajectory;
    place_trajectory.preposition=pick_trajectory.afterposition;
    place_trajectory.position=pick_trajectory.position;
    place_trajectory.afterposition=pick_trajectory.preposition;
    place_trajectory.gripper_pre=pick_trajectory.gripper_after;
    place_trajectory.gripper_after=pick_trajectory.gripper_pre;
    return place_trajectory;
}

/**
 * @brief main Initialize node, wait for message for circles or for user defined objects, calculate rotations of the object, approaching angle for the grasping position, calculate the pick and place trajectories, perform the pick and place task.
 * example: rosrun robot_part pick_and_place ibuflam 715 960
 * @param argc two or four
 * @param argv first argument is the object tah will be grasped, second and third the x, y position in capturted image where will be the object placed. (NOTE: The picture for user defined object is displayed by the function recognize and show object with pattern picture, because of that it is needed to substract the from the x value the pattern x size before you write it as argument.)
 * @return
 */
int main(int argc, char **argv)
{
    bool ret;
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh;
    cv::Mat calibration_matrix=load_calibration_matrix();

    if(calibration_matrix.empty()){
        return -1; // roserror to that already in load_calibration_matrix()
    }

    std::string object_name;
    if(argc>1){
        object_name=argv[1];
    }else{
        ROS_ERROR("Not enought input arguments, please add name of object you want to pick and place. The object names can be found in /data/user_defined_objects/user_defined_objects.names, or choose circles for pick and place a can.");
        return -1;
    }
    if(object_name.compare("circles")==0){
        circles=1;
    }
    ros::ServiceClient name_of_object = nh.serviceClient<vision_part::object_to_be_recognised>("vision_part/object_to_be_recognised");
    vision_part::object_to_be_recognised srv_obj_name;
    srv_obj_name.request.name = object_name;
    if(name_of_object.call(srv_obj_name)){
        //std::cout<<"ok: "<<srv_obj_name.response.ok<<std::endl;
    }


    double diameter;
    double roll;
    double pitch;

    if(circles==1){
        //PICK AND PLACE OF CIRCULAR_OBJECTS
        ROS_INFO("Waiting for recognizing a circle (topic vision_part/hough_circle_position_with_diameter).");
        boost::shared_ptr<vision_part::hough_circle_position_with_diameter const> hough_circle_position_pointer;
        hough_circle_position_pointer = ros::topic::waitForMessage<vision_part::hough_circle_position_with_diameter>("vision_part/hough_circle_position_with_diameter", nh,ros::Duration(10,0));
        if(!hough_circle_position_pointer){
            ROS_ERROR("The hough_circle_position is not published for time longer then 10 seconds, is the camera properly connected and is the kinect_bridge launched and also display_picture_kinect,is a object in the camera field of view? Also check window RGB, if there is enough stable recognition of the object.");
            return -1;
        }else{
            vision_part::hough_circle_position_with_diameter  hough_circles_position_object= *hough_circle_position_pointer;
            vision_position_vector=(cv::Mat_<double>(3,1) << hough_circles_position_object.x, hough_circles_position_object.y, hough_circles_position_object.z);//+0.08);
            diameter=calculate_diameter_from_pixels(hough_circles_position_object.diameter,hough_circles_position_object.z);//-0.01
            high_of_the_object=hough_circles_position_object.height/100; //from cm to m
            ROS_INFO("Circle was detected on coordinates [%.0lf, %.0lf, %.3lf] with diameter %.2lf.", hough_circles_position_object.x,hough_circles_position_object.y,hough_circles_position_object.z, diameter);
        }
        roll=0;
        pitch=0;
    }else{
        //PICK AND PLACE OTHER THING THEN CIRCULAR
        ROS_INFO("Waiting for recognizing an object (topic vision_part/area_of_recognized_object).");
        boost::shared_ptr<vision_part::area_of_recognized_object const> area_of_recognized_object_pointer;
        area_of_recognized_object_pointer = ros::topic::waitForMessage<vision_part::area_of_recognized_object>("vision_part/area_of_recognized_object", nh,ros::Duration(10,0));
        if(!area_of_recognized_object_pointer){
            ROS_ERROR("The area_of_recognized_object is not published for time longer then 10 seconds, is the camera properly connected and is the kinect_bridge launched and also display_picture_kinect, is a object in the camera field of view? Also check window RGB, if there is enough stable recognition of the object.");
            return -1;
        }else{
            vision_part::area_of_recognized_object  area_of_recognized_object_object= *area_of_recognized_object_pointer;
            ROS_INFO("Object was recognized on position: [%.0lf, %.0lf, %.3lf] with diameters [%.2lf,%.2lf] and angle %.2lf.", area_of_recognized_object_object.center_x[0],area_of_recognized_object_object.center_y[0],area_of_recognized_object_object.center_z[0], area_of_recognized_object_object.length_x[0], area_of_recognized_object_object.length_y[0], area_of_recognized_object_object.rotation_z[0]);
            vision_position_vector=(cv::Mat_<double>(3,1) << area_of_recognized_object_object.center_x[0], area_of_recognized_object_object.center_y[0], area_of_recognized_object_object.center_z[0]);
            high_of_the_object=area_of_recognized_object_object.length_z[0]/100; //from cm to m
            double object_length;
            //posible to choose the shorter size, change also the corners, angle POKUD DRUHE DVA BODY STACI UHEL OTOCIT O 90 STUPNU
            //            if(area_of_recognized_object_object.length_x[0]>area_of_recognized_object_object.length_y[0]){
            object_length=area_of_recognized_object_object.length_y[0];
            //            }else{
            //                object_length=area_of_recognized_object_object.length_x[0];
            //            }
            diameter=calculate_diameter_from_pixels(object_length,area_of_recognized_object_object.center_z[0])-0.003;
            roll=calculate_grasping_angle(area_of_recognized_object_object, calibration_matrix);
            pitch = M_PI/2;
        }
    }

    cv::Mat robot_position_vector= calibrate_to_robot_coordinate_system(calibration_matrix,vision_position_vector);
    cv::Mat_<double> pick_object_position=(cv::Mat_<double>(6,1) <<robot_position_vector.at<double>(0),robot_position_vector.at<double>(1),robot_position_vector.at<double>(2),roll,pitch,0);

    //calculate prepare position and position after grasping/placeing
    trajectory pick_trajectory=calculate_pick_positions(pick_object_position,diameter);
    ROS_INFO("Object position in robot coordinate system: [%.3lf, %.3lf, %.3lf] and rool %.3lf and pitch angle %3lf.", pick_trajectory.position.at<double>(0),pick_trajectory.position.at<double>(1),pick_trajectory.position.at<double>(2),pick_trajectory.position.at<double>(3), pick_trajectory.position.at<double>(4));

    cv::Mat_<double> place_object_position;
    trajectory place_trajectory;

    if(argc==4){
        double x=atof(argv[2]);
        double y=atof(argv[3]);
        double z;
        ros::ServiceClient client_vision = nh.serviceClient<vision_part::get_depth>("vision_part/get_depth");
        vision_part::get_depth srv_vision;
        srv_vision.request.x = x;
        srv_vision.request.y = y;
        if (client_vision.call(srv_vision)){
            if (srv_vision.response.z>0.01){
                z=srv_vision.response.z;
                cv::Mat vision_position_vector_place=(cv::Mat_<double>(3,1) << x, y, z);//+0.08);//0.0005 5mm save distance from surface or other objects
                ROS_INFO("Place position in camera coordinate system: [%.0lf, %.0lf, %.3lf] with same angle as was recognized.", x,y,z);
                cv::Mat tmp_3_coordinates_place_object_position= calibrate_to_robot_coordinate_system(calibration_matrix,vision_position_vector_place);
                place_object_position=(cv::Mat_<double>(6,1) <<tmp_3_coordinates_place_object_position.at<double>(0),tmp_3_coordinates_place_object_position.at<double>(1),tmp_3_coordinates_place_object_position.at<double>(2),roll,pitch,0);//0.31,0.1,0.15,0,0,0);
            }else{
                ROS_WARN("Chosen position containes zero in depth map. It can mean that on choosen position is on the edge between two height or that it is out of depth map range.");
                place_trajectory=pick_and_place_same(pick_trajectory);
            }

        }else{
            ROS_ERROR("Node depth_on_RGB_coordinates from package vision_part does not run and it is essential for current task, please run this node by typing in command line rosrun vision_part depth_on_RGB_coordinates.");
            return -1;
        }
        place_trajectory=calculate_place_positions(place_object_position,diameter);
        place_trajectory.gripper_pre=pick_trajectory.gripper_after;

    }else{
        ROS_WARN("No position as argument chossed it will be used same position for pick and also for place.");
        place_trajectory=pick_and_place_same(pick_trajectory);
    }


    ROS_INFO("Place position in robot coordinate system: [%.3lf, %.3lf, %.3lf] and rool %.3lf and pitch angle %3lf.", place_trajectory.position.at<double>(0),place_trajectory.position.at<double>(1),place_trajectory.position.at<double>(2),place_trajectory.position.at<double>(3), place_trajectory.position.at<double>(4));

    //control position if they are not out of robot range
    bool in_range_pick_trajectory=control_trajectory(pick_trajectory,nh, true);
    bool in_range_place_trajectory=control_trajectory(place_trajectory,nh, false);

    // run the pick and place on robo
    if(in_range_pick_trajectory==true && in_range_place_trajectory==true){
        ret=go_on_position(pick_trajectory, nh);
        if(ret==false){
            return -1;
        }

        ret=go_on_position(place_trajectory, nh);
        if(ret==false){
            return -1;
        }
    }else{
        ROS_WARN("Pick and place task can not be performed, because the trajectory interferes out of robot range or there is a collision dangerous.");
    }
    return 0;
}
