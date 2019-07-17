/** @file go_home.cpp
 *  @brief This node is an service, that run once after calling from comand line and send the robot to the home position [0.05,0.05,0.62] with roll and pitch rotations [2.93,-1.5];.
 *  @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "control_msgs/GripperCommandActionGoal.h"
#include "katana_msgs/JointMovementActionGoal.h"
#include "katana_msgs/JointMovementActionResult.h"
#include <vision_part/hough_circle_position.h>
#include <vision_part/hough_circle_position_with_diameter.h>
#include "control_msgs/GripperCommandActionResult.h"
#include "control_msgs/JointTrajectoryActionGoal.h"
#include "control_msgs/JointTrajectoryActionResult.h"
#include <robot_part/service_end_effector_position_with_orientation.h>


/**
 * @brief change_of_position send the robot to the position object_in_robot_coordinate_system inrespective of collisions with environment
 * @param object_in_robot_coordinate_system position with orientation, where the robot will go cv::Mat_<double>(6,1)
 * @param nh note handle
 */
void change_of_position(cv::Mat_<double> object_in_robot_coordinate_system, ros::NodeHandle nh){
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
    }else
    {
        ROS_ERROR("Failed to call service my_inverse_kinematic");
        return;
    }
    return;
}
/**
 * @brief change_of_gripper_setting set gripper on value gripper_position
 * @param gripper_position
 * @param nh note handler
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
            //print_status_info(status);
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
 * @brief main initializes the node and define the home position
 * @param argc none
 * @param argv none
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_home");
    ros::NodeHandle nh;
    change_of_gripper_setting(0.3, nh);
    cv::Mat home_position=(cv::Mat_<double>(6,1) <<0.05,0.05,0.62,2.93,-1.5,0);
    change_of_position(home_position, nh);
    ROS_INFO("Robot is going home");
}
