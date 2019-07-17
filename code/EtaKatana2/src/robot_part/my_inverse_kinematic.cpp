/** @file my_inverse_kinematic.cpp
 *  @brief This node calculate inverse kinematic for robotic arm Katana 400 6m180, that is provided as a service robot_part/service_end_effector_position_with_orientation.
 * @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <robot_part/end_effector_position_with_orientation.h>
#include <robot_part/service_end_effector_position_with_orientationRequest.h>
#include <robot_part/service_end_effector_position_with_orientationResponse.h>
#include <robot_part/service_end_effector_position_with_orientation.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "std_msgs/String.h"
#define PI 3.14159265
cv::Mat R_x(double theta);
cv::Mat R_z(double theta);
cv::Mat T_z(double t);
cv::Mat T_x(double t);
cv::Mat D_H_notation_step(double theta_z,double t_z, double t_x, double theta_x);
cv::Mat compute_forward_kinematic(double theta [] );
///lower limits of katana robotic arm joints
double m_lower [5]={-3.025528,-0.135228,-2.221804,-2.033309,-2.993240};
///upper limits of katana robotic arm joints
double m_upper [5]={2.891097,2.168572,2.054223,1.876133,2.870985};

/**
 * @brief R_x prepare rotation matrix around x axes about angle theta
 * @param theta angle in radius
 * @return rotation matrix
 */
cv::Mat R_x(double theta){
    cv::Mat R_x = (cv::Mat_<double>(4,4) <<
                   1,       0,              0,             0,
                   0,       std::cos(theta),   -std::sin(theta),     0,
                   0,       std::sin(theta),   std::cos(theta),      0,
                   0,0,0,1
                   );
    return R_x;
}

/**
 * @brief R_z prepare rotation matrix around z axes about angle theta
 * @param theta angle in radius
 * @return rotation matrix
 */
cv::Mat R_z(double theta){
    cv::Mat R_z = (cv::Mat_<double>(4,4) <<
                   std::cos(theta),    -std::sin(theta),      0,       0,
                   std::sin(theta),    std::cos(theta),       0,       0,
                   0,               0,              1,       0,
                   0,               0,              0,       1);
    return R_z;
}

/**
 * @brief T_z prepare translation matrix along z axes on lenght t
 * @param t length of translation
 * @return translation matrix
 */
cv::Mat T_z(double t){
    cv::Mat T_z = (cv::Mat_<double>(4, 4) <<
                   1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, t,
                   0, 0, 0, 1);
    return T_z;
}

/**
 * @brief T_x prepare translation matrix along x axes on lenght t
 * @param t length of translation
 * @return translation matrix
 */
cv::Mat T_x(double t){
    cv::Mat T_x = (cv::Mat_<double>(4, 4) <<
                   1, 0, 0, t,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1);
    return T_x;
}

/**
 * @brief theta angle calculated as atan2(c2-b2,c1-b1)-atan2(b2-a2,b1-a1)
 * @param a1 X coordinate of first point
 * @param a2 Y coordinate of first point
 * @param b1 X coordinate of the point, where I am looking forward the point
 * @param b2 Y coordinate of the point, where I am looking forward the point
 * @param c1 X coordinate third point
 * @param c2 Y coordinate third point
 * @return angle theta
 */
double theta(double a1,double a2,double b1,double b2,double c1,double c2){
    double Xo=atan2(c2-b2,c1-b1)-atan2(b2-a2,b1-a1);
    return Xo;
}

/**
 * @brief wrapAngle Recalculate angle to be between 0 and 2*pi.
 * @param angle Angle in radians.
 * @return Angle in radians with value in range 0-2*pi.
 */
inline double wrapAngle( double angle )
{
    //source:https://stackoverflow.com/questions/11980292/how-to-wrap-around-a-range
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}

/**
 * @brief angle_between_pi_and_minus_pi Recalculate angle to the range [-pi, pi]
 * @param arg Angle in radians.
 * @return Angle in range [-pi, pi]
 */
double angle_between_pi_and_minus_pi(double arg){
    double tmp=arg;
    while(tmp>M_PI){
        tmp=tmp-2*M_PI;
    }
    while(tmp<-M_PI){
        tmp=tmp+2*M_PI;
    }
    return tmp;
}

/**
 * @brief calculate_planar_manipulator Calculate the rotations for the second manipulator, third and fourth joint of katana robotic arm.
 * @param x X coordinate of the end effector position.
 * @param z Z coordinate of the end effector position.
 * @param pitch pitch angle of the end effector.
 * @return Return up to two solutions for the rotations of the j2-j3 joints (katana_motor2_lift_joint, katana_motor3_lift_joint and katana_motor4_lift_joint)
 */
cv::Mat_<double>calculate_planar_manipulator(double x, double z, double pitch){
    cv::Mat planar_joints_rotation;
    double gripper_theta=pitch; //till now only the horizontal position
    double x4=x;
    double z4=z;
    double l1= 214.5/1000;// old 274.5 //201.5
    double l2=190.0/1000;
    double l3=139.0/1000;
    double l4=245.0/1000;//152.3/1000;//  FOR MIDDLE OF THE GRIPPER 260
    double z1=l1;
    double x1=0;
    double x3=-std::cos(gripper_theta)*l4+x4;
    double z3=std::sin(gripper_theta)*l4+z4;
    //calculate the intersection
    double distance = sqrt(pow((x1-x3),2)+(pow((z1-z3),2)));
    double max_distance= l2+l3;
    if (max_distance>=distance){
        double m = (((pow(l2,2))-(pow(l3,2)))/(2*distance))+(distance/2);
        double V = sqrt((pow(l2,2))-(pow(m,2)));
        double stredx = x1+(m/distance)*(x3-x1);
        double stredy = z1+(m/distance)*(z3-z1);
        double Sx1 = stredx-(V/distance)*(z1-z3);
        double Sz1 = stredy+(V/distance)*(x1-x3);
        double Sx2 = stredx+(V/distance)*(z1-z3);
        double Sz2 = stredy-(V/distance)*(x1-x3);
        double omega2a;
        double omega1a;
        double omega2b;
        double omega1b;
        if(Sz1>Sz2){
            omega2a = angle_between_pi_and_minus_pi(theta(x3,z3,Sx1,Sz1,x1,z1));
            omega1a =angle_between_pi_and_minus_pi((theta(Sx1,Sz1,x1,z1,0,0)-PI/2));
            omega2b = angle_between_pi_and_minus_pi(theta(x3,z3,Sx2,Sz2,x1,z1));
            omega1b =angle_between_pi_and_minus_pi((theta(Sx2,Sz2,x1,z1,0,0)-PI/2));
        }else{
            omega2a = angle_between_pi_and_minus_pi(theta(x3,z3,Sx2,Sz2,x1,z1));
            omega1a =angle_between_pi_and_minus_pi((theta(Sx2,Sz2,x1,z1,0,0)-PI/2));
            omega2b = angle_between_pi_and_minus_pi(theta(x3,z3,Sx1,Sz1,x1,z1));
            omega1b =angle_between_pi_and_minus_pi((theta(Sx1,Sz1,x1,z1,0,0)-PI/2));
        }
        double o1a=angle_between_pi_and_minus_pi(wrapAngle(omega1a*(-1)));
        double o2a=angle_between_pi_and_minus_pi(-wrapAngle(omega2a));
        double o3a=angle_between_pi_and_minus_pi(gripper_theta+o1a+o2a);
        double o1b=angle_between_pi_and_minus_pi(wrapAngle(omega1b*(-1)));
        double o2b=angle_between_pi_and_minus_pi(-wrapAngle(omega2b));
        double o3b=angle_between_pi_and_minus_pi(gripper_theta+o1b+o2b);
        planar_joints_rotation=(cv::Mat_<double>(2,3) << o1a, o2a, o3a,
                                o1b, o2b, o3b);
    }else{
        ROS_WARN("Out of robot range.");
    }
    return planar_joints_rotation;
}

/**
 * @brief control_field Check all the joints rotations if they are in the the joints ranges.
 * @param rotations Matrix with all the robot rotations expected for katana cv::Mat_<double>(5,1).
 * @return Return false if some of the joint is out of the joint limit, otherwise it returns true.
 */
bool control_field(cv::Mat rotations){
    bool tmp=true;
    for(int i =0; i<sizeof(m_upper)/sizeof(*m_upper); i++){
        if(rotations.at<double>(cv::Point(0, i))>m_upper[i] ||rotations.at<double>(cv::Point(0, i))<m_lower[i]){
            tmp= false;
            ROS_WARN("Rotation of %i -th joint is out of range, lower joint limit: %lf, joint calculated value: %lf, upper joint limit: %lf", i+1, m_lower[i],rotations.at<double>(cv::Point(0, i)),m_upper[i]);
        }
    }
    return tmp;
}

/**
 * @brief control_limits Choose solution if is one of them in the robot joints range. Choose as first the solution with higher value of third joint in the z axes, as the priority solution, that is more far from the surface.
 * @param robot_joints_rotations_1 first possible solution
 * @param robot_joints_rotations_2 second posible solution
 * @return Choosed solution, if non of them is in joints limits retun empty matrix.
 */
cv::Mat control_limits(cv::Mat robot_joints_rotations_1,cv::Mat robot_joints_rotations_2){
    //control all limits by robot 1
    if(control_field(robot_joints_rotations_1)==true){
        ROS_INFO("No angle out of limits for first solution.");
        return robot_joints_rotations_1;
    }else if(control_field(robot_joints_rotations_2)==true){
        return robot_joints_rotations_2;
        ROS_INFO("No angle out of limits for second solution.");
    }else{
        cv::Mat blank_matrix;
        return blank_matrix;
    }
}

/**
 * @brief inverseKinematic Service that calculates the inverse kinematic for the robot Katana 400 6m180. Service name robot_part/service_end_effector_position_with_orientation. BE CAREFUL: The roll angle is calculated for symmetrical two fingers gripper, so the roll angle range is narrowed everytime in gripper range.
 * @param req Request are the 3D coordinates and rotations of the end effector.
 * @param res Responce are rotations of the joints.
 * @return true for valid solutions, false otherwise.
 */
bool inverseKinematic(robot_part::service_end_effector_position_with_orientation::Request &req,robot_part::service_end_effector_position_with_orientation::Response &res)
{
    double transformed_x= sqrt(pow(req.x, 2)+pow(req.y, 2));
    double motor1_pan_joint=angle_between_pi_and_minus_pi(atan2(req.y, req.x));
    double motor5_wrist_joint=angle_between_pi_and_minus_pi(-req.roll);
    if(motor5_wrist_joint>m_upper[4]){
        motor5_wrist_joint=motor5_wrist_joint-M_PI;
    }else if(motor5_wrist_joint<m_lower[4]){
        motor5_wrist_joint=motor5_wrist_joint+M_PI;
    }
    cv::Mat planar_robot_joints=calculate_planar_manipulator(transformed_x, req.z, req.pitch);
    if(!planar_robot_joints.empty()){
        cv::Mat robot_joints_rotations_1;
        cv::Mat robot_joints_rotations_2;
        robot_joints_rotations_1=(cv::Mat_<double>(5,1) << motor1_pan_joint, planar_robot_joints.at<double>(cv::Point(0, 0)), planar_robot_joints.at<double>(cv::Point(1, 0)),planar_robot_joints.at<double>(cv::Point(2, 0)), motor5_wrist_joint);
        robot_joints_rotations_2=(cv::Mat_<double>(5,1) << motor1_pan_joint, planar_robot_joints.at<double>(cv::Point(3, 0)), planar_robot_joints.at<double>(cv::Point(4, 0)),planar_robot_joints.at<double>(cv::Point(5, 0)), motor5_wrist_joint);
        bool control1=control_field(robot_joints_rotations_1)==true;
        bool control2=control_field(robot_joints_rotations_2)==true;
        if(control1){
            res.katana_motor1_pan_joint.push_back(robot_joints_rotations_1.at<double>(cv::Point(0, 0)));
            res.katana_motor2_lift_joint.push_back(robot_joints_rotations_1.at<double>(cv::Point(1, 0)));
            res.katana_motor3_lift_joint.push_back(robot_joints_rotations_1.at<double>(cv::Point(2,0)));
            res.katana_motor4_lift_joint.push_back(robot_joints_rotations_1.at<double>(cv::Point(3, 0)));
            res.katana_motor5_wrist_roll_joint.push_back(robot_joints_rotations_1.at<double>(cv::Point(4, 0)));
            ROS_INFO("Solution [%lf, %lf, %lf, %lf, %lf] is in joints limits.", robot_joints_rotations_1.at<double>(cv::Point(0, 0)),robot_joints_rotations_1.at<double>(cv::Point(1, 0)),robot_joints_rotations_1.at<double>(cv::Point(2, 0)),robot_joints_rotations_1.at<double>(cv::Point(3, 0)),robot_joints_rotations_1.at<double>(cv::Point(4, 0)));
        }
        if(control2){
            res.katana_motor1_pan_joint.push_back(robot_joints_rotations_2.at<double>(cv::Point(0, 0)));
            res.katana_motor2_lift_joint.push_back(robot_joints_rotations_2.at<double>(cv::Point(1, 0)));
            res.katana_motor3_lift_joint.push_back(robot_joints_rotations_2.at<double>(cv::Point(2,0)));
            res.katana_motor4_lift_joint.push_back(robot_joints_rotations_2.at<double>(cv::Point(3, 0)));
            res.katana_motor5_wrist_roll_joint.push_back(robot_joints_rotations_2.at<double>(cv::Point(4, 0)));
            ROS_INFO("Solution [%lf, %lf, %lf, %lf, %lf] is in joints limits.", robot_joints_rotations_2.at<double>(cv::Point(0, 0)),robot_joints_rotations_2.at<double>(cv::Point(1, 0)),robot_joints_rotations_2.at<double>(cv::Point(2, 0)),robot_joints_rotations_2.at<double>(cv::Point(3, 0)),robot_joints_rotations_2.at<double>(cv::Point(4, 0)));
        }
        if(!(control1 || control2)){
            ROS_WARN("No solution of the inverse kinematic in robot joints limit.");
            return false;
        }
        return true;
    }
}

/**
 * @brief main intialize the node and the service robot_part/service_end_effector_position_with_orientation
 * @param argc 1
 * @param argv none
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_inverse_kinematic");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("robot_part/service_end_effector_position_with_orientation", inverseKinematic);
    ros::spin();
    return 0;
}
