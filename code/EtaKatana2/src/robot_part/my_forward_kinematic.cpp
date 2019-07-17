/** @file my_forward_kinematic.cpp
 *  @brief This node calculate forward kinematic for robotic arm Katana 400 6m180 from the /joint_states and publish the result in /robot_part/end_effector_position_with_orientation.
 *  @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "std_msgs/String.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "sensor_msgs/JointState.h"
#include <robot_part/end_effector_position_with_orientation.h>
#define PI 3.14159265



cv::Mat R_x(double theta);
cv::Mat R_z(double theta);
cv::Mat T_z(double t);
cv::Mat T_x(double t);
cv::Mat D_H_notation_step(double theta_z,double t_z, double t_x, double theta_x);
cv::Mat compute_forward_kinematic(double theta [] );
bool isRotationMatrix(cv::Mat &R);
cv::Mat rotationMatrixToEulerAngles(cv::Mat &R);
ros::Publisher pub_end_effector;

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
 * @brief D_H_notation_step One step for modified DH notation R_x, T_x, R_z, T_z
 * @param theta_z rotation around z axes
 * @param t_z move along z axes
 * @param t_x move along x axes
 * @param theta_x rotation around x axes
 * @param T currend transformation matrix
 * @return transformation multiplied by next D_H notatiion step
 */
cv::Mat D_H_notation_step(double theta_z,double t_z, double t_x, double theta_x, cv::Mat T){
    //FOR MODIFIED DH NOTATION
    T=T*R_x(theta_x);
    T=T*T_x(t_x);
    T=T*R_z(theta_z);
    T=T*T_z(t_z);
    return T;
}

/**
 * @brief compute_forward_kinematic Compute the forward kinematic of katana 400 180 robotic arm for vector of thetas, that are rotation along z axes
 * @param theta vector of rotations along z axes expected length 5
 * @return end effector position
 */
cv::Mat compute_forward_kinematic(double theta [] ){
    //using dh notation for katana
    cv::Mat T = (cv::Mat_<double>(4, 4) <<
                 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);

    std::cout << "theta 0: " << theta[0]*180/PI<<" theta 1: " << theta[1]*180/PI<<" theta 2: "<< theta[2]*180/PI<<" theta 3: "<< theta[3]*180/PI<<" theta 4: "<< theta[4]*180/PI<< std::endl;

    std::cout << "----------------------------------------------------------------------------------------------- " << std::endl;

    cv::Mat eye_vector=(cv::Mat_<double>(4,1) << 0, 0, 0, 1);

    T=D_H_notation_step(theta[0], 214.5,0,0,T); //(double theta_z,double t_z, double t_x, double theta_x, cv::Mat T originally 201.5         274.5
    T=D_H_notation_step(theta[1],0,0,PI/2, T);
    T=D_H_notation_step(theta[2],0,190,0,T);
    T=D_H_notation_step(theta[3],0,139,PI, T);
    T=D_H_notation_step(theta[4]-PI/2,0,245.0,PI/2, T);  //152.3 old //260
    cv::Mat position=T*eye_vector;
    return position;

}

/**
 * @brief isRotationMatrix Control if matrix is a rotation matrix true if it is.
 * source: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
 * @param R expected rotation matrix
 * @return frue if it is rotation matrix
 */
bool isRotationMatrix(cv::Mat &R)
//source:https://www.learnopencv.com/rotation-matrix-to-euler-angles/
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
/**
 * @brief rotationMatrixToEulerAngles Recalculate rotation matrix to euler angles.
 * @param R expected rotation matrix.
 * @return Vector of euler angles x,y,z.
 */
cv::Mat rotationMatrixToEulerAngles(cv::Mat &R)
{
    //source:https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Mat_<double>(4,1) << x,y,z,1;
}

/**
 * @brief robotStateCallback Calculate and publish the forvard_kinematic end effector position and orientation every time when are published the /joint_states.
 * @param msg /joint_states in our case published by node /katana.
 */
void robotStateCallback(sensor_msgs::JointState msg){
    double theta [] ={ msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4] };
    cv::Mat position=compute_forward_kinematic(theta);//D_H_notation_step( 1,PI/2,2,PI/3);//T_z(1)*R_z(PI/2)*T_x(2)*R_x(PI/3);
    robot_part::end_effector_position_with_orientation msg2;
    msg2.x= (position.at<double>(cv::Point(0, 0)))/1000;
    msg2.y= (position.at<double>(cv::Point(1, 0)))/1000;
    msg2.z= (position.at<double>(cv::Point(2, 0)))/1000;
    msg2.roll=-msg.position[4];
    msg2.pitch=-(msg.position[1]+msg.position[2]-msg.position[3]);
    msg2.yaw=msg.position[0];
    ROS_INFO("position of endeffector: [%.2lf, %.2lf, %.2lf] with orientation:[%.2lf, %.2lf, %.2lf]", msg2.x, msg2.y, msg2.z, msg2.roll,msg2.pitch, msg2.yaw);
    pub_end_effector.publish(msg2);
}

/**
 * @brief main Initialize the node and publisher of /robot_part/end_effector_position_with_orientation and subcriber /joint_states.
 * @param argc none
 * @param argv none
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_forward_kinematic");
    ros::NodeHandle nh;
    pub_end_effector= nh.advertise<robot_part::end_effector_position_with_orientation>("/robot_part/end_effector_position_with_orientation", 1000);
    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, robotStateCallback);
    ros::spin();
    return 0;

}

