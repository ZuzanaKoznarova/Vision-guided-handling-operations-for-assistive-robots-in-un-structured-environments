/** @file calibration.cpp
 *  @brief This node calculate the transformation matrix between robot and camera coordinate systems using the recognition of calibration ball. If there is any second argument, the testing of the transformating will run instead of calculationg of the transformation matrix.
 * The user will be asked to confirm movement to the next position by the enter, because we are expecting an uncalibrated system where is no calibration matrix applied. Also the recognized circle need to be confirmed, because there is also a risk of misdetection.
 * @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <robot_part/end_effector_position_with_orientation.h>
#include <robot_part/service_end_effector_position_with_orientation.h>
#include <opencv2/highgui/highgui.hpp>
#include <vision_part/hough_circle_position_with_diameter.h>
#include "control_msgs/GripperCommandActionGoal.h"
#include "katana_msgs/JointMovementActionGoal.h"
#include <math.h>
#include <ctime>
#include "include/vision_part/camera_parameters.h"

#define _USE_MATH_DEFINES

//define number of points between each two neighboring points
#define POINTS_IN_GRID 4


#define CALIBRATION_BALL_RADIUS 0.035

cv::Mat robot_position_vector;
cv::Mat vision_position_vector;
ros::Publisher chatter_pub_joint;
cv::Mat R_z(double theta);
cv::Mat R_x(double theta);
cv::Vec3d CalculateMean(const cv::Mat_<cv::Vec3d> &points);
cv::Mat_<double> FindRigidTransform(const cv::Mat_<cv::Vec3d> &points1, const cv::Mat_<cv::Vec3d> points2);
void publish_robot_joint_position(float theta1, float theta2,float theta3,float theta4,float theta5);
std::string name_of_the_calibration_file(std::string name);

/**
 * @brief The vision_and_robot_position struct with the robot and also vision matrix
 */
struct vision_and_robot_position
{
    cv::Mat_<cv::Vec3d> robot_position_matrix;
    cv::Mat_<cv::Vec3d> vision_position_matrix;

};

/**
 * @brief The deviations struct that includes deviations in x,y,z direction and then distance between robot real and robot calculated position and also the average deviation.
 */
struct deviations
{
    std::vector<double> deviation_x;
    std::vector<double> deviation_y;
    std::vector<double> deviation_z;
    std::vector<double> deviation_distance;
    double average_deviation;
};

/**
 * @brief robotCallback Subscribe the end_effector_position_with_orientation that is the actual robot position in 3D space calculated by my_forward_kinematic and save it to the global variable robot_position_vector.
 * @param msg coordinates of robot end effector
 */
void robotCallback(const robot_part::end_effector_position_with_orientation& msg)
{
    robot_position_vector=(cv::Mat_<double>(3,1) << msg.x, msg.y, msg.z);
    ROS_INFO("Subcribing the robot endeffector position");
    return;
}
/**
 * @brief visionCallback Subcribe hough_circle_position_with_diameter and take from that the position of the recognized circle and save it in vision_position_vector.
 * @param msg hough_circle_position_with_diameter, middle of the recognized circle in 3D
 */
void visionCallback(const vision_part::hough_circle_position_with_diameter& msg)
{
    vision_position_vector=(cv::Mat_<double>(3,1) << msg.x, msg.y, msg.z);
    //std::cout << vision_position_vector << std::endl;
    ROS_INFO("Subcribing the last founded hough circle");
    return;
}

//source: VAGRAN. Opencv: Rigid Transformation between two 3D point clouds [online]. In: . 2018 [cit. 2019-04-03]. Available from: https://stackoverflow.com/questions/21206870/opencv-rigid-transformation-between-two-3d-point-clouds
/**
 * @brief CalculateMean source: VAGRAN. Opencv: Rigid Transformation between two 3D point clouds [online]. In: . 2018 [cit. 2019-04-03]. Available from: https://stackoverflow.com/questions/21206870/opencv-rigid-transformation-between-two-3d-point-clouds
 * @param points pointcload
 * @return mean of the pointcload in 3D
 */
cv::Vec3d CalculateMean(const cv::Mat_<cv::Vec3d> &points)
{
    cv::Mat_<cv::Vec3d> result;
    cv::reduce(points, result, 0, CV_REDUCE_AVG);
    return result(0, 0);

}

/**
 * @brief FindRigidTransform Calculate the transformation between two pointcloads (from points1 to points2) where are defined the pairs in first and second pointcload.
 * source: VAGRAN. Opencv: Rigid Transformation between two 3D point clouds [online]. In: . 2018 [cit. 2019-04-03]. Available from: https://stackoverflow.com/questions/21206870/opencv-rigid-transformation-between-two-3d-point-clouds
 * @param points1 Pointcload, from where I want to find the transformation.
 * @param points2 Pointcload, to where I want to dind the transformation.
 * @return transformation matrix
 */
cv::Mat_<double> FindRigidTransform(cv::Mat_<cv::Vec3d> &points1, cv::Mat_<cv::Vec3d> points2)
{
    /* Calculate centroids. */
    cv::Vec3d t1 = -CalculateMean(points1);
    cv::Vec3d t2 = -CalculateMean(points2);
    cv::Mat_<double> T1 = cv::Mat_<double>::eye(4, 4);
    T1(0, 3) = t1[0];
    T1(1, 3) = t1[1];
    T1(2, 3) = t1[2];
    cv::Mat_<double> T2 = cv::Mat_<double>::eye(4, 4);
    T2(0, 3) = -t2[0];
    T2(1, 3) = -t2[1];
    T2(2, 3) = -t2[2];
    /* Calculate covariance matrix for input points. Also calculate RMS deviation from centroid
     * which is used for scale calculation.
     */
    cv::Mat_<double> C(3, 3, 0.0);
    double p1Rms = 0, p2Rms = 0;
    cv::Mat_<double> distance_p1_p2= cv::Mat_<double>::eye(1, 3);
    for (int ptIdx = 0; ptIdx < points1.rows; ptIdx++) {
        cv::Vec3d p1 = points1(ptIdx, 0) + t1;
        cv::Vec3d p2 = points2(ptIdx, 0) + t2;
        p1Rms += p1.dot(p1);
        p2Rms += p2.dot(p2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C(i, j) += p2[i] * p1[j];
            }
        }
    }
    cv::Mat_<double> u, s, vh;
    cv::SVD::compute(C, s, u, vh); //singular value decompozition
    cv::Mat_<double> R = u * vh;
    if (cv::determinant(R) < 0) {
        R -= u.col(2) * (vh.row(2) * 2.0);
    }
    double scale = sqrt(p2Rms / p1Rms);
    R *= scale;
    cv::Mat_<double> M = cv::Mat_<double>::eye(4, 4);
    R.copyTo(M.colRange(0, 3).rowRange(0, 3));
    cv::Mat_<double> result = T2 * M * T1;
    result /= result(3, 3);
    return result.rowRange(0, 3);
}
//end of source: VAGRAN. Opencv: Rigid Transformation between two 3D point clouds [online]. In: . 2018 [cit. 2019-04-03]. Available from: https://stackoverflow.com/questions/21206870/opencv-rigid-transformation-between-two-3d-point-clouds

/**
 * @brief measure_vision_positions This function sent the robot to the positions defined by end_effector_position_matrix then recognize the circle and send both informations back to the user.
 * The user control the movings and saving of the recogniyed positions by pressing enter for moving to the next position and by presing enter bz saving the recognized position. It is because of delays in depth recognition. And also because by the calibration we can not rely on vision, because of that there is no collision avoidance.
 * @param nh node handle
 * @param end_effector_position_matrix Matrix of the robot calibration positions.
 * @param number_of_points Number of points in calibration matrix.
 * @param robot_position_matrix Empty matrix for robot positions.
 * @param vision_position_matrix Empty matrix for recognized positions.
 * @return return stucture with vision and also robot positions matrixes.
 */
vision_and_robot_position measure_vision_positions(ros::NodeHandle nh,cv::Mat_<double> end_effector_position_matrix, int number_of_points,cv::Mat_<cv::Vec3d> robot_position_matrix,cv::Mat_<cv::Vec3d> vision_position_matrix){
    boost::shared_ptr<robot_part::end_effector_position_with_orientation const>end_effector_position_pointer;
    boost::shared_ptr<vision_part::hough_circle_position_with_diameter const> hough_circle_position_pointer;
    for(int j=0; j<number_of_points-1;j++){
        for(int m=0;m<POINTS_IN_GRID;m++){
            ros::ServiceClient client = nh.serviceClient<robot_part::service_end_effector_position_with_orientation>("robot_part/service_end_effector_position_with_orientation");
            robot_part::service_end_effector_position_with_orientation srv;
            srv.request.x = end_effector_position_matrix.at<double>(cv::Point(0,j))+(double)m/(double)POINTS_IN_GRID*(end_effector_position_matrix.at<double>(cv::Point(0,j+1))-end_effector_position_matrix.at<double>(cv::Point(0,j)));
            srv.request.y = end_effector_position_matrix.at<double>(cv::Point(1,j))+(double)m/(double)POINTS_IN_GRID*(end_effector_position_matrix.at<double>(cv::Point(1,j+1))-end_effector_position_matrix.at<double>(cv::Point(1,j)));
            srv.request.z = end_effector_position_matrix.at<double>(cv::Point(2,j))+(double)m/(double)POINTS_IN_GRID*(end_effector_position_matrix.at<double>(cv::Point(2,j+1))-end_effector_position_matrix.at<double>(cv::Point(2,j)));
            srv.request.roll =  end_effector_position_matrix.at<double>(cv::Point(3,j));
            srv.request.pitch = end_effector_position_matrix.at<double>(cv::Point(4,j));
            srv.request.yaw = end_effector_position_matrix.at<double>(cv::Point(5,j));

            if (client.call(srv)){
                if(!(srv.response.katana_motor1_pan_joint[0]==0 && srv.response.katana_motor2_lift_joint[0] ==0 )){
                    std::cout<<"For moving to next position press Enter."<<std::endl;
                    std::cin.ignore();
                    publish_robot_joint_position(srv.response.katana_motor1_pan_joint[0],srv.response.katana_motor2_lift_joint[0],srv.response.katana_motor3_lift_joint[0],srv.response.katana_motor4_lift_joint[0],srv.response.katana_motor5_wrist_roll_joint[0]); //DENGEROUS MORE SOLUTIONS
                    std::cout<<"The robot moved to the position: ["<<    srv.request.x<<", "<<    srv.request.y<<", "<<    srv.request.z<<", "<<    srv.request.roll<<", "<<   srv.request.pitch<<", "<<   srv.request.yaw<<"]"<<std::endl;
                }
                std::cout<<"If the movement is finished press Enter and you want to choose actual position press Enter."<<std::endl;
                std::cin.ignore();
                end_effector_position_pointer = ros::topic::waitForMessage<robot_part::end_effector_position_with_orientation>("robot_part/end_effector_position_with_orientation", nh, ros::Duration(10,0));
                if(!end_effector_position_pointer){
                    ROS_WARN("The end_effector_possition is not pubblished for time longer then 10 seconds, is the robot properly connected and is the katana launch launched and also the my_forward_kinematik?");
                    j--;
                }else{
                    robot_part::end_effector_position_with_orientation  end_effector_position_object= *end_effector_position_pointer;
                    robot_position_vector=(cv::Mat_<double>(3,1) << end_effector_position_object.x, end_effector_position_object.y, end_effector_position_object.z);

                    hough_circle_position_pointer = ros::topic::waitForMessage<vision_part::hough_circle_position_with_diameter>("vision_part/hough_circle_position_with_diameter", nh, ros::Duration(10,0));
                    if(!hough_circle_position_pointer){
                        ROS_WARN("The hough_circle_position is not published for time longer then 10 seconds, is the camera properly connected and is the kinect_bridge launched and also display_picture_kinect, is the calibration ball in the camera field of view?");
                        j--;
                    }else{
                        vision_part::hough_circle_position_with_diameter  hough_circles_position_object= *hough_circle_position_pointer;
                        double z=hough_circles_position_object.z+CALIBRATION_BALL_RADIUS; //added radius of the calibration ball
                        if(z>0.01){
                            double tmp=TMP_SCALE;
                           vision_position_vector=(cv::Mat_<double>(3,1) <<(hough_circles_position_object.x-X_MOVE)*z*tmp, (hough_circles_position_object.y-Y_MOVE)*z*tmp,z );//(hough_circles_position_object.x)*z*0.00173667, (hough_circles_position_object.y)*z*0.00173667,z ); //added scale old 0.0027
                            robot_position_matrix(j*POINTS_IN_GRID+m,0)=robot_position_vector;
                            vision_position_matrix(j*POINTS_IN_GRID+m,0)=vision_position_vector;
                        }else{
                            j--;
                        }
                    }

                }
                std::cout<<"Your choosed position is: "<<  vision_position_matrix(j*POINTS_IN_GRID+m,0)<<std::endl;
            }else{
            ROS_ERROR("Failed to call service my_inverse_kinematic, check if the service my_inverse_kinematic from package robot_part is running and if there exist a solution for the end effector possition.");
            vision_and_robot_position tmp;
            return tmp;
            }
        }
    }
    vision_and_robot_position result;
    result.robot_position_matrix = robot_position_matrix;
    result.vision_position_matrix=vision_position_matrix;
    return result;
}

/**
 * @brief calculate_the_deviations Calculates the error between real and recognized position of the calibration ball.
 * @param number_of_points
 * @param transformation transformation matrix from camera to robot coordinate system.
 * @param robot_position_matrix
 * @param vision_position_matrix
 * @return return structure of the deviations in x,y,z direction the distance between the pairs of points and average distance for all points
 */
deviations calculate_the_deviations(int number_of_points,cv::Mat_<double> transformation,cv::Mat_<cv::Vec3d> robot_position_matrix,cv::Mat_<cv::Vec3d> vision_position_matrix){
    cv::Mat D;
    double average_deviation=0.0;
    cv::Mat test;
    deviations dev;
    for(int k=0;k<(number_of_points-1)*POINTS_IN_GRID;k++){
        std::cout<< "vision vector:["<<(double)vision_position_matrix.at<double>(k,0)<<", "<<(double)vision_position_matrix.at<double>(k,1)<<", "<<(double)vision_position_matrix.at<double>(k,2)<<"]"<<std::endl;
        D=(cv::Mat_<double>(4,1)<< (double)vision_position_matrix.at<double>(k,0),(double)vision_position_matrix.at<double>(k,1),(double)vision_position_matrix.at<double>(k,2),(double)1);
        std::cout <<"robot original position: " <<k<<": "<< transformation*D<<std::endl;
        std::cout <<"robot calculated position: " <<k<<": ["<<(double)robot_position_matrix.at<double>(k,0)<<", "<<(double)robot_position_matrix.at<double>(k,1)<<", "<<(double)robot_position_matrix.at<double>(k,2) <<"]"<<std::endl;
        test=transformation*D;
        //std::cout <<"D: "<< test.at<double>(0)<<"robot: " <<robot_position_matrix.at<double>(k,0)<<std::endl;
        double d_x=(test.at<double>(0)-(double)robot_position_matrix.at<double>(k,0));
        double d_y=(test.at<double>(1)-(double)robot_position_matrix.at<double>(k,1));
        double d_z=(test.at<double>(2)-(double)robot_position_matrix.at<double>(k,2));
        dev.deviation_x.push_back(d_x);
        dev.deviation_y.push_back(d_y);
        dev.deviation_z.push_back(d_z);
        double current_deviation=sqrt(pow(d_x,2)+pow(d_y,2)+pow(d_z,2));
        dev.deviation_distance.push_back(current_deviation);
        std::cout <<"deviation by coordinates: " <<k<<": ["<<(test.at<double>(0)-(double)robot_position_matrix.at<double>(k,0))<<", "<<(test.at<double>(1)-(double)robot_position_matrix.at<double>(k,1))<<", "<<(test.at<double>(2)-(double)robot_position_matrix.at<double>(k,2))<<"]"<<std::endl;
        average_deviation=average_deviation+current_deviation;
    }
    average_deviation=average_deviation/(number_of_points-1)/POINTS_IN_GRID;
    std::cout<<"Average_deviation: "<< average_deviation<<std::endl;
    dev.average_deviation=average_deviation;
    return dev;
}

/**
 * @brief save_the_deviations Saves the errors between the real and calculated robot positions, then robot positions and vision positions in file data/robot_camera_calibration/backup/deviations_and_data_test_YYYY_MM_DD_HH_MM_SS.ext (if variant is zero) or data/robot_camera_calibration/backup/deviations_and_data_YYYY_MM_DD_HH_MM_SS.ext (otherwise).
 * @param dev structure of deviations that will be saved
 * @param robot_position_matrix
 * @param vision_position_matrix
 * @param variant
 */
void save_the_deviations(deviations dev,cv::Mat_<cv::Vec3d> robot_position_matrix,cv::Mat_<cv::Vec3d> vision_position_matrix, int variant){
    std::string name;
    if(variant==0){
        name="data/robot_camera_calibration/backup/deviations_and_data_test";
    }else{
        name="data/robot_camera_calibration/backup/deviations_and_data";
    }
    cv::FileStorage file(name_of_the_calibration_file(name), cv::FileStorage::WRITE);
    file << "vision_position_matrix" <<vision_position_matrix;
    file << "robot_position_matrix" << robot_position_matrix;
    file << "deviation_x" << dev.deviation_x;
    file << "deviation_y" << dev.deviation_y;
    file << "deviation_z" << dev.deviation_z;
    file << "deviation_distance" << dev.deviation_distance;
    file << "average_deviation" << dev.average_deviation;
}

/**
 * @brief save_the_transformation_to_file Save the trasformation matrix for further use in files data/robot_camera_calibration/backup/calibration_transformation_YYYY_MM_DD_HH_MM_SS.ext and data/robot_camera_calibration/calibration_transformation.ext
 * @param transformation transformation matrix from vision to the robot
 * @param average_deviation average error between calculated and real robot position
 */
void save_the_transformation_to_file(cv::Mat_<double> transformation,double average_deviation){
    //save the backup calibration matrix
    cv::FileStorage file(name_of_the_calibration_file("data/robot_camera_calibration/backup/calibration_transformation"), cv::FileStorage::WRITE);//"calibration_transformation.ext", cv::FileStorage::WRITE);
    file << "transformation" << transformation;
    file << "average_deviation" << average_deviation;
    //save the actual calibration matrix
    cv::FileStorage file1("data/robot_camera_calibration/calibration_transformation.ext", cv::FileStorage::WRITE);
    file1 << "transformation" << transformation;
    file1 << "average_deviation" << average_deviation;
}

/**
 * @brief load_transformation load transformation matrix from file data/robot_camera_calibration/calibration_transformation.ext
 * @return transformation matrix
 */
cv::Mat_<double> load_transformation(){
    cv::Mat calibration_matrix;
    try
    {
        cv::FileStorage storage("data/robot_camera_calibration/calibration_transformation.ext", cv::FileStorage::READ);
        storage["transformation"] >> calibration_matrix;
        storage.release();
        std::cout<<"calibration matrix: "<<calibration_matrix<<std::endl;
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
 * @brief main Calculate the transformation matrix between robot and camera coordinate systems using the recognition of calibration ball. If there is any second argument, the testing of the transformating will run instead of calculationg of the transformation matrix.
 * @param argc if is 2 or more arguments the calibration will be tested otherwise will be founded calibration matrix
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("robot_part/end_effector_position", 1000, robotCallback);
    ros::Subscriber sub2 = nh.subscribe("vision_part/hough_circle_position_with_diameter", 1000, visionCallback);
    //calculate the tansformation matrix
    chatter_pub_joint = nh.advertise<katana_msgs::JointMovementActionGoal>("/katana_arm_controller/joint_movement_action/goal", 1000);
    cv::Vec3d vec = cv::Vec3d(0,0,0);


    //    cv::Mat_<double> joint_position_matrix=(cv::Mat_<double>(6,5) <<
    //                                                                    0.5,0.7,-1.6,-0.7,-3,
    //                                                                    0,0.7,-1.6,-0.7,-3,
    //                                                                    0.5,M_PI/4,-M_PI/3,-0.4,-3,
    //                                                                    0.5,M_PI/4,-M_PI/2+0.2,-0.4,-3,
    //                                                                    0.5,M_PI/4-0.2,-M_PI/2+0.2,-0.4,-3,
    //                                                                    0.2,M_PI/4,-M_PI/2,0,-3);



    //cv::Mat_<double> end_effector_position_matrix=(cv::Mat_<double>(number_of_points,6) <<0.25,0,0.4,0,0,0,
    //                                                       0.4,0,0.4,0,0,0,
    //                                                       0.4,0,0.15,0,0,0,
    //                                                       0.25,0.25,0.4,0,0,0,
    //                                                       0.4,0.1,0.4,0,0,0,
    //                                                       0.37,0.13,0.15,0,0,0);

    if(argc<2){
        int number_of_points=8;
        cv::Mat_<cv::Vec3d> robot_position_matrix=cv::Mat_<cv::Vec3d>((number_of_points-1)*POINTS_IN_GRID,1) << vec; //here is possible to change for how many points we would like to make the calibration
        cv::Mat_<cv::Vec3d> vision_position_matrix=cv::Mat_<cv::Vec3d>((number_of_points-1)*POINTS_IN_GRID,1) << vec;
        cv::Mat_<double> end_effector_position_matrix=(cv::Mat_<double>(number_of_points,6) <<0.35,0.35,0.24,0,0,0,
                                                       0.5,0.1,0.34,0,0,0,
                                                       0.47,0.13,0.09,0,0,0,
                                                       0.35,0.25,0.4,0,0,0,
                                                       0.5,0,0.09,0,0,0,
                                                       0.35,0.25,0.34,0,0,0,
                                                       0.5,0.1,0.34,0,0,0,
                                                       0.47,0.13,0.09,0,0,0);//0.5,-0.13,0.34,0,0,0,//0.44,0.04,0.44,0,0,0,




        vision_and_robot_position tmp_return=measure_vision_positions(nh,end_effector_position_matrix,number_of_points,robot_position_matrix,vision_position_matrix);
        if(tmp_return.robot_position_matrix.empty()){
            return -1;
        }
        robot_position_matrix=tmp_return.robot_position_matrix;
        vision_position_matrix=tmp_return.vision_position_matrix;

        cv::Mat_<double> transformation= FindRigidTransform(vision_position_matrix, robot_position_matrix);
        //NOTE: The transformation need to be used for homogenous vectors (4x1)
        std::cout<<"Robot position matrix: "<<robot_position_matrix<<std::endl;
        std::cout<<"Vision position matrix: "<<vision_position_matrix<<std::endl;
        //std::cout<<"Transformation matrix: "<<transformation<<std::endl;

        deviations dev=calculate_the_deviations(number_of_points,transformation,robot_position_matrix,vision_position_matrix);
        double average_deviation=dev.average_deviation;
        save_the_transformation_to_file(transformation,average_deviation);
        int variant=1; //variant is 1 for the training data
        save_the_deviations(dev, robot_position_matrix,vision_position_matrix, variant);
    }else{
        ROS_INFO("You choosed to test calibration transformation matrix in file data/robot_camera_calibration/calibration_transformation.ext.");

        int number_of_points=5;
        cv::Mat_<cv::Vec3d> robot_position_matrix=cv::Mat_<cv::Vec3d>((number_of_points-1)*POINTS_IN_GRID,1) << vec; //here is possible to change for how many points we would like to make the calibration
        cv::Mat_<cv::Vec3d> vision_position_matrix=cv::Mat_<cv::Vec3d>((number_of_points-1)*POINTS_IN_GRID,1) << vec;
        //choose other points
        cv::Mat_<double> end_effector_position_matrix=(cv::Mat_<double>(number_of_points,6) <<0.45,0,0.1,0,0,0,//0.44,0.04,0.44,0,0,0,
                                                       0.37,-0.125,0.1,0,0,0,
                                                       0.45,0,0.35,0,0,0,
                                                       0.35,0.25,0.35,0,0,0,
                                                       0.35,0.25,0.1,0,0,0);



        vision_and_robot_position tmp_return=measure_vision_positions(nh,end_effector_position_matrix,number_of_points,robot_position_matrix,vision_position_matrix);
        robot_position_matrix=tmp_return.robot_position_matrix;
        vision_position_matrix=tmp_return.vision_position_matrix;

        cv::Mat_<double> transformation= load_transformation();
        if(transformation.empty()){
            return -1;
        }
        //NOTE: The transformation need to be used for homogenous vectors (4x1)
        std::cout<<"Robot position matrix: "<<robot_position_matrix<<std::endl;
        std::cout<<"Vision position matrix: "<<vision_position_matrix<<std::endl;
        //std::cout<<"Transformation matrix: "<<transformation<<std::endl;

        deviations dev=calculate_the_deviations(number_of_points,transformation,robot_position_matrix,vision_position_matrix);
        int variant=0; //variant is 0 for the testing
        save_the_deviations(dev, robot_position_matrix,vision_position_matrix, variant);
    }
    return 0;
}

//https://stackoverflow.com/questions/16357999/current-date-and-time-as-string
/**
 * @brief name_of_the_calibration_file Add in the end of the argument name actual date and time and extension .ext.
 * source: //https://stackoverflow.com/questions/16357999/current-date-and-time-as-string
 * @param name string to that will be the date added
 * @return name with added information
 */
std::string name_of_the_calibration_file(std::string name){
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"_%Y_%m_%d_%H_%M_%S",timeinfo);
    std::string str(buffer);

    std::string output=name+str+".ext";
    return output;

}
/**
 * @brief publish_robot_joint_position publish a message to the katana_msgs::JointMovementActionGoal and with that rotate with Katana joints
 * @param theta1 rotation of first joint katana_motor1_pan_joint
 * @param theta2 rotation of second joint katana_motor2_lift_joint
 * @param theta3 rotation of third joint katana_motor3_lift_joint
 * @param theta4 rotation of fourth joint katana_motor4_lift_joint
 * @param theta5 rotation of fifth joint katana_motor5_wrist_roll_joint
 */
void publish_robot_joint_position(float theta1, float theta2,float theta3,float theta4,float theta5){
    int count = 0;
    katana_msgs::JointMovementActionGoal msg3;
    float r = ((double) rand() / (RAND_MAX))-0.5;
    r = ((double) rand() / (RAND_MAX));//*2*M_PI;
    msg3.goal.jointGoal.name.push_back("katana_motor1_pan_joint");
    msg3.goal.jointGoal.position.push_back(theta1);//0.5); // before were r
    msg3.goal.jointGoal.effort.push_back(0.5);
    msg3.goal.jointGoal.name.push_back("katana_motor2_lift_joint");
    msg3.goal.jointGoal.position.push_back(theta2);//1);
    msg3.goal.jointGoal.effort.push_back(0.5);
    msg3.goal.jointGoal.name.push_back("katana_motor3_lift_joint");
    msg3.goal.jointGoal.position.push_back(theta3);//-1.5);
    msg3.goal.jointGoal.effort.push_back(0.5);
    msg3.goal.jointGoal.name.push_back("katana_motor4_lift_joint");
    msg3.goal.jointGoal.position.push_back(theta4);//-0.8);
    msg3.goal.jointGoal.effort.push_back(0.5);
    msg3.goal.jointGoal.name.push_back("katana_motor5_wrist_roll_joint");
    msg3.goal.jointGoal.position.push_back(theta5);//3.14);
    msg3.goal.jointGoal.effort.push_back(0.5);
    chatter_pub_joint.publish(msg3);
    return;
}

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
