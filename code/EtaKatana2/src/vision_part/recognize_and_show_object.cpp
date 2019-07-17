/** @file recognize_and_show_object.cpp
 *  @brief This node recognize all object that is named as first argument. If there is no argument from default are recognized circles. What is recognized can be changed by calling the service object_to be recognized.srv
 *  This node is a complex recognition tool that combines recognition of cyrcles using hough cyrcle transformation, then general object using YOLOv2 DNN and user defined objects using features based detection.
 *  This node publishes to the topics area_of_recognized_object and hough_circle_position_with_diameter.
 *  And provide the service object_to_be_recognized with that is posible to change the detected object, service variable "name".
 * @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include <vision_part/hough_circle_position.h>
#include <vision_part/hough_circle_position_with_diameter.h>
#include <vision_part/area_of_recognized_object.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
# include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "include/vision_part/camera_parameters.h"
#include "vision_part/object_to_be_recognised.h"


using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace cv::xfeatures2d;

//defines for the darknet
#define PATH_TO_CFG "dark_net/darknet/cfg/yolov2.cfg"
#define PATH_TO_MODEL "dark_net/darknet/yolov2.weights"
#define PATH_TO_CLASS_NAMES "dark_net/darknet/data/coco.names"
#define PATH_TO_USER_DEFINED_CLASS_NAMES "data/user_defined_objects/user_defined_objects.names"
#define MIN_CONFIDENCE 0.1


#define CIRCLES "circles"
#define GENERAL_OBJECT "general_object"
#define USER_DEFINED_OBJECT "user_defined_object"
#define ANY_TYPE "any_type"



//for the difference in length for check of the transformation
#define LIMIT_MULTIPLIER 0.5
#define LIMIT_ANGLE M_PI/4


///
/// \brief object_type Can be "circles" or "general_object" or "user_defined_object"
///
String object_type;
///
/// \brief object_name Concrete name of the object for example: orange, paralen or knife. Set as the first function argument or by the service object_to_be_recognized.
///
String object_name;

std::vector<cv::Vec3f> findCircles(cv::Mat img);
void drawCircles(std::vector<cv::Vec3f> circles, cv::Mat img_for_drawing,std::vector<float> vec_depth);
void print_info_about_circles(std::vector<cv::Vec3f> circles);
ros::Publisher pub_circles;
ros::Publisher pub_area_of_recognized_object;
std::vector<cv::Mat> detect_object(cv::Mat frame, String name, String object_name);
cv::Mat detect_the_saved_object(String object_name, Mat image_from_camera);
vision_part::area_of_recognized_object fill_the_publisher(cv::Mat detected_objects_mat,vision_part::area_of_recognized_object msg_user_defined);

///
/// \brief depth depth map
///
cv::Mat depth;

/**
 * @brief load_object_height Read from a file (data/user_defined_objects/measurements.ext) height in z-axes of the user defined objet.
 * @param name Name of the object for example: paralen.
 * @return
 */
double load_object_height(std::string name){
    double object_height;
    try{
        cv::FileStorage storage("data/user_defined_objects/measurements.ext", cv::FileStorage::READ);
        object_height = (double) storage[name];
        storage.release();
    }catch (int err_code){
        ROS_ERROR("Error during loading the high of the file.");
    }
    return object_height;
}

/**
 * @brief imageCallback Everytime when is new captured image published by a camera, is called this function that runs the recognition proces of the choosed object.
 * @param msg captured camera image
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv::Mat img=cv_bridge::toCvShare(msg, "bgr8")->image; //this is each frame
        if(object_type.compare(CIRCLES)==0 || object_type.compare(ANY_TYPE)==0){
            std::vector<cv::Vec3f> circles=findCircles(img);
            if(circles.empty()){
                std::cout << "No circles found in picture" << std::endl;
                if(!depth.empty()){
                    cv::imshow("RGB", img);
                    cv::imwrite("data/fromCamera.png",img);
                    cv::waitKey(30);
                    cv::imshow("Depth", depth);
                    cv::waitKey(30);
                }
            } else if(depth.empty()){
                std::cout << "Missing depth map" << std::endl;
            }else{
                int length=circles.size();
                std::vector<float> vec_depth(length, 0.0);
                vision_part::hough_circle_position_with_diameter msg2;
                for(int i=0;i<circles.size();i++){
                    cv::Point center= cv::Point(circles[i][0], circles[i][1]);
                    double pixel_cm=PIXELS_TO_CM;
                    cv::Point center2= cv::Point(circles[i][0]+DEPTH_COLOUR_MOVE_X*pixel_cm, circles[i][1]+DEPTH_COLOUR_MOVE_Y*pixel_cm);
                    if(center2.x<depth.size().width && center2.y<depth.size().height){
                        vec_depth[i]=depth.at<float>(center2);
                        if(vec_depth[i]>0.0001){
                            //published will be last circle
                            msg2.x= circles[i][0];
                            msg2.y= circles[i][1];
                            msg2.z= vec_depth[i];
                            msg2.diameter= 2*circles[i][2];
                            msg2.height=load_object_height(object_name);
                            //to send the forst detected circle
                            pub_circles.publish(msg2);
                            drawCircles(circles ,img, vec_depth);
                            cv::imshow("RGB", img);
                            cv::imwrite("data/fromCamera.png",img);
                            cv::waitKey(30);
                            cv::imshow("Depth", depth);
                            cv::waitKey(30);
                        }else{
                            ROS_WARN("Zero depth at the center of the recognized object, probably due to the heterogeneity of the object or its convexity or concavityof the object.");
                        }
                        break;
                    }else{
                        ROS_WARN("Object out of depth range.");
                    }
                }
            }
        }
        if(object_type.compare(GENERAL_OBJECT)==0 || object_type.compare(ANY_TYPE)==0){

            if(depth.empty()){
                std::cout << "Missing depth map" << std::endl;
            }else{
                for(int m=0; m<1;m++){ //m<4
                    cv::Mat img_for_yolo=img.clone();
                    int rotation=m*90;
                    std::stringstream name_stream;
                    name_stream << "Picture was rotated by: " << rotation << " degrees.";
                    std::string name_of_the_window = name_stream.str();
                    //String name_of_the_vindow="rotation by "+ tmp +" degrees";
                    //cv::rotate(img_for_yolo, img_for_yolo,m);//corresponds to cv rotations example: cv::ROTATE_90_CLOCKWISE
                    std::vector<cv::Mat> detected_objects=detect_object(img_for_yolo, name_of_the_window, object_name); //new detection
                    vision_part::area_of_recognized_object msg_user_defined;
                    for(int k=0;k<detected_objects.size();k++){
                        cv::Mat detected_objects_mat=detected_objects.at(k);
                        detected_objects_mat.at<double>(0,4)=0.0;
                        msg_user_defined=fill_the_publisher(detected_objects_mat,msg_user_defined);
                        msg_user_defined.length_z.push_back(-1);//No information about high from file
                    }
                    pub_area_of_recognized_object.publish(msg_user_defined);
                    if(!detected_objects.empty()){
                        cv::Mat tmp=detected_objects[0];
                        std::cout<<"vector: "<<tmp<<endl;
                    }
                }
            }
        }
        if(object_type.compare(USER_DEFINED_OBJECT)==0||object_type.compare(ANY_TYPE)==0){
            if(depth.empty()){
                std::cout << "Missing depth map" << std::endl;
            }else{
                cv::Mat detected_objects_mat=detect_the_saved_object(object_name, img);
                if(!detected_objects_mat.empty()){
                    vision_part::area_of_recognized_object msg_user_defined;
                    msg_user_defined=fill_the_publisher(detected_objects_mat,msg_user_defined);
                    msg_user_defined.length_z.push_back(load_object_height(object_name));
                    pub_area_of_recognized_object.publish(msg_user_defined);
                }
                cv::imshow("Depth", depth);
                //cv::waitKey(30);
            }
        }
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


/**
 * @brief fill_the_publisher take the information about detected object and prepare them for a message to be published.
 * @param detected_objects_mat Mat_<double>(1,13) [x, y, size_x, size_y, rotation, corner_xRightTop, corner_yRightTop, corner_xRightBottom, corner_yRightBottom, corner_xLeftTop, corner_yLeftTop, corner_xLeftBottom, corner_yLeftBottom]
 * @param msg_user_defined Message that will be fill in by the information about the recognized object.
 * @return msg_user_defined
 */
vision_part::area_of_recognized_object fill_the_publisher(cv::Mat detected_objects_mat,vision_part::area_of_recognized_object msg_user_defined){
    if(!detected_objects_mat.empty()){
        double pixel_cm=PIXELS_TO_CM;
        cv::Point center= cv::Point(detected_objects_mat.at<double>(0,0)+DEPTH_COLOUR_MOVE_X*pixel_cm, detected_objects_mat.at<double>(0,1)+DEPTH_COLOUR_MOVE_Y*pixel_cm); //rewriten centre
        if(center.x<depth.size().width && center.y<depth.size().height){
            double z=depth.at<float>(center);
            msg_user_defined.center_x.push_back(detected_objects_mat.at<double>(0,0));
            msg_user_defined.center_y.push_back(detected_objects_mat.at<double>(0,1));
            msg_user_defined.center_z.push_back(z);
            msg_user_defined.length_x.push_back(detected_objects_mat.at<double>(0,2));
            msg_user_defined.length_y.push_back(detected_objects_mat.at<double>(0,3));
            msg_user_defined.rotation_z.push_back(detected_objects_mat.at<double>(0,4));
            msg_user_defined.right_top_x.push_back(detected_objects_mat.at<double>(0,5));
            msg_user_defined.right_top_y.push_back(detected_objects_mat.at<double>(0,6));
            msg_user_defined.right_down_x.push_back(detected_objects_mat.at<double>(0,7));
            msg_user_defined.right_down_y.push_back(detected_objects_mat.at<double>(0,8));
            msg_user_defined.left_top_x.push_back(detected_objects_mat.at<double>(0,9));
            msg_user_defined.left_top_y.push_back(detected_objects_mat.at<double>(0,10));
            msg_user_defined.left_down_x.push_back(detected_objects_mat.at<double>(0,11));
            msg_user_defined.left_down_y.push_back(detected_objects_mat.at<double>(0,12));
            ROS_INFO("Object was recognized on position: [%.2lf, %.2lf, %.2lf] with size: [%.2lf, %.2lf] with rotation in picture cooordinates: %.2lf°",detected_objects_mat.at<double>(0,0),detected_objects_mat.at<double>(0,1),z,detected_objects_mat.at<double>(0,2),detected_objects_mat.at<double>(0,3),detected_objects_mat.at<double>(0,4)*180/M_PI);
        }else{
            ROS_WARN("Object out of depth map range.");
        }
    }
    return msg_user_defined;
}

/**
 * @brief imageCallbackDepth Save captured depth image to the global variable depth.
 * @param msg2 captured depth image
 */
void imageCallbackDepth(const sensor_msgs::ImageConstPtr& msg2){
    try{
        cv_bridge::CvImageConstPtr depth_img_cv;
        // Get the ROS image to openCV
        depth_img_cv = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_16UC1);
        // Convert the uints to floats
        depth_img_cv->image.convertTo(depth, CV_32F, 0.001);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("Problem with depth map %s.", msg2->encoding.c_str());
    }
}


/**
 * @brief drawCircles Draw the recognized circle and also the moved circle by the translation between depth and GRB image.
 * Recognized circle is drawn by a light green and the moved circle is grown in dark green by both circles are described by coordinates.
 * @param circles vector of vectors from three floats x, y coordinates of the circle middle and circle radius
 * @param img_for_drawing cv::Mat inside it will be the circles drawn
 * @param vec_depth vector with depth on middle of the circle
 */
void drawCircles(std::vector<cv::Vec3f> circles, cv::Mat img_for_drawing,std::vector<float> vec_depth){
    for(int i=0;i<circles.size();i++){
        cv::Point center= cv::Point(circles[i][0], circles[i][1]);
        double depth_rgb = depth.at<float>(center);
        cv::circle(img_for_drawing, center , (int)(circles[i][2]), cv::Scalar( 0, 255, 0 ),5);
        cv::circle(img_for_drawing, center , 1, cv::Scalar( 0, 255, 0 ),5);
        char text [50];
        std::sprintf(text , "[%i, %i, %.2f]", (int)circles[i][0],(int)circles[i][1],depth_rgb);
        cv::putText(img_for_drawing, text, cvPoint((int)circles[i][0]-(int)circles[i][2]-10,(int)circles[i][1]-(int)circles[i][2]-10),cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0,255,0), 1, CV_AA);
        double pixel_cm=PIXELS_TO_CM;
        cv::Point center2= cv::Point(circles[i][0]+DEPTH_COLOUR_MOVE_X*pixel_cm, circles[i][1]+DEPTH_COLOUR_MOVE_Y*pixel_cm);
        cv::circle(img_for_drawing, center2 , (int)(circles[i][2]), cv::Scalar( 0, 125, 0 ),5);
        cv::circle(img_for_drawing, center2 , 1, cv::Scalar( 0, 125, 0 ),5);
        char text2 [50];
        double depth2=vec_depth[i];
        std::sprintf(text2 , "[%i, %i, %.2f]",(int)(circles[i][0]+DEPTH_COLOUR_MOVE_X*pixel_cm), (int)(circles[i][1]+DEPTH_COLOUR_MOVE_Y*pixel_cm), depth2);
        cv::putText(img_for_drawing, text2, cvPoint((int)circles[i][0],(int)circles[i][1]+(int)circles[i][2]+DEPTH_COLOUR_MOVE_Y*pixel_cm+15),cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0,125,0), 1, CV_AA);
        //cv::circle(depth, center2 , 1, cv::Scalar( 0, 255, 0 ),5);
    }

}

/**
 * @brief findCircles Function that will find circles in the image using hough circles transformation.
 * The image will be recalculated to the HSV picture and on the chanel saturation is applied hough circle transformation.
 * @param img BGR picture
 * @return Vec3f with 2D coordinates of middle of the circle and radius of the circle.
 */
std::vector<cv::Vec3f> findCircles(cv::Mat img){
    std::vector<cv::Vec3f> circles;
    cv::Mat img_with_circles=img.clone();
    cv::cvtColor(img_with_circles, img_with_circles, CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(img_with_circles, channels);
    cv::GaussianBlur( channels[2], channels[2], cv::Size(11, 11), 0, 0 );
    if(HD){
        cv::HoughCircles(channels[2], circles, CV_HOUGH_GRADIENT, 1,200, 100, 50, 0, 100);// pro hd
    }else{
        cv::HoughCircles(channels[2], circles, CV_HOUGH_GRADIENT, 2,200, 100, 50, 0, 18); // pro sd
    }
    print_info_about_circles(circles);
    return circles;
}

/**
 * @brief distance_points
 * @param a first point
 * @param b second point
 * @return distance
 */
float distance_points(Point_<float> a,Point_<float> b){
    float d=(float)sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    return d;
}

/**
 * @brief angle_using_cosin_law Calculate the angle by the point c.
 * @param a triangle top
 * @param b triangle top
 * @param c trinagle top on them will be the angle calculated
 * @return Angle by the point c.
 */
float angle_using_cosin_law(Point_<float> a,Point_<float> b,Point_<float> c){
    float side_a=distance_points(b,c);
    float side_b=distance_points(a,c);
    float side_c=distance_points(b,a);
    float gamma= acos((pow(side_a,2)+pow(side_b,2)-pow(side_c,2))/(2*side_a*side_b));
    return gamma;
}

/**
 * @brief control_angles Control if the angles by point c is in defined range.
 * The predefined range in PI +- constant LIMIT_ANGLE
 * note: The control is important for controling the proper localization of the object.
 * @param a triangle top
 * @param b triangle top
 * @param c trinagle top on them will be the angle calculated
 * @return True if the angle is in predefined range.
 */
bool control_angles(Point_<float> a,Point_<float> b,Point_<float> c){
    bool fault=false;
    float angle=angle_using_cosin_law(a,b,c);
    if(angle<LIMIT_ANGLE ||angle>(M_PI/2+LIMIT_ANGLE)){
        //std::cout<<"angle: "<<angle<<endl;
        fault=true;
    }
    return fault;
}

/**
 * @brief transformation_sence Control if the scene_corners indicate the corners of the rectangle with a tolerance of some deviation.
 * Controled informations are: ratio between sides in pattern and in localized rectangle is similar, the length of the parallel sides of the rectangle is similar, sides are bigger the 10 pixels, the inner angles of the rectangle are within tolerance.
 * @param obj_corners 2D coordinates of the corners of the pattern image.
 * @param scene_corners Corners of the localized quadrilateral.
 * @return True, if the quadrilateral is in tolerance to be for us a rectangle.
 */
bool transformation_sence(std::vector<Point2f> obj_corners,std::vector<Point2f> scene_corners){
    bool sence =true;
    float d1=distance_points(scene_corners[0],scene_corners[1]);
    float d2=distance_points(scene_corners[1],scene_corners[2]);
    float d3=distance_points(scene_corners[2],scene_corners[3]);
    float d4=distance_points(scene_corners[3],scene_corners[0]);
    float ratio_obj=(float)abs(obj_corners[0].x-obj_corners[1].x)/(float)abs(obj_corners[1].y-obj_corners[2].y);
    float ration_scene=d1/d2;
    if((1+LIMIT_MULTIPLIER)*ratio_obj<ration_scene || LIMIT_MULTIPLIER*ratio_obj>ration_scene){
        sence=false;
    }
    if((1+LIMIT_MULTIPLIER)*d1<d3 || LIMIT_MULTIPLIER*d1>d3||(1+LIMIT_MULTIPLIER)*d3<d1 || LIMIT_MULTIPLIER*d3>d1||(1+LIMIT_MULTIPLIER)*d4<d2 || LIMIT_MULTIPLIER*d4>d2||(1+LIMIT_MULTIPLIER)*d2<d4 || LIMIT_MULTIPLIER*d2>d4){
        sence=false;
    }
    if(d1<10 ||d2<10){
        sence=false;
    }
    if(control_angles(scene_corners[0],scene_corners[2],scene_corners[1])||control_angles(scene_corners[1],scene_corners[3],scene_corners[2])||control_angles(scene_corners[2],scene_corners[0],scene_corners[3])||control_angles(scene_corners[3],scene_corners[1],scene_corners[0])){
        sence=false;
    }
    return sence;
}



/**
 * @brief detect_the_saved_object The detection of the user defined object using the feature based detection.
 * Firstly load the pattern image, then change captured image to grayscale. Then use SURF for detecting the features, the features are linked by FLANN based matcher.
 * After that the homography between pattern and detected object is calculated.
 * Then is the calculation controled, if for a rectangular pattern image was found also approximatly rectangular area.
 * At the end are calculeted all the information about the object.
 * sources: HUAMAN, Ana. Feature Matching with FLANN [online]. In: . OpenCV, 2014 [cit. 2019-04-10]. Available from: https://docs.opencv.org/3.1.0/d5/d6f/tutorial_feature_flann_matcher.html
 * CLAUDIU. Object detection with SURF, KNN, FLANN, OpenCV 3.X and CUDA [online]. In: . 2016 [cit. 2019-04-10]. Available from: http://www.coldvision.io/2016/06/27/object-detection-surf-knn-flann-opencv-3-x-cuda/
 * @param object_name What will be detected, example: paralen.
 * @param image_from_camera RGB image captured by camera, where we will look for the object.
 * @return Matrix with information about the object. Mat_<double>(1,13) [x, y, size_x, size_y, rotation, corner_xRightTop, corner_yRightTop, corner_xRightBottom, corner_yRightBottom, corner_xLeftTop, corner_yLeftTop, corner_xLeftBottom, corner_yLeftBottom]
 */
//sources: HUAMAN, Ana. Feature Matching with FLANN [online]. In: . OpenCV, 2014 [cit. 2019-04-10]. Available from: https://docs.opencv.org/3.1.0/d5/d6f/tutorial_feature_flann_matcher.html
//         CLAUDIU. Object detection with SURF, KNN, FLANN, OpenCV 3.X and CUDA [online]. In: . 2016 [cit. 2019-04-10]. Available from: http://www.coldvision.io/2016/06/27/object-detection-surf-knn-flann-opencv-3-x-cuda/
cv::Mat detect_the_saved_object(String object_name, Mat image_from_camera){
    cv::Mat info_about_found_objects;
    std::stringstream name_stream;
    name_stream << "data/user_defined_objects/" << object_name<< ".jpg";
    std::string relative_way_to_object = name_stream.str();
    Mat img_1 = imread( relative_way_to_object, CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_2;
    cv::cvtColor(image_from_camera, img_2, CV_BGR2GRAY);

    try {
        if( !img_1.data || !img_2.data )
        { std::cout<< " --(!) Error reading images " << std::endl; return info_about_found_objects; }
        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create();
        detector->setHessianThreshold(minHessian);
        std::vector<KeyPoint> keypoints_1, keypoints_2;
        Mat descriptors_1, descriptors_2;
        detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
        detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );
        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_1, descriptors_2, matches,2 );
        double max_dist = 0; double min_dist = 10;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors_1.rows; i++ )
        { double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        vector<float> x_matches;
        vector<float> y_matches;
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_1.rows; i++ )
        {
            if( matches[i].distance <= max(5*min_dist, 0.02) ){
                good_matches.push_back( matches[i]);
            }
        }
        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>());
        //https://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
        }

        Mat H = findHomography( obj, scene, CV_RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_1.cols, 0 );
        obj_corners[2] = cvPoint( img_1.cols, img_1.rows );
        obj_corners[3] = cvPoint( 0, img_1.rows );
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);
        if(transformation_sence(obj_corners,scene_corners)){
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( img_1.cols, 1), scene_corners[1] + Point2f( img_1.cols, 1), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( img_1.cols, 1), scene_corners[2] + Point2f( img_1.cols, 1), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( img_1.cols, 1), scene_corners[3] + Point2f( img_1.cols, 1), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( img_1.cols, 1), scene_corners[0] + Point2f( img_1.cols, 1), Scalar( 0, 255, 0), 4 );

            Point2f middle=(scene_corners[0]+Point2f( img_1.cols, 1)+scene_corners[1]+Point2f( img_1.cols, 1)+scene_corners[2]+Point2f( img_1.cols, 1)+scene_corners[3]+Point2f( img_1.cols, 1))/4;
            double d_x=((double)distance_points(scene_corners[0],scene_corners[1])+(double)distance_points(scene_corners[2],scene_corners[3]))/2;
            double d_y=((double)distance_points(scene_corners[1],scene_corners[2])+(double)distance_points(scene_corners[3],scene_corners[0]))/2;

            double scale= (distance_points(scene_corners[0],scene_corners[1])+distance_points(scene_corners[1],scene_corners[2])+distance_points(scene_corners[2],scene_corners[3])+distance_points(scene_corners[3],scene_corners[0]))/(distance_points(obj_corners[0],obj_corners[1])+distance_points(obj_corners[1],obj_corners[2])+distance_points(obj_corners[2],obj_corners[3])+distance_points(obj_corners[3],obj_corners[0]));

            double rot_z=angle_using_cosin_law(scene_corners[0] + Point2f( img_1.cols, 1),-obj_corners[2]/2*scale + middle,middle);//((double)atan2(scene_corners[1].y-scene_corners[0].y,scene_corners[1].x-scene_corners[0].x));//+(double)atan2(scene_corners[3].y-scene_corners[2].y,scene_corners[3].x-scene_corners[2].x))/2; // camera has rotated z axes; // camera has rotated z axes
            Point_<float> a=scene_corners[0] + Point2f( img_1.cols, 1);
            Point_<float> c=middle;
            float angle_tmp=atan2(a.y-c.y,a.x-c.x)+3*M_PI/4;
            if(angle_tmp<0||angle_tmp>M_PI){
                rot_z=2*M_PI-rot_z;
            }
            //ROTATION WITH Z AXIS LOOKING IN THE TABLE
            cv::circle(img_matches,  middle, 30, cv::Scalar( 0, 255, 0 ),5);


            double pixel_cm=PIXELS_TO_CM;
            cv::Point recalculated_middle=middle-Point2f( img_1.cols, 1);
            cv::Point center= cv::Point(recalculated_middle.x+DEPTH_COLOUR_MOVE_X*pixel_cm, recalculated_middle.y+DEPTH_COLOUR_MOVE_Y*pixel_cm); //rewriten centre
            //cv::circle(depth,  center, 30, cv::Scalar( 0, 255, 0 ),5);
            if(center.x<depth.size().width && center.y<depth.size().height){
                char text2 [50];
                std::sprintf(text2, "[%i, %i, %.2f]",(int)(middle.x), (int)(middle.y), depth.at<float>(center));
                cv::putText(img_matches, text2, middle,cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(255,0,0), 1, CV_AA);
                info_about_found_objects=(Mat_<double>(1,13) <<(double)recalculated_middle.x,(double)recalculated_middle.y,d_x,d_y, rot_z,(scene_corners[0]).x,(scene_corners[0]).y,(scene_corners[3]).x,(scene_corners[3]).y,(scene_corners[1]).x,(scene_corners[1]).y,(scene_corners[2]).x,(scene_corners[2]).y);
            }else{
                ROS_WARN("Object out of depth range.");
            }
        }else{
            ROS_WARN("To low quality of the recognized corespondences. It can be caused bz to low resolution or bed light conditions.");

        }
        //-- Show detected matches
        imshow( "RGB", img_matches );
        //waitKey(0);

        if (waitKey(1) >= 0) return info_about_found_objects;

    } catch (cv::Exception ex) {
        ROS_WARN("Not enought corresponding points found.");
    }

    return info_about_found_objects;
}

//End of sources: HUAMAN, Ana. Feature Matching with FLANN [online]. In: . OpenCV, 2014 [cit. 2019-04-10]. Available from: https://docs.opencv.org/3.1.0/d5/d6f/tutorial_feature_flann_matcher.html
//         CLAUDIU. Object detection with SURF, KNN, FLANN, OpenCV 3.X and CUDA [online]. In: . 2016 [cit. 2019-04-10]. Available from: http://www.coldvision.io/2016/06/27/object-detection-surf-knn-flann-opencv-3-x-cuda/


/**
 * @brief sort_by_confidence comparison if a or b matrix on coordinate [0,4] is bigger
 * @param a
 * @param b
 * @return return true if a matrix on coordinate [0,4] is bigger
 */
int sort_by_confidence(cv::Mat& a,cv::Mat& b){
    return a.at<double>(0,4)>b.at<double>(0,4);
}


/**
 * @brief detect_object Detect the general objects from coco dataset using YOLOv2 DNN.
 * Read the darknet from file, control if object_name exist in the database, detect the object, filer the object with higher confidence, that a trashold MIN_CONFIDENCE, filter only choosed category object_name, callculate information about object, sort founded objects by confidence.
 * source: DE OLIVEIRA FARIA, Alessandro. YOLO DNNs [online]. In: . OpenCV, 2017 [cit. 2019-04-03]. Available from: https://docs.opencv.org/3.4.0/da/d9d/tutorial_dnn_yolo.html
 * @param frame captured camera image
 * @param name name of the frame for the displayed RGB picture
 * @param object_name Name of the object, that is recognized, example: banana.
 * @return vector of the matrixes of the description of the fouded objects sorted by the confidence from bigist to the smallest. The information are: Mat_<double>(1,13) [x, y, size_x, size_y, confidence, corner_xRightTop, corner_yRightTop, corner_xRightBottom, corner_yRightBottom, corner_xLeftTop, corner_yLeftTop, corner_xLeftBottom, corner_yLeftBottom]
 */
//source: DE OLIVEIRA FARIA, Alessandro. YOLO DNNs [online]. In: . OpenCV, 2017 [cit. 2019-04-03]. Available from: https://docs.opencv.org/3.4.0/da/d9d/tutorial_dnn_yolo.html
static const char* about =
        "This sample uses You only look once (YOLO)-Detector (https://arxiv.org/abs/1612.08242) to detect objects on camera/video/image.\n"
        "Models can be downloaded here: https://pjreddie.com/darknet/yolo/\n"
        "Default network is 416x416.\n"
        "Class names can be downloaded here: https://github.com/pjreddie/darknet/tree/master/data\n";
static const char* params =
        "{ help           | false | print usage         }"
        "{ cfg            |       | model configuration }"
        "{ model          |       | model weights       }"
        "{ camera_device  | 0     | camera device number}"
        "{ source         |       | video or image for detection}"
        "{ min_confidence | 0.24  | min confidence      }"
        "{ class_names    |       | File with class names, [PATH-TO-DARKNET]/data/coco.names }";
std::vector<cv::Mat> detect_object(cv::Mat frame, String name, String object_name)
{
    std::vector<cv::Mat> info_about_found_objects;
    String modelConfiguration = PATH_TO_CFG;
    String modelBinary = PATH_TO_MODEL;
    dnn::Net net = readNetFromDarknet(modelConfiguration, modelBinary);
    //cout<<"model loaded. "<<endl;
    if (net.empty())
    {
        cerr << "Can't load network by using the following files: " << endl;
        cerr << "cfg-file:     " << modelConfiguration << endl;
        cerr << "weights-file: " << modelBinary << endl;
        cerr << "Models can be downloaded here:" << endl;
        cerr << "https://pjreddie.com/darknet/yolo/" << endl;
        exit(-1);
    }
    vector<string> classNamesVec;
    ifstream classNamesFile(PATH_TO_CLASS_NAMES);
    bool object_exist=false;
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className)){
            classNamesVec.push_back(className);
            if(object_name.compare(className)==0){
                object_exist=true;
            }
        }
        if(!object_exist){
            ROS_WARN("Object %s is not a general object please check the file for names of available objects %s.",object_name.c_str(), PATH_TO_CLASS_NAMES);
            ros::shutdown();
        }
    }
    if (frame.empty())
    {
        waitKey();
        return info_about_found_objects;
    }
    if (frame.channels() == 4){
        cvtColor(frame, frame, COLOR_BGRA2BGR);
    }
    Mat inputBlob = blobFromImage(frame, 1 / 255.F, Size(416, 416), Scalar(), true, false); //Convert Mat to batch of images
    net.setInput(inputBlob, "data");                   //set the network input
    Mat detectionMat = net.forward("detection_out");   //compute output
    vector<double> layersTimings;
    double freq = getTickFrequency() / 1000;
    double time = net.getPerfProfile(layersTimings) / freq;
    ostringstream ss;
    ss << "FPS: " << 1000/time << " ; time: " << time << " ms";
    putText(frame, ss.str(), Point(20,20), 0, 0.5, Scalar(0,0,255));
    float confidenceThreshold = MIN_CONFIDENCE;//parser.get<float>("min_confidence");
    bool was_object_found=false;
    vector<float> confidence_vec;
    for (int i = 0; i < detectionMat.rows; i++)
    {
        const int probability_index = 5;
        const int probability_size = detectionMat.cols - probability_index;
        float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);
        size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
        float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
        if (confidence > confidenceThreshold && classNamesVec[objectClass].compare(object_name)==0) //with deleting classNamesVec[objectClass].compare(object_name)==0 will inform about all recognized objects
        {
            confidence_vec.push_back(confidence);
            was_object_found=true;
            float x = detectionMat.at<float>(i, 0);
            float y = detectionMat.at<float>(i, 1);
            float width = detectionMat.at<float>(i, 2);
            float height = detectionMat.at<float>(i, 3);
            int xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
            int yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
            int xRightTop = static_cast<int>((x + width / 2) * frame.cols);
            int yRightTop = static_cast<int>((y + height / 2) * frame.rows);
            int xLeftTop = static_cast<int>((x - width / 2) * frame.cols);
            int yLeftTop = static_cast<int>((y - height / 2) * frame.rows);
            int xRightBottom = static_cast<int>((x + width / 2) * frame.cols);
            int yRightBottom = static_cast<int>((y + height / 2) * frame.rows);
            Rect object(xLeftBottom, yLeftBottom,
                        xRightTop - xLeftBottom,
                        yRightTop - yLeftBottom);
            rectangle(frame, object, Scalar(0, 255, 0));
            double rot_z=0;
            cv::Mat_<double> tmp_vec=(Mat_<double>(1,13) <<(double)(x*frame.cols),(double)(y*frame.rows),(double)(xRightTop - xLeftBottom),(double)(yRightTop - yLeftBottom), (double)confidence,(double)xRightTop,(double)yRightTop, (double)xRightBottom,(double)yRightBottom, (double)xLeftTop, (double)yLeftTop, (double) xLeftBottom, (double) yLeftBottom);//(static_cast<int>((x) * frame.cols),static_cast<int>((y) * frame.rows), width, height);
            std::cout<<tmp_vec<<std::endl;
            info_about_found_objects.push_back(tmp_vec);
            if (objectClass < classNamesVec.size())
            {
                ss.str("");
                ss << confidence;
                String conf(ss.str());
                String label = String(classNamesVec[objectClass]) + ": " + conf;
                int baseLine = 0;
                Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom ),
                                      Size(labelSize.width, labelSize.height + baseLine)),
                          Scalar(255, 255, 255), CV_FILLED);
                putText(frame, label, Point(xLeftBottom, yLeftBottom+labelSize.height),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
                ROS_INFO("%s", label.c_str());
            }
            else
            {
                cout << "Class: " << objectClass << endl;
                cout << "Confidence: " << confidence << endl;
                cout << " " << xLeftBottom
                     << " " << yLeftBottom
                     << " " << xRightTop
                     << " " << yRightTop << endl;
            }
        }

    }
    sort(info_about_found_objects.begin(),info_about_found_objects.end(), sort_by_confidence);

    if(!was_object_found){
        ROS_INFO("%s not found",object_name.c_str());
    }
    imshow("RGB", frame);
    cv::imshow("Depth", depth);
    // waitKey(0);
    if (waitKey(1) >= 0) return info_about_found_objects;
    return info_about_found_objects;
}
// End of code from: DE OLIVEIRA FARIA, Alessandro. YOLO DNNs [online]. In: . OpenCV, 2017 [cit. 2019-04-03]. Dostupné z: https://docs.opencv.org/3.4.0/da/d9d/tutorial_dnn_yolo.html


/**
 * @brief print_info_about_circles print 2D coordinates of the middle of the circle and the radius of the circle
 * @param circles
 */
void print_info_about_circles(std::vector<cv::Vec3f> circles){
    std::cout << "---------------FOUNDED CIRCLES--------------------"<< std::endl;
    for(int i=0;i<circles.size();i++){
        std::cout << "circle " << i <<  " has value x: " << circles[i][0]<<" and value y: " << circles[i][1] <<" and radius value: " << circles[i][2] << std::endl;
    }
}

/**
 * @brief is_defined Control if the object name is in the file with object names.
 * @param path path to the file with names
 * @param object_type type of the object, general or user defined objects
 * @param object_name name of the object, example orange, paralen
 * @return true if the object exist
 */
bool is_defined(String path, String object_type, String object_name){
    ifstream classNamesFile(path.c_str());
    bool object_exist=false;
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className)){
            if(object_name.compare(className)==0){
                object_exist=true;
                ROS_INFO("It was choosen to recognize object %s with type %s.",object_name.c_str(),object_type.c_str());
            }
        }
    }
    return object_exist;
}
/**
 * @brief detect_object_class Detect if the object belongs to the circles, general object or user defined object and dependent on that set the global variable object_type.
 * @param name name of the object, example: banana or paralen
 */
void detect_object_class(string name){
    if(is_defined(PATH_TO_USER_DEFINED_CLASS_NAMES, "user_defined_object", name)){
        object_type="user_defined_object";
    }
    else if(is_defined(PATH_TO_CLASS_NAMES, "general_object", name)){
        object_type="general_object";
    }else if (name.compare(CIRCLES)==0) {
        object_type="circles";
        ROS_INFO("It was choosen to recognize %s.", name.c_str());
    }else{
        ROS_WARN("Nonexisting object %s, please choose some of defined objects or define a new object. Object can be defined by adding a picture in %s in name_of_object.jpg format and adding the name_of_object in the file user_defined_objects.names",name.c_str(),PATH_TO_USER_DEFINED_CLASS_NAMES);
    }
}

/**
 * @brief change_object Service that change the object name.
 * note: Used for example by the pick and place node.
 * @param req name of the node
 * @param res true
 * @return true
 */
bool change_object(vision_part::object_to_be_recognised::Request  &req,
                   vision_part::object_to_be_recognised::Response &res)
{
    object_name=req.name;
    detect_object_class(object_name);
    res.ok=true;
    return true;
}

/**
 * @brief main Initialize the node and subcribers of RGB and depth image, publishers of hough_circle_position_with_diameter and area_of_recognized_object and also the service change object.
 * @param argc one, if default variant circles is used, two if there is also the object_name
 * @param argv first argument can be the object_name
 * @return 0
 */
int main(int argc, char **argv){
    if(argc>1){
        object_name=argv[1];
        detect_object_class(argv[1]);
    }else{
        //ROS_WARN("Not enought input arguments, please add argument with name of the object you want to recognize.");
        object_name="circles";
        object_type="circles";
        ROS_WARN("No object was written as argument, it is chosen default variant circles. It can be modified by service vision_part/object_to_be_recognised.");

    }
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    pub_circles= nh.advertise<vision_part::hough_circle_position_with_diameter>("/vision_part/hough_circle_position_with_diameter", 1000);
    pub_area_of_recognized_object= nh.advertise<vision_part::area_of_recognized_object>("/vision_part/area_of_recognized_object", 1000);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub;
    image_transport::Subscriber sub_depth;
    if(HD){
        sub= it.subscribe("/kinect2/hd/image_color", 1, imageCallback); //hd//it.subscribe("/kinect2/sd/image_color_rect", 1, imageCallback);//sd ////
        sub_depth=it.subscribe("/kinect2/hd/image_depth_rect", 1, imageCallbackDepth); //hd//it.subscribe("/kinect2/sd/image_depth", 1, imageCallbackDepth);//sd ////
    }else{
        sub= it.subscribe("/kinect2/sd/image_color_rect", 1, imageCallback);//sd ////
        sub_depth=it.subscribe("/kinect2/sd/image_depth_rect", 1, imageCallbackDepth);//sd ////
    }
    ros::ServiceServer service = nh.advertiseService("vision_part/object_to_be_recognised", change_object);
    ros::spin();
    return 0;


}



