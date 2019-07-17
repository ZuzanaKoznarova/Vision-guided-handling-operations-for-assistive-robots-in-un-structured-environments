/** @file depth_on_RGB_coordinates.cpp
 *  @brief This node subcribes the depth and store it in a global variable and provide the service to get the dept on concrete coordinate x,y.
 *  @author Zuzana Koznarova (zuzanazofka@gmail.com)
 */
#include "ros/ros.h"
#include "vision_part/get_depth.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include "include/vision_part/camera_parameters.h"


cv::Mat depth;

/**
 * @brief imageCallbackDepth is a callback function for the actualizing the depth information in  the global variable cv::Mat depth
 * @param msg2 message with depth image from the camera
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
 * @brief get_object_depth service that has as the input x and y coordinate and find the depth on asked position
 * @param req service request wit x and y coordinates
 * @param res response is double res.z that is the depth coordinate
 * @return returns whether the service has been successfully completed
 */
bool get_object_depth(vision_part::get_depth::Request  &req,
                      vision_part::get_depth::Response &res)
{
    if(!depth.empty()){
        double pixel_cm=PIXELS_TO_CM;
        cv::Point center2= cv::Point(req.x+DEPTH_COLOUR_MOVE_X*pixel_cm, req.y+DEPTH_COLOUR_MOVE_Y*pixel_cm);
        if(center2.x<depth.size().width && center2.y<depth.size().height){
            double vec_depth=depth.at<float>(center2);
            if(vec_depth>0.01){
                res.z=vec_depth;
            }
            ROS_INFO("Depth coordinate on [x=%ld, y=%ld] is %lf", (long int)center2.x, (long int)center2.y, vec_depth);
        }else{
            ROS_WARN("Out of picture.");
            return false;
        }
    }else{
        ROS_WARN("Empty depth");
        return false;
    }
    return true;
}

/**
 * @brief main this node keeps track of the current depth map and provides service with depth information on concrete pixel
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_on_RGB_coordinates");
    ros::NodeHandle nh;
    
    image_transport::Subscriber sub_depth;
    image_transport::ImageTransport it(nh);
    if(HD){
        sub_depth=it.subscribe("/kinect2/hd/image_depth_rect", 1, imageCallbackDepth); //hd//it.subscribe("/kinect2/sd/image_depth", 1, imageCallbackDepth);//sd ////
    }else{
        sub_depth=it.subscribe("/kinect2/sd/image_depth", 1, imageCallbackDepth);//sd ////
    }
    ros::ServiceServer service = nh.advertiseService("vision_part/get_depth", get_object_depth);
    ros::spin();
    
    return 0;
}
