
// roslaunch zed_wrapper zed.launch

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <string>

#define RAD2DEG 57.295779513


using namespace std;
using namespace message_filters;


static const std::string OPENCV_WINDOW = "Image window";

/**
 * Subscriber callbacks
 */


class Robot {
public:
    double odomX, odomY, odomZ, odomRoll, odomPitch, odomYaw;
    double poseX, poseY, poseZ, poseRoll, posePitch, poseYaw;

    const std::vector<unsigned char> depthData;
 
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void pointCloundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);


};


void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Camera position in map frame
    odomX = msg->pose.pose.position.x;
    odomY = msg->pose.pose.position.y;
    odomZ = msg->pose.pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix 
    m.getRPY(odomRoll, odomPitch, odomYaw);

    // Output the measure
    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             odomX, odomY, odomZ,
             odomRoll * RAD2DEG, odomPitch * RAD2DEG, odomYaw * RAD2DEG);
}


void Robot::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // Camera position in map frame
    double poseX = msg->pose.position.x;
    double poseY = msg->pose.position.y;
    double poseZ = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    m.getRPY(poseRoll, posePitch, poseYaw);

    // Output the measure
    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             poseX, poseY, poseZ,
             poseRoll * RAD2DEG, posePitch * RAD2DEG, poseYaw * RAD2DEG);
}


void Robot::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;

    // Output the measure
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
}



void Robot::imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
//    ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);

    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

//        cv::resize(cv_ptr->image, imageDisplay, displaySize);
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
        }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

}



void Robot::imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}



void Robot::pointCloundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Point Cloud received from ZED - Size: %dx%d   Is dence: %d", msg->width, msg->height, msg->is_dense);


    
}


/**
 * Node main function
 */
int main(int argc, char** argv) {

    Robot robot;

    ros::init(argc, argv, "zed_processing");

    ros::NodeHandle n;

    

    ros::Subscriber subOdom  = n.subscribe("/zed/zed_node/odom", 10, &Robot::odomCallback, &robot);
    ros::Subscriber subPose  = n.subscribe("/zed/zed_node/pose", 10, &Robot::poseCallback, &robot);
    ros::Subscriber subDepth = n.subscribe("/zed/zed_node/depth/depth_registered", 10, &Robot::depthCallback, &robot);


    ros::Subscriber subRightRectified = n.subscribe("/zed/zed_node/right/image_rect_color", 10,
                                        &Robot::imageRightRectifiedCallback, &robot);
    ros::Subscriber subLeftRectified  = n.subscribe("/zed/zed_node/left/image_rect_color", 10,
                                        &Robot::imageLeftRectifiedCallback, &robot);
    ros::Subscriber pointCloud = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10,
                                        &Robot::pointCloundCallback, &robot);


/*
  	message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/zed/zed_node/right/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(n, "/zed/zed_node/point_cloud/cloud_registered", 1);

    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, depth_sub, 10);
    sync.registerCallback(boost::bind(&Robot::pointCloundCallback,&robot, _1, _2));

*/

    ros::spin();

    cv::namedWindow(OPENCV_WINDOW);

    return 0;
}
