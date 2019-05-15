
// roslaunch zed_wrapper zed.launch

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <string>

// Image Processing Libraries ---------------
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Message libraries ----------------------
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// mathy stuff libraries ---------------
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



using namespace std;
using namespace message_filters;

static const std::string OPENCV_WINDOW = "Image window";


/*----------------------------------------------------------------------------------------------
 * Subscriber callbacks
 */


void getXYZ(sensor_msgs::PointCloud2 my_pcl, int x, int y, geometry_msgs::Point& p)
{
    int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8

    float X ;
    float Y ;
    float Z ;

    memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
    memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
    memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

    // geometry_msgs::Point p;
    // put data into the point p
    p.x = X;
    p.y = Y;
    p.z = Z;
}


//==== ROBOT ===================================================================================
class Robot {
public:
    double odomX, odomY, odomZ, odomRoll, odomPitch, odomYaw;
    double poseX, poseY, poseZ, poseRoll, posePitch, poseYaw;

    const std::vector<unsigned char> depthData;
    sensor_msgs::PointCloud2 my_pcl;
 
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
/*
    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             odomX, odomY, odomZ,
             RAD2DEG(odomRoll), RAD2DEG(odomPitch), RAD2DEG(odomYaw) );
*/
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
/*
    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             poseX, poseY, poseZ,
             RAD2DEG(poseRoll), RAD2DEG(posePitch), RAD2DEG(poseYaw) );
*/
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
//    ROS_INFO("Center distance : %g m", depths[centerIdx]);
}



void Robot::imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);

    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
        }

    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

}



void Robot::imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
//    ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}



void Robot::pointCloundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
//    ROS_INFO("Point Cloud received from ZED - Size: %dx%d   Is dence: %d", msg->width, msg->height, msg->is_dense);

   geometry_msgs::Point p; 

   getXYZ(*msg, 600, 350, p);


   cout<<"Pt.x: "<<p.x<<endl;
   cout<<"Pt.y: "<<p.y<<endl;
   cout<<"Pt.z: "<<p.z<<endl;

}



/*--------------------------------------------------------------------------------------------
 * Node main function
 */

int main(int argc, char** argv) {

    Robot robot;

    ros::init(argc, argv, "demo_zed_node");

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

    cv::namedWindow(OPENCV_WINDOW);

    ros::spin();

    return 0;
}
