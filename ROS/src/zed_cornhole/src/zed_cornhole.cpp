
// roslaunch zed_wrapper zed.launch

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <string>

// Image Processing Libraries ---------------
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
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

int LowerH = 170;
int LowerS = 150;
int LowerV = 60;
int UpperH = 179;
int UpperS = 255;
int UpperV = 255;
 


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


//==== ROBOT CLASS ===================================================================================
class Robot {

private:
	ros::Publisher pub;
	ros::NodeHandle n;
	
public:

	int posX = 0;
    int posY = 0;
    int numTargets = 0;
    
    double odomX, odomY, odomZ, odomRoll, odomPitch, odomYaw;
    double poseX, poseY, poseZ, poseRoll, posePitch, poseYaw;

    const std::vector<unsigned char> depthData;

    geometry_msgs::Point target; 
 
/* 
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void pointCloundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
*/

	Robot(){
		pub = n.advertise<geometry_msgs::Point>("target",5);
	}

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	    odomX = msg->pose.pose.position.x; 			// Camera position in map frame
	    odomY = msg->pose.pose.position.y;
	    odomZ = msg->pose.pose.position.z;
	
	    tf2::Quaternion q( 					// Orientation quaternion
	        msg->pose.pose.orientation.x,
	        msg->pose.pose.orientation.y,
	        msg->pose.pose.orientation.z,
	        msg->pose.pose.orientation.w);
	
	    tf2::Matrix3x3 m(q); 				// 3x3 Rotation matrix from quaternion
	 
	    m.getRPY(odomRoll, odomPitch, odomYaw); 		// Roll Pitch and Yaw from rotation matrix
	
	 /*
	    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
	             msg->header.frame_id.c_str(),
	             odomX, odomY, odomZ,
	             RAD2DEG(odomRoll), RAD2DEG(odomPitch), RAD2DEG(odomYaw) );
	 */
	}
	
	
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	
	    double poseX = msg->pose.position.x; 		// Camera position in map frame
	    double poseY = msg->pose.position.y;
	    double poseZ = msg->pose.position.z;
	
	    tf2::Quaternion q( 					// Orientation quaternion
	        msg->pose.orientation.x,
	        msg->pose.orientation.y,
	        msg->pose.orientation.z,
	        msg->pose.orientation.w);
	
	    tf2::Matrix3x3 m(q); 				// 3x3 Rotation matrix from quaternion
	
	    m.getRPY(poseRoll, posePitch, poseYaw); 		// Roll Pitch and Yaw from rotation matrix
	
	/*
	    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
	             msg->header.frame_id.c_str(),
	             poseX, poseY, poseZ,
	             RAD2DEG(poseRoll), RAD2DEG(posePitch), RAD2DEG(poseYaw) );
	*/
	}
	
	
	void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
	
	    float* depths = (float*)(&msg->data[0]);   	// Get a pointer to the depth values casting the data
	    
	    int u = msg->width / 2; 			// Image coordinates of the center pixel
	    int v = msg->height / 2;
	
	    int centerIdx = u + msg->width * v;  	// Linear index of the center pixel
	
	    cout<<"Center distance: "<< depths[centerIdx] <<"[m]" << endl;
	}
	
	
	void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
	//    ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
	}
	
	
	void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
	//	ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
	
	    cv_bridge::CvImagePtr cv_ptr;
	
	    try {
	        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	        // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	        // cv::waitKey(3);
	
	        }
	
		catch(cv_bridge::Exception& e){
	        ROS_ERROR("cv_bridge exception: %s", e.what());
	        return;
	        }
	
	 
		/// Detect Circlular blobs ----------------------------------------------------------------------------
		cv::Mat im = cv_ptr->image;
	   
		// Setup SimpleBlobDetector parameters.
		cv::SimpleBlobDetector::Params params;
	 
		// Change thresholds
		params.minThreshold = 100; //100
		params.maxThreshold = 250; //200
	 
		// Filter by Area.
		params.filterByArea = true;
		params.minArea = 110; //200
		params.maxArea = 320; //350
	 
		// Filter by Circularity
		params.filterByCircularity = true;
		params.minCircularity = 0.55; //.55
		params.maxCircularity = 0.85; //.75
	 
		// Filter by Convexity
		params.filterByConvexity = true;
		params.minConvexity = 0.87; //.87
	 
		// Filter by Inertia
		params.filterByInertia = true;
		params.minInertiaRatio = 0.1; //.1
		params.maxInertiaRatio = 0.4; //.4
	
		// Set up the detector with default parameters.
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	
		// Detect blobs.
		std::vector<cv::KeyPoint> keypoints;
		detector->detect( im, keypoints);
	   
		numTargets = keypoints.size();
	   
		if(keypoints.size() > 0) {
			posX = keypoints[0].pt.x;
			posY = keypoints[0].pt.y;
			}
		else {
			poseX = 0;
			poseY = 0;
			}
	   
	    // Label the tarvet with cartsian coordiantes
		char txt[50];
		sprintf(txt, "X: %f",target.x);	
		cv::putText( im, txt, cv::Point(60,60), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255), 1);
		
		sprintf(txt, "Y: %f",target.y);	
		cv::putText( im, txt, cv::Point(60,120), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255), 1);
		
		sprintf(txt, "Z: %f",target.z);	
		cv::putText( im, txt, cv::Point(60,180), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255), 1);
	
		// Draw detected blobs as red circles.
		cv::drawKeypoints( im, keypoints, im, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		cv::circle(  im, cv::Point(posX, posY), 2, cv::Scalar(0,0,255), 2);
		
	 
		// Show blobs
		cv::imshow("keypoints", im );
		cv::waitKey(1);
	}
	
	
	
	void pointCloundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//    ROS_INFO("Point Cloud received from ZED - Size: %dx%d   Is dence: %d", msg->width, msg->height, msg->is_dense);
	
	//	cout<<endl<<"posx:"<<posX<<"  posy:"<<posY<<endl;  
	
		if( posX != 0 && posY != 0) {
			
			getXYZ(*msg, posX, posY, target);
			pub.publish(target);
	
	//		cout <<"posx:"<<posX<<"  posy:" << posY << "   pt.X: " << target.x << "   Y: " << target.y << "   Z: " << target.z << endl;
	   }
	}

};

/*--------------------------------------------------------------------------------------------
 * Node main function
 */

int main(int argc, char** argv) {

    ros::init(argc, argv, "zed_cornhole");

	Robot robot;
	   
    ros::NodeHandle n;

 
    ros::Subscriber subRightRectified = n.subscribe("/zed/zed_node/right/image_rect_color", 10,
                                        &Robot::imageRightRectifiedCallback, &robot);
    ros::Subscriber pointCloud = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10,
                                        &Robot::pointCloundCallback, &robot);

 /*
    ros::Subscriber subOdom  = n.subscribe("/zed/zed_node/odom", 10, &Robot::odomCallback, &robot);
    ros::Subscriber subPose  = n.subscribe("/zed/zed_node/pose", 10, &Robot::poseCallback, &robot);
    ros::Subscriber subDepth = n.subscribe("/zed/zed_node/depth/depth_registered", 10, &Robot::depthCallback, &robot);
    ros::Subscriber subLeftRectified  = n.subscribe("/zed/zed_node/left/image_rect_color", 10,
                                        &Robot::imageLeftRectifiedCallback, &robot);
 */

   // cv::namedWindow(OPENCV_WINDOW);

    ros::spin();

    return 0;
}
