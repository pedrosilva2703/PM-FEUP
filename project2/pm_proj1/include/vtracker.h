// C++ includes
#include <iostream>
#include <string>
#include <sstream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/// Ros includes
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/console.h>

/// ROS MSGs includes
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>

ros::Publisher pub_ball_center_tracker;
geometry_msgs::Point ball_center_rcv;

cv::Mat frame_rcv, output;
bool new_frame = false;
bool new_center = false;

// Para calcular o dT no kf
int fps = 30;

// Kalman Filter
int stateSize = 4;
int measSize = 2;
int controlSize = 0;  // No control

unsigned int type = CV_32F;
cv::KalmanFilter kf(stateSize, measSize, controlSize, type);

cv::Mat state(stateSize, 1, type); // [x,y,v_x,v_y]
cv::Mat meas(measSize, 1, type);   // [z_x,z_y]

// For saving the position estimation(KF) and the received 
std::vector<geometry_msgs::Point> est_recv;
std::vector<geometry_msgs::Point> est_kf;



