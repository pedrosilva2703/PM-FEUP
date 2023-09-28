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

int bnd_width, bnd_height = INT_MIN;

int state = 0;

std::string video_path;

std::vector<int> lower;
std::vector<int> upper;

std::vector<double> ball_area;

void readParams();

