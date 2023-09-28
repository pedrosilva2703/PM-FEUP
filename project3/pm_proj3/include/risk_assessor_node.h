
#ifndef RISKASSESSORNODE_H
#define RISKASSESSORNODE_H

#include <iostream>
#include <vector>
#include <ctime>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

#include <std_msgs/Time.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/octree/octree_search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/common/common.h>

// PCL Registration
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <pcl/console/time.h> // TicToc

typedef pcl::Normal Normal;                                    //!< Data type definition for a Normal
typedef pcl::PointXYZ PointXYZ;                                //!< Data type definition for a PointXYZ
typedef pcl::PointXYZI PointXYZI;                              //!< Data type definition for a PointXYZI
typedef pcl::PointNormal PointXYZNormal;                       //!< Data type definition for a PointXYZNormal
typedef pcl::PointXYZINormal PointXYZINormal;                  //!< Data type definition for a PointXYZINormal
typedef pcl::PointCloud<Normal> PointCloudNormal;              //!< Data type definition for a PointCloud with Normal
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;               //!< Data type definition for a PointCloud with PointXYZ
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;             //!< Data type definition for a PointCloud with PointXYZI
typedef pcl::PointCloud<PointXYZNormal> PointCloudXYZNormal;   //!< Data type definition for a PointCloud with PointXYZNormal
typedef pcl::PointCloud<PointXYZINormal> PointCloudXYZINormal; //!< Data type definition for a PointCloud with PointXYZINormal

/// ROS include

/// Global vars
bool first_run = true;
nav_msgs::Odometry old_odom;

geometry_msgs::Vector3 velocity;

ros::Publisher pub_ttc;
ros::Publisher pub_ROI_pc;
ros::Publisher vis_pub;

ros::Publisher pub_ttc_time_msg;

double time_counter;
#endif
