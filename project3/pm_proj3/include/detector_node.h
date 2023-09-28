
#ifndef DETECTORNODE_H
#define DETECTORNODE_H

#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>


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

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

// PCL Registration
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <pcl/common/transforms.h>
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
ros::Publisher pub_obs_pc;
ros::Publisher pub_stab_pc;
ros::Publisher pub_centroid;
ros::Publisher pub_bbox;
geometry_msgs::Quaternion quat_msg;

/// Function Headers

void calculateCentroid(auto pc_in, pcl::PointXYZ &centroid);

float euclDist(pcl::PointXYZ pt1, pcl::PointXYZ pt2);

void filter_near_sensor_origin(PointCloudXYZI::Ptr pc_in,
                               PointCloudXYZI::Ptr pc_out);

void filter_near_sensor_origin_xyz(PointCloudXYZ::Ptr pc_in,
                               PointCloudXYZ::Ptr pc_out);

void filter_voxellGrid(PointCloudXYZ::Ptr pc_in,
                       PointCloudXYZ::Ptr pc_out);

void filter_radius(PointCloudXYZ::Ptr pc_in,
                   PointCloudXYZ::Ptr pc_out,
                   pcl::PointXYZ origin,
                   float R);

void clusterExtraction(PointCloudXYZ::Ptr pc_in,
                       std::vector<pcl::PointIndices> &cluster_out_indices);

void minCutSegmentation(PointCloudXYZ::Ptr pc_in,
                        pcl::PointXYZ origin,
                        PointCloudXYZI::Ptr pc_out);

void createObject_pc(PointCloudXYZ::Ptr pc_in,
                     std::vector<pcl::PointIndices> &indices_vec,
                     PointCloudXYZI::Ptr pc_out);

void closerCluster_pc(PointCloudXYZ::Ptr pc_in,
                      std::vector<pcl::PointIndices> &indices_vec,
                      PointCloudXYZ::Ptr pc_out);

void publishPC(auto pc_in);

void publishBbox(pcl::PointXYZ pmin, pcl::PointXYZ pmax);

void publishVelocityMarker(geometry_msgs::Point c);

void method_rad_stats(PointCloudXYZ::Ptr pc_in);

void method_eucl_clusters(PointCloudXYZ::Ptr pc_in);

#endif
