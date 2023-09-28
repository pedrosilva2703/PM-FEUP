
#include "detector_node.h"

void calculateCentroid(auto pc_in, PointXYZ &centroid)
{

  // Calculate centroid
  int number_of_points = pc_in->size();
  if (number_of_points == 0)
  {
    return;
  }
  float x = 0.0f, y = 0.0f, z = 0.0f;
  for (int i = 0; i < number_of_points; i++)
  {
    x += pc_in->points[i].x;
    y += pc_in->points[i].y;
    z += pc_in->points[i].z;
  }
  centroid.x = x / number_of_points;
  centroid.y = y / number_of_points;
  centroid.z = z / number_of_points;

  return;
}

float euclDist(PointXYZ pt1, PointXYZ pt2)
{
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
}

void filter_near_sensor_origin(PointCloudXYZI::Ptr pc_in,
                               PointCloudXYZI::Ptr pc_out)
{

  // Remove outliers near (0,0,0)
  pcl::ConditionOr<PointXYZI>::Ptr range_cond(new pcl::ConditionOr<PointXYZI>());

  range_cond->addComparison(pcl::FieldComparison<PointXYZI>::ConstPtr(new pcl::FieldComparison<PointXYZI>("x", pcl::ComparisonOps::GT, 0.5)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZI>::ConstPtr(new pcl::FieldComparison<PointXYZI>("x", pcl::ComparisonOps::LT, -0.5)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZI>::ConstPtr(new pcl::FieldComparison<PointXYZI>("y", pcl::ComparisonOps::GT, 0.5)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZI>::ConstPtr(new pcl::FieldComparison<PointXYZI>("y", pcl::ComparisonOps::LT, -0.5)));

  // build the filter
  pcl::ConditionalRemoval<PointXYZI> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(pc_in);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter(*pc_out);
}

void filter_near_sensor_origin_xyz(PointCloudXYZ::Ptr pc_in,
                                   PointCloudXYZ::Ptr pc_out)
{

  // Remove outliers near (0,0,0)
  pcl::ConditionOr<PointXYZ>::Ptr range_cond(new pcl::ConditionOr<PointXYZ>());

  range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(new pcl::FieldComparison<PointXYZ>("x", pcl::ComparisonOps::GT, 0.5)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(new pcl::FieldComparison<PointXYZ>("x", pcl::ComparisonOps::LT, -0.5)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(new pcl::FieldComparison<PointXYZ>("y", pcl::ComparisonOps::GT, 0.5)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(new pcl::FieldComparison<PointXYZ>("y", pcl::ComparisonOps::LT, -0.5)));

  // build the filter
  pcl::ConditionalRemoval<PointXYZ> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(pc_in);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter(*pc_out);
}

void filter_low_intensity(PointCloudXYZI::Ptr pc_in,
                          PointCloudXYZ::Ptr pc_out)
{

  pcl::ConditionAnd<PointXYZI>::Ptr int_cond(new pcl::ConditionAnd<PointXYZI>());

  int_cond->addComparison(pcl::FieldComparison<PointXYZI>::ConstPtr(new pcl::FieldComparison<PointXYZI>("intensity", pcl::ComparisonOps::GT, 200)));

  // build the filter
  pcl::ConditionalRemoval<PointXYZI> condrem;
  condrem.setCondition(int_cond);
  condrem.setInputCloud(pc_in);
  condrem.setKeepOrganized(true);
  // apply filter
  PointCloudXYZI::Ptr pc_out_xyzi(new PointCloudXYZI);
  condrem.filter(*pc_out_xyzi);

  pc_out->points.resize(pc_out_xyzi->size());
  for (size_t i = 0; i < pc_out_xyzi->size(); i++)
  {
    pc_out->points[i].x = pc_out_xyzi->points[i].x;
    pc_out->points[i].y = pc_out_xyzi->points[i].y;
    pc_out->points[i].z = pc_out_xyzi->points[i].z;
  }
}

void filter_radius(PointCloudXYZ::Ptr pc_in,
                   PointCloudXYZ::Ptr pc_out,
                   PointXYZ origin,
                   float R)
{
  // Using octree for faster point access
  float resolution = 256.0f;
  pcl::octree::OctreePointCloudSearch<PointXYZ> octree(resolution);
  octree.setInputCloud(pc_in);
  octree.addPointsFromInputCloud();

  // Filter points within a radius from origin
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if (octree.radiusSearch(origin, R, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      pc_out->push_back(pc_in->points[pointIdxRadiusSearch[i]]);
    }
  }
}

void clusterExtraction(PointCloudXYZ::Ptr pc_in,
                       std::vector<pcl::PointIndices> &cluster_out_indices)
{

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_in, *indices);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
  tree->setInputCloud(pc_in);
  pcl::EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(0.3); // 30cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(20000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc_in);
  ec.setIndices(indices);
  ec.extract(cluster_out_indices);
}

void minCutSegmentation(PointCloudXYZ::Ptr pc_in,
                        PointXYZ origin,
                        PointCloudXYZI::Ptr pc_out)
{

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_in, *indices);

  pcl::MinCutSegmentation<PointXYZ> seg;
  seg.setInputCloud(pc_in);
  seg.setIndices(indices);

  PointCloudXYZ::Ptr foreground_points(new PointCloudXYZ());

  foreground_points->points.push_back(origin);
  seg.setForegroundPoints(foreground_points);

  seg.setSigma(0.5);
  seg.setRadius(2.0);
  seg.setNumberOfNeighbours(20);
  seg.setSourceWeight(0.7);

  std::vector<pcl::PointIndices> clusters;
  seg.extract(clusters);

  for (const auto &cluster : clusters)
  {
    PointXYZI point;
    point.intensity = (rand() % 100 + 1) * 500;
    for (const auto &idx : cluster.indices)
    {
      point.x = (*pc_in)[idx].x;
      point.y = (*pc_in)[idx].y;
      point.z = (*pc_in)[idx].z;
      pc_out->push_back(point);
    }
  }

  // std::cout << "(MinCutSegm) PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
}

void createObject_pc(PointCloudXYZ::Ptr pc_in,
                     std::vector<pcl::PointIndices> &indices_vec,
                     PointCloudXYZ::Ptr pc_out)
{

  // See the two larger clusters
  int max_idx = 0, max_idx2 = 0;
  int max_points = INT_MIN, max_points2 = INT_MIN;
  for (int j = 0; j < indices_vec.size(); j++)
  {
    int size = indices_vec[j].indices.size();
    if (size >= max_points)
    {
      max_points = size;
      max_idx = j;
    }
    else if (size >= max_points2)
    {
      max_points2 = size;
      max_idx2 = j;
    }
  }

  // In case of there is PC (do not exist the bigger one)
  if (indices_vec[max_idx].indices.size() == 0)
    return;

  // Determine the centoid of the biggest PC
  // std::cout << "1: " << max_points << " at: " << max_idx << " 2: " << max_points2 << " at: " << max_idx2 << std::endl;

  PointXYZ point;
  for (const auto &idx : indices_vec[max_idx].indices)
  {
    point.x = (*pc_in)[idx].x;
    point.y = (*pc_in)[idx].y;
    point.z = (*pc_in)[idx].z;
    pc_out->push_back(point);
  }

  PointXYZ cent;
  calculateCentroid(pc_out, cent);
  // std::cout << "centroid 1: " << cent << std::endl;

  // Determine the centroid of the 2nd biggest PC
  PointCloudXYZ::Ptr cluster2(new PointCloudXYZ);
  for (const auto &idx : indices_vec[max_idx2].indices)
  {
    point.x = (*pc_in)[idx].x;
    point.y = (*pc_in)[idx].y;
    point.z = (*pc_in)[idx].z;
    cluster2->push_back(point);
  }
  PointXYZ cent2;
  calculateCentroid(cluster2, cent2);

  // std::cout << "centroid 2: " << cent << std::endl;
  // std::cout << "Centroids Distance: " << euclDist(cent, cent2) << std::endl;

  // If the centroids are closer, it is the same object
  if (euclDist(cent, cent2) < 10.0f)
  {
    for (const auto &idx : indices_vec[max_idx2].indices)
    {
      point.x = (*pc_in)[idx].x;
      point.y = (*pc_in)[idx].y;
      point.z = (*pc_in)[idx].z;
      pc_out->push_back(point);
    }
  }
}

void closerCluster_pc(PointCloudXYZ::Ptr pc_in,
                      std::vector<pcl::PointIndices> &indices_vec,
                      PointCloudXYZ::Ptr pc_out)
{
  // Get each cluster
  for (const auto &cluster : indices_vec)
  {

    // Create PC with that cluster
    PointCloudXYZ::Ptr cloud_cluster(new PointCloudXYZ);
    for (const auto &idx : cluster.indices)
    {
      cloud_cluster->push_back((*pc_in)[idx]);
    }

    // Compute centroid of the cluster
    PointXYZ cent;
    calculateCentroid(cloud_cluster, cent);

    // Calculate distance to that centroid
    PointXYZ origin;
    origin.x = 0.0f;
    origin.y = 0.0f;
    origin.z = 0.0f;
    float dist = euclDist(cent, origin);
    // Build pc_out
    if (dist < 15.0f)
    {
      for (const auto &idx : cluster.indices)
      {
        pc_out->push_back((*pc_in)[idx]);
      }
    }
  }
}

void publishPC(auto pc_in)
{
  // *********************************** CONVERT PCL PC TO ROSMSG ******************************************** //
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(*pc_in, cloud_out);

  std_msgs::Header header;
  header.frame_id = "os_sensor";
  header.stamp = ros::Time::now();
  cloud_out.header = header;
  pub_obs_pc.publish(cloud_out);
}

void publishBbox(pcl::PointXYZ pmin, pcl::PointXYZ pmax)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "os_sensor";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 1;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.1;

  // Line list is green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  // Zmin points
  p.x = pmin.x;
  p.y = pmin.y;
  p.z = pmin.z;
  line_list.points.push_back(p);
  p.x = pmax.x;
  p.y = pmin.y;
  p.z = pmin.z;
  line_list.points.push_back(p);

  line_list.points.push_back(p);
  p.x = pmax.x;
  p.y = pmax.y;
  p.z = pmin.z;
  line_list.points.push_back(p);

  line_list.points.push_back(p);
  p.x = pmin.x;
  p.y = pmax.y;
  p.z = pmin.z;
  line_list.points.push_back(p);

  line_list.points.push_back(p);
  p.x = pmin.x;
  p.y = pmin.y;
  p.z = pmin.z;
  line_list.points.push_back(p);

  // Zmax points
  p.x = pmin.x;
  p.y = pmin.y;
  p.z = pmax.z;
  line_list.points.push_back(p);
  p.x = pmax.x;
  p.y = pmin.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  line_list.points.push_back(p);
  p.x = pmax.x;
  p.y = pmax.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  line_list.points.push_back(p);
  p.x = pmin.x;
  p.y = pmax.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  line_list.points.push_back(p);
  p.x = pmin.x;
  p.y = pmin.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  // Vertical lines
  p.x = pmin.x;
  p.y = pmin.y;
  p.z = pmin.z;
  line_list.points.push_back(p);
  p.x = pmin.x;
  p.y = pmin.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  p.x = pmin.x;
  p.y = pmax.y;
  p.z = pmin.z;
  line_list.points.push_back(p);
  p.x = pmin.x;
  p.y = pmax.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  p.x = pmax.x;
  p.y = pmax.y;
  p.z = pmin.z;
  line_list.points.push_back(p);
  p.x = pmax.x;
  p.y = pmax.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  p.x = pmax.x;
  p.y = pmin.y;
  p.z = pmin.z;
  line_list.points.push_back(p);
  p.x = pmax.x;
  p.y = pmin.y;
  p.z = pmax.z;
  line_list.points.push_back(p);

  // Publish markers
  pub_bbox.publish(line_list);
}

void publishVelocityMarker(geometry_msgs::Point c)
{
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = "os_sensor";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "spheres";
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;
  sphere_list.id = 0;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  sphere_list.scale.x = 0.2;
  sphere_list.scale.y = 0.2;
  sphere_list.scale.z = 0.5;

  // Points are red
  sphere_list.color.r = 1.0f;
  sphere_list.color.g = 0.0f;
  sphere_list.color.b = 0.0f;
  sphere_list.color.a = 1.0;

  sphere_list.points.push_back(c);
  pub_centroid.publish(sphere_list);
}

void method_rad_stats(PointCloudXYZ::Ptr pc_in)
{

  // ******************************** USING OCTREE TO FASTER ACCESS, RADIUS FILTER ***************************************** //
  // Radius filter, return points inside a sphere of radius R from origin point
  PointCloudXYZ::Ptr c_out_radius_filtered(new PointCloudXYZ);
  PointXYZ originPoint;
  originPoint.x = 0.0f;
  originPoint.y = 0.0f;
  originPoint.z = 0.0f;

  float radius = 15.0f;
  filter_radius(pc_in, c_out_radius_filtered, originPoint, radius);

  // *********************************** FILTER OUTLIERS ******************************************** //
  PointCloudXYZ::Ptr c_out_outliers_filtered(new PointCloudXYZ);
  pcl::StatisticalOutlierRemoval<PointXYZ> sor;
  sor.setInputCloud(c_out_radius_filtered);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*c_out_outliers_filtered);

  // *********************************** PUBLISH PC ******************************************** //
  publishPC(c_out_outliers_filtered);

  // *********************************** CALCULATE & PUBLISH CENTROID ******************************************** //
  geometry_msgs::Point centroid;
  PointXYZ conversion_cent;

  calculateCentroid(c_out_outliers_filtered, conversion_cent);
  centroid.x = conversion_cent.x;
  centroid.y = conversion_cent.y;
  centroid.z = conversion_cent.z;
  //std::cout << centroid.x << " " << centroid.y << std::endl;
  publishVelocityMarker(centroid);

  // *********************************** PUBLISH BOUNDING BOX MARKER ******************************************** //
  PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*c_out_outliers_filtered, minPt, maxPt);
  publishBbox(minPt, maxPt);
}

void method_eucl_clusters(PointCloudXYZ::Ptr pc_in)
{

  /*  ****************************** MinCut Segmentation Method(not the best) ****************************** */
  // MinCutSegmentation
  // Need a initial point to makes the cuts around that point
  // The best option it to calculate the centroid of the c_out_cube_filtered
  /*
  PointXYZ initPoint;
  PointCloudXYZI::Ptr c_out_minCutSeg(new PointCloudXYZI);
  minCutSegmentation(c_out_cube_filtered, initPoint, c_out_minCutSeg);
  */

  /*  ****************************** Cluster Extraction Method ****************************** */
  // Cluster Extraction for detection of objects -> Segmentation
  std::vector<pcl::PointIndices> cluster_indices;
  clusterExtraction(pc_in, cluster_indices);

  // Extract from the cluster the object
  // In this case, the boat
  PointCloudXYZ::Ptr c_out_object(new PointCloudXYZ);
  createObject_pc(pc_in, cluster_indices, c_out_object);

  // *********************************** PUBLISH PC ******************************************** //
  publishPC(c_out_object);

  // *********************************** CALCULATE & PUBLISH CENTROID ******************************************** //
  geometry_msgs::Point centroid;
  PointXYZ conversion_cent;

  calculateCentroid(c_out_object, conversion_cent);
  centroid.x = conversion_cent.x;
  centroid.y = conversion_cent.y;
  centroid.z = conversion_cent.z;
  //std::cout << centroid.x << " " << centroid.y << std::endl;
  publishVelocityMarker(centroid);

  // *********************************** PUBLISH BOUNDING BOX MARKER ******************************************** //
  PointXYZ minPt, maxPt;
  PointCloudXYZ conv;
  pcl::getMinMax3D(*c_out_object, minPt, maxPt);
  publishBbox(minPt, maxPt);
}

void method_close_clusters(PointCloudXYZ::Ptr pc_in)
{

  /*  ****************************** Cluster Extraction Method ****************************** */
  // Cluster Extraction for detection of objects -> Segmentation
  std::vector<pcl::PointIndices> cluster_indices;
  clusterExtraction(pc_in, cluster_indices);

  // Extract from the cluster the object
  // In this case, the boat
  PointCloudXYZ::Ptr c_out_object(new PointCloudXYZ);
  closerCluster_pc(pc_in, cluster_indices, c_out_object);

  // *********************************** PUBLISH PC ******************************************** //
  publishPC(c_out_object);

  // *********************************** CALCULATE & PUBLISH CENTROID ******************************************** //
  geometry_msgs::Point centroid;
  PointXYZ conversion_cent;

  calculateCentroid(c_out_object, conversion_cent);
  centroid.x = conversion_cent.x;
  centroid.y = conversion_cent.y;
  centroid.z = conversion_cent.z;
  std::cout << centroid.x << " " << centroid.y << std::endl;
  publishVelocityMarker(centroid);

  // *********************************** PUBLISH BOUNDING BOX MARKER ******************************************** //
  PointXYZ minPt, maxPt;
  PointCloudXYZ conv;
  pcl::getMinMax3D(*c_out_object, minPt, maxPt);
  publishBbox(minPt, maxPt);
}

void cbPC2(const sensor_msgs::PointCloud2ConstPtr &input)
{

  /**
   * Was implemented 2 methods (both using a filtered PC with the closest point removed (antena base points)):
   *    -(1) Radius filter and statistical outliers removal in order to publish the cloud point with the boat only
   *    -(2) Euclidean Cluster Extraction and build the object with the 2 biggest clusters
   *    -(3) Build the object using the closest cluster (distancing 15m from the LIDAR)
   */
  int method = 1;

  PointCloudXYZI::Ptr cloud_in_1(new PointCloudXYZI);
  PointCloudXYZI::Ptr c_out_cube_filtered_1(new PointCloudXYZI);
  PointCloudXYZ::Ptr c_out_lowI_filtered_1(new PointCloudXYZ);

  PointCloudXYZ::Ptr cloud_in_2(new PointCloudXYZ);
  PointCloudXYZ::Ptr c_out_cube_filtered_2(new PointCloudXYZ);

  PointCloudXYZ::Ptr cloud_in_3(new PointCloudXYZ);
  PointCloudXYZ::Ptr c_out_cube_filtered_3(new PointCloudXYZ);
  switch (method)
  {
  case 1:

    pcl::fromROSMsg(*input, *cloud_in_1);

    filter_near_sensor_origin(cloud_in_1, c_out_cube_filtered_1);

    filter_low_intensity(c_out_cube_filtered_1, c_out_lowI_filtered_1);

    method_rad_stats(c_out_lowI_filtered_1);
    break;
  case 2:

    pcl::fromROSMsg(*input, *cloud_in_2);

    filter_near_sensor_origin_xyz(cloud_in_2, c_out_cube_filtered_2);
    method_eucl_clusters(c_out_cube_filtered_2);
    break;
  case 3:

    pcl::fromROSMsg(*input, *cloud_in_3);

    filter_near_sensor_origin_xyz(cloud_in_3, c_out_cube_filtered_3);
    method_close_clusters(c_out_cube_filtered_3);
    break;
  default:
    break;
  }

  PointCloudXYZI::Ptr cloud_in(new PointCloudXYZI);
  pcl::fromROSMsg(*input, *cloud_in);
  // Point Cloud stabilization with IMU attempt
  double q0 = quat_msg.x;
  double q1 = quat_msg.y;
  double q2 = quat_msg.z;
  double q3 = quat_msg.w;

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  // First row of the rotation matrix
  transform_1(0, 0) = 2 * (q0 * q0 + q1 * q1) - 1;
  transform_1(0, 1) = 2 * (q1 * q2 - q0 * q3);
  transform_1(0, 2) = 2 * (q1 * q3 + q0 * q2);

  // Second row of the rotation matrix
  transform_1(1, 0) = 2 * (q1 * q2 + q0 * q3);
  transform_1(1, 1) = 2 * (q0 * q0 + q2 * q2) - 1;
  transform_1(1, 2) = 2 * (q2 * q3 - q0 * q1);

  // Third row of the rotation matrix
  transform_1(2, 0) = 2 * (q1 * q3 - q0 * q2);
  transform_1(2, 1) = 2 * (q2 * q3 + q0 * q1);
  transform_1(2, 2) = 2 * (q0 * q0 + q3 * q3) - 1;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform_1);

  sensor_msgs::PointCloud2 transformed_cloud_out;
  pcl::toROSMsg(*transformed_cloud, transformed_cloud_out);
  std_msgs::Header header;
  header.frame_id = "imu_nav_box";
  header.stamp = ros::Time::now();
  transformed_cloud_out.header = header;

  pub_stab_pc.publish(transformed_cloud_out);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg_in)
{

  quat_msg = msg_in->orientation;
}

int main(int argc, char **argv)
{
  // Init the ros system
  ros::init(argc, argv, "detector_node");

  // Create the node handles to establish the program as a ROS node
  ros::NodeHandle n_public;

  ros::Subscriber sub_imu = n_public.subscribe<sensor_msgs::Imu>("/imu_nav/data", 1, imu_cb);
  ros::Subscriber sub_pc = n_public.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, cbPC2);

  pub_stab_pc = n_public.advertise<sensor_msgs::PointCloud2>("/detector/stab_pc", 1);
  pub_obs_pc = n_public.advertise<sensor_msgs::PointCloud2>("/detector/obstacle", 1);
  pub_centroid = n_public.advertise<visualization_msgs::Marker>("/detector/centroid", 100);
  pub_bbox = n_public.advertise<visualization_msgs::Marker>("/detector/bbox", 1);

  ros::spin();
  return 0;
}
