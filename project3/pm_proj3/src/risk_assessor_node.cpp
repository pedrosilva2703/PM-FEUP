
#include "risk_assessor_node.h"

float euclDist(PointXYZ pt1, PointXYZ pt2)
{
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
}

int getQuadrante(PointXYZ pt)
{
    if (pt.x >= 0.0 && pt.y >= 0.0)
        return 1;

    if (pt.x <= 0.0 && pt.y >= 0.0)
        return 2;

    if (pt.x <= 0.0 && pt.y <= 0.0)
        return 3;

    if (pt.x >= 0.0 && pt.y <= 0.0)
        return 4;

    return -1;
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
    pub_ROI_pc.publish(cloud_out);
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
    vis_pub.publish(sphere_list);
}

void publishTTC(float ttc)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "os_sensor";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 2.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    if (ttc > 0.001 && ttc < 1000)
    {
        std::string s1 = "TTC = ";
        std::string s2 = " s";

        std::ostringstream oss;
        oss << std::setprecision(2) << ttc;

        marker.text = s1 + oss.str() + s2;
        
        // Separate in seconds and nanoseconds and publish
        std_msgs::Time time_ttc;
        int sec = (int)ttc;
        double decpart = ttc - sec;
        int nsec = decpart * 1000000000;
        time_ttc.data.sec = sec;
        time_ttc.data.nsec = nsec;
        pub_ttc_time_msg.publish(time_ttc);
    }
    else
    {
        marker.text = " ";
    }

    marker.scale.x = (double)10.0/(ttc*ttc);
    marker.scale.y = (double)10.0/(ttc*ttc);
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Publish markers
    pub_ttc.publish(marker);


}

void cbPC2(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // Conversion from ROS PointCloud2 to PCL PointCloud
    PointCloudXYZ::Ptr cloud_in(new PointCloudXYZ);
    pcl::fromROSMsg(*input, *cloud_in);

    PointCloudXYZ::Ptr ROI(new PointCloudXYZ);
    // geometry_msgs::Vector3 boat_vel = recv_odom.twist.twist.linear;

    geometry_msgs::Point boat_vel;
    boat_vel.x = velocity.x;
    boat_vel.y = velocity.y;
    boat_vel.z = velocity.z;
    // publishCentroidMarker(boat_vel);

    PointXYZ closest_pt;
    double m = boat_vel.y / boat_vel.x;
    float min_dist = FLT_MAX;

    for (auto &point : *cloud_in)
    {
        if ((point.y - (m * point.x)) > -3.0 && (point.y - (m * point.x)) < 3.0)
        {
            ROI->push_back(point);
            PointXYZ vel;
            vel.x = boat_vel.x;
            vel.y = boat_vel.y;
            vel.z = boat_vel.z; // not needed

            PointXYZ origin;
            origin.x = 0.0f;
            origin.y = 0.0f;
            origin.z = 0.0f;
            float dist = euclDist(origin, point);
            if (euclDist(origin, point) < min_dist)
            {
                closest_pt = point;
                min_dist = dist;
            }
        }
    }

    publishPC(ROI);

    // calculate distance
    PointXYZ vel;
    vel.x = boat_vel.x;
    vel.y = boat_vel.y;
    vel.z = boat_vel.z; // not needed

    float abs_boat_vel = 0, time = 0;

    if (getQuadrante(vel) == getQuadrante(closest_pt))
    {
        abs_boat_vel = sqrt(pow(boat_vel.x, 2) + pow(boat_vel.y, 2));
        time = min_dist / abs_boat_vel;
        // std::cout << "TTC (s): " << time << std::endl;
    }

    publishTTC(time);
    //std::cout << time_counter << " " << time << std::endl;
}

void cb_odom(const nav_msgs::Odometry odom)
{

    if (first_run)
    {
        old_odom = odom;
        first_run = false;

        std::cout << 0.0;
        return;
    }

    double elapsed_time = (odom.header.stamp - old_odom.header.stamp).toSec();
    time_counter += elapsed_time;

    //velocity.x = (double)(odom.pose.pose.position.x - old_odom.pose.pose.position.x) / elapsed_time;
    //velocity.y = (double)(odom.pose.pose.position.y - old_odom.pose.pose.position.y) / elapsed_time;

    velocity.x = (double)odom.twist.twist.linear.x;
    velocity.y = (double)odom.twist.twist.linear.y;

    geometry_msgs::Point boat_vel;
    boat_vel.x = velocity.x;
    boat_vel.y = velocity.y;
    publishVelocityMarker(boat_vel);

    // Save the old odometry msg
    old_odom = odom;

}

int main(int argc, char **argv)
{
    // Init the ros system
    ros::init(argc, argv, "risk_assessor_node");

    // Create the node handles to establish the program as a ROS node
    ros::NodeHandle n_public;

    ros::Subscriber sub_pc = n_public.subscribe<sensor_msgs::PointCloud2>("/detector/obstacle", 1, cbPC2);
    ros::Subscriber sub_odom = n_public.subscribe<nav_msgs::Odometry>("/gps/rtkfix", 1, cb_odom);

    pub_ttc = n_public.advertise<visualization_msgs::Marker>("risk_assessor/ttc_marker", 1, false);
    vis_pub = n_public.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    pub_ROI_pc = n_public.advertise<sensor_msgs::PointCloud2>("/risk_assesor/ROI", 1);
    pub_ttc_time_msg = n_public.advertise<std_msgs::Time>("/risk_assessor/time_to_collision", 1);

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}