#include <iostream>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int         i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "planar_segmentation_node");

  ros::NodeHandle nh;
  ros::Publisher seg_pub;
  seg_pub = nh.advertise<sensor_msgs::PointCloud2>("PointXYZRGB",1);

  // while (ros::ok())
  // {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data
    cloud->width  = 15;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // Generate the data
    for (auto& point: *cloud)
    {
      point.x = 1024 * rand () / (RAND_MAX + 1.0f);
      point.y = 1024 * rand () / (RAND_MAX + 1.0f);
      point.z = 1.0;
    }

    // Set a few outliers
    (*cloud)[0].z = 2.0;
    (*cloud)[3].z = -2.0;
    (*cloud)[6].z = 4.0;

    std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
    for (const auto& point: *cloud)
      std::cerr << "    " << point.x << " "
                          << point.y << " "
                          << point.z << std::endl;

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    colorize(*cloud, *cloud_colored, {255, 0, 0});
    // colorize(*tgt, *tgt_colored, {0, 255, 0});

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
    // ROS_WARN_STREAM(coefficients->values[0]);

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    // for (std::size_t i = 0; i < inliers->indices.size (); ++i)
    for (const auto& idx: inliers->indices)
    {
      std::cerr << idx << "    " << cloud->points[idx].x << " "
                                << cloud->points[idx].y << " "
                                << cloud->points[idx].z << std::endl;
      // ROS_WARN_STREAM("YES");
    }
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(*cloud_colored, cloud_out);
  cloud_out.header.frame_id = "map";
  cloud_out.header.stamp = ros::Time::now();
  std::cout << cloud->points[0].x << std::endl;
  ROS_WARN_STREAM("map!");

  seg_pub.publish(cloud_out);
// }
//PCLVisualizer
// pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
// viewer1.addPointCloud<pcl::PointXYZRGB>(cloud_colored);
// viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_green");

// while (!viewer1.wasStopped()) {
//     viewer1.spinOnce();
// }

//CloudViewer
// pcl::visualization::CloudViewer viewer2("Cloud Viewer");
// viewer2.showCloud(cloud_colored);
// int cnt = 0;
// while (!viewer2.wasStopped()) {
//     cnt++;
// }

  return (0);
}
