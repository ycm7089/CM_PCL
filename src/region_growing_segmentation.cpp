#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <typeinfo>
#include <sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>
#include "cm_pcl/cm_configConfig.h"

class cm_l515_param
{
public:
  ros::Publisher seg_pub;
  ros::Subscriber realsense_sub;
  pcl::PointCloud<pcl::PointXYZ> cloud_dst;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  double depth_width, depth_height, depth_fps, color_width, color_height, color_fps;
  bool enable_depth, enable_color;
  cm_l515_param(ros::NodeHandle *nh)
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    seg_pub = nh->advertise<sensor_msgs::PointCloud2>("PointXYZRGB",1);

    realsense_sub = nh->subscribe("/camera/depth/color/points",1, &cm_l515_param::realsense_callback,this); 
    std::cout << "ddd" <<std::endl;
    //Parse params from launch file 
    nh->getParam("depth_width", depth_width);
    nh->getParam("depth_height", depth_height); 
    nh->getParam("depth_fps", depth_fps);
    nh->getParam("color_width",color_width);
    nh->getParam("color_height",color_height);
    nh->getParam("color_fps",color_fps);
    nh->getParam("enable_depth",enable_depth);
    nh->getParam("enable_color",enable_color);
  }
public:
  void realsense_callback(const sensor_msgs::PointCloud2::Ptr&msg);
  void segmentation();
};
void callback(cm_pcl::cm_configConfig &config, uint32_t)
{
  ROS_INFO("Reconfigure Request: %d %d %d %f ", 
            config.setMinClusterSize, config.setMaxClusterSize, 
            config.setNumberOfNeighbours, 
            config.setSmoothnessThreshold);  
}
void cm_l515_param::segmentation()
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("/home/chanzz/catkin_ws/src/cm_pcl/region_growing_tutorial.pcd", *cloud) == -1)
  // {
  //   std::cout << "Cloud reading failed." << std::endl;
  //   // return -1;
  // }
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (5000);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  // reg.setMinClusterSize (50);
  // reg.setMaxClusterSize (1000000);
  // reg.setSearchMethod (tree);
  // reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (cloud);
  // //reg.setIndices (indices);
  // reg.setInputNormals (normals);
  // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  // reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  // while (counter < clusters[0].indices.size ())
  // {
    // std::cout << clusters[0].indices[counter] << ", ";
    // counter++;
    // if (counter % 10 == 0)
      // std::cout << std::endl;
  // }
  // std::cout << std::endl;
  sensor_msgs::PointCloud2 cloud_out;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  std::cout << cloud->points[0].x << std::endl;
  pcl::toROSMsg(*colored_cloud, cloud_out); //pcl -> pointcloud
  ROS_WARN_STREAM("sadasd");
  cloud_out.header.frame_id = "camera_link";
  // cloud_out.header.stamp = ros::Time::now();
  ROS_WARN_STREAM("camera_link!");
  // seg_pub.publish(cloud_out);

  pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(cloud_out, *cloud2); //pointcloud -> pcl

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.05, 0.05, 0.05); // 샘플링 하는 방법 이거 너무 작게 하면 샘플링 에러 메세지 뜸 고것을 주의 하자
  sor.filter(cloud_filtered);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output); // pcl -> pointcloud
  output.header.frame_id = "camera_link";
  output.header.stamp = ros::Time::now();
  seg_pub.publish(output);

}
void cm_l515_param::realsense_callback(const sensor_msgs::PointCloud2::Ptr&msg)
{
  std::cout << "dddasd" <<std::endl;

  pcl::fromROSMsg(*msg,*cloud);
  segmentation();
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Segmentation_node");
  
  ros::NodeHandle nh("~");
  dynamic_reconfigure::Server<cm_pcl::cm_configConfig> server;
  dynamic_reconfigure::Server<cm_pcl::cm_configConfig>::CallbackType f;
  f = boost::bind(&callback,_1, _2);
  server.setCallback(f);

  cm_l515_param cm_l515_params(&nh);
  while (ros::ok())
  {
    ros::spinOnce();

  }  
  return (0);
}
