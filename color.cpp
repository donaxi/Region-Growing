#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl17/ros/conversions.h>
//#include <pcl/conversions.h>
#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <iostream>
#include <vector>
#include <pcl17/io/pcd_io.h>
#include <pcl17/search/search.h>
#include <pcl17/search/kdtree.h>
#include <pcl17/visualization/cloud_viewer.h>
#include <pcl17/filters/passthrough.h>
#include <pcl17/segmentation/region_growing_rgb.h>
#include <boost/shared_ptr.hpp>
//#include <boost/filesystem.hpp>
//#include <usr/share/doc/libboost_system.1.46.1>
//#include <pcl/io.hpp>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  boost::shared_ptr<pcl17::search::KdTree<pcl17::PointXYZRGB> > tree (new pcl17::search::KdTree<pcl17::PointXYZRGB> ()); 
  //pcl17::search::Search <pcl17::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl17::search::Search<pcl17::PointXYZRGB> > (new  pcl17::search::KdTree<pcl17::PointXYZRGB>);

  //pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr cloud (new pcl17::PointCloud <pcl17::PointXYZRGB>);
  boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > cloud (new pcl17::PointCloud<pcl17::PointXYZRGB>);
  //boost::shared_ptr<pcl17::PointCloud<pcl17::PointXYZRGB> > input_1 (new pcl17::PointCloud<pcl17::PointXYZRGB>); 
  //pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr input_1;
  if ( pcl17::io::loadPCDFile <pcl17::PointXYZRGB> ("/home/acp/fuerte_workspace/color/src/region_growing_rgb_tutorial.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    //return (-1);
  }
  
  //pcl17::fromROSMsg (*input, *input_1);
  //boost::shared_ptr<pcl17::Indices> indices (new std::vector <int>);
  pcl17::IndicesPtr indices (new std::vector <int>);
  boost::shared_ptr<pcl17::PassThrough<pcl17::PointXYZRGB> > pass (new pcl17::PassThrough<pcl17::PointXYZRGB>);
  //pcl17::PassThrough<pcl17::PointXYZRGB> pass;
  pass->setInputCloud (cloud);
  pass->setFilterFieldName ("z");
  pass->setFilterLimits (0.0, 1.0);
  pass->filter (*indices);

  boost::shared_ptr<pcl17::RegionGrowingRGB<pcl17::PointXYZRGB> >  reg (new pcl17::RegionGrowingRGB<pcl17::PointXYZRGB>);
  //pcl17::RegionGrowingRGB<pcl17::PointXYZRGB> reg;
  reg->setInputCloud (cloud);
  reg->setIndices (indices);
  reg->setSearchMethod (tree);
  reg->setDistanceThreshold (10);
  reg->setPointColorThreshold (6);
  reg->setRegionColorThreshold (5);
  reg->setMinClusterSize (600);

  //boost::shared_ptr<std::vector <pcl::PointIndices> > clusters (new std::vector <pcl::PointIndices>);
  std::vector <pcl17::PointIndices> clusters;
  reg->extract (clusters);

  boost::shared_ptr<pcl17::PointCloud <pcl17::PointXYZRGB> > colored_cloud (new pcl17::PointCloud <pcl17::PointXYZRGB>);
  //pcl17::PointCloud<pcl17::PointXYZRGB> colored_cloud= reg.getColoredCloud ();
  colored_cloud= reg->getColoredCloud ();
  //pcl17::visualization::CloudViewer viewer ("Cluster viewer");
  //reg->getColoredCloud(*colored_cloud);
  //boost::shared_ptr<pcl17::visualization::CloudViewer> viewer (new pcl17::visualization::CloudViewer);
  //viewer.showCloud (colored_cloud);
  //while (!viewer->wasStopped ())
  //{
  //boost::this_thread::sleep (boost::posix_time::microseconds (100));
  //}
  boost::shared_ptr<sensor_msgs::PointCloud2> final (new sensor_msgs::PointCloud2);
   //sensor_msgs::PointCloud2::Ptr final;
  pcl17::toROSMsg (*colored_cloud, *final);// Se cambia el formato de la variable que contine la nube de puntos.
  //ROS_INFO("%d",final->header.frame_id);
  final-> header.frame_id= "kinect";
  //std::cerr << final->header.frame_id << std::endl;
  // Publish the data
  pub.publish (*final);
  

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "color");
  ros::NodeHandle nh;
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("color_1", 1);

  // Spin
  ros::spin ();
}
