#include "ros/ros.h"
#include "my_grasping/AddTwoInts.h"
#include "my_grasping/GetObjectClouds.h"


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL segmentation specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

bool add(my_grasping::AddTwoInts::Request  &reqblah,
         my_grasping::AddTwoInts::Response &resblah)
{
  resblah.sum = reqblah.a + reqblah.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


// JASON: ideally request should be const, but causes error, may it's const by default or not needed
bool cluster(my_grasping::GetObjectClouds::Request &req, my_grasping::GetObjectClouds::Response &res)
{
  //res.sum = req.a + req.b;
  //res.objects = req.entire_cloud;
  res.objects.push_back(req.entire_cloud);

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (req.entire_cloud, cloud);

  std::cout << "PointCloud before filtering has: " << cloud.size () << " data points." << std::endl; //*



  // HOW DO WE CALL THIS SERVICE?? AND HOW DO WE SPECIFY A POINTCLOUD
  // maybe create a subsriber that keeps updating a pointcloud variable
  // then create the client and pass in the variable

  // alternatively, you already have service that saves point cloud to file
  // You can create function that reads the file and outputs a bunch more files (the clusters)
  // the request would be string path and the response would be string[] paths

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  //*cloud = req.entire_cloud;


  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::ServiceServer myService = n.advertiseService("get_object_clouds", cluster);
  ROS_INFO("Cluster Service ready.");

  ros::spin();

  return 0;
}
