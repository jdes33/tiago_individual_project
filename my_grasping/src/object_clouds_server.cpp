#include "ros/ros.h"
#include "my_grasping/AddTwoInts.h"
#include "my_grasping/GetObjectClouds.h"


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool add(my_grasping::AddTwoInts::Request  &req,
         my_grasping::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


// JASON: ideally request should be const, but causes error, may it's const by default or not needed
bool cluster(my_grasping::GetObjectClouds::Request &req, my_grasping::GetObjectClouds::Response &res)
{
  //res.sum = req.a + req.b;
  //res.objects = req.entire_cloud;
  res.objects.push_back(req.entire_cloud);
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
