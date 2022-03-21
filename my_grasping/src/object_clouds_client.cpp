#include "ros/ros.h"
#include "my_grasping/GetObjectClouds.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_object_clouds_client");
  if (argc != 3)
  {
    ROS_INFO("usage: get_object_clouds_client X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_grasping::GetObjectClouds>("get_object_clouds");
  my_grasping::GetObjectClouds srv;
  //srv.request.a = atoll(argv[1]);
  //srv.request.b = atoll(argv[2]);
  //srv.request.entire_cloud = 
  if (client.call(srv))
  {
    ROS_INFO("gesundheit!");
  }
  else
  {
    ROS_ERROR("Failed to call service get_object_clouds");
    return 1;
  }

  return 0;
}
