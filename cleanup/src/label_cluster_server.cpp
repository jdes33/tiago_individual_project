#include "ros/ros.h"
#include <ros/package.h>
#include <set>
#include <utility>      // std::pair, std::get
#include "yolo4/DetectObjects.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class ClusterLabeler{
public:
  ClusterLabeler(ros::NodeHandle& nh){
    nh_ = nh;
    ROS_INFO("label_cluster_server service running");
    server_ = nh_.advertiseService("label_cluster", &ClusterLabeler::label_cluster, this);
  }

  bool label_cluster(){

    // get yolo detections
    ros::ServiceClient client = nh_.serviceClient<yolo4::DetectObjects>("/yolo_server/detect_objects");
    yolo4::DetectObjects srv;
    if (client.call())
    {
      std::cout << "hey it worked";
      std::cout << srv.response.detection;
    }
    else
    {
      std::cout << "fuck it failed";
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }


    // load point cloud
    std::string detectedObjectPath = ros::package::getPath("cleanup") + "/data/object/detected_screwdriver.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (detectedObjectPath, *objectCloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");  // jason: maybe change this to ros error message
      return (-1);
    }
    std::cout << "Loaded "
              << objectCloud->width * objectCloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (const auto& point: *objectCloud)
     std::cout << "    " << point.x
                << " "    << point.y
                << " "    << point.z << std::endl;

    // get depth image
    sensor_msgs::Image::Ptr depthImage = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/depth_registered/image_raw",create_path);

    // yolo boxes
    // get rgb image
    // get yolo boxes according to rgb image
    //boxes = [(20, 20, 100, 100), (100,100,200,200)]

    /*std::set<std::pair<int, int>> myset;
    for (const auto& point: *objectCloud){
      // get image point and store it
      int image_x;
      int image_y;
      getImagePoint(point.x, point.y, point.z, image_x, image_y);
      myset.insert(std::make_pair(image_x, image_y));
    }
  */
  }
private:
  ros::ServiceServer server_;
  ros::NodeHandle nh_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "label_cluster_server");
  ros::NodeHandle nh;

  //ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ClusterLabeler clusterLabeler(nh);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
