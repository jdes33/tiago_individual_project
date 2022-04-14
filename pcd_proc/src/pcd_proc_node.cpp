#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#include <visualization_msgs/Marker.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

// polygonal segmentation
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/surface/convex_hull.h>

// Needed to return a Pose message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// Needed for ROS Service
#include "pcd_proc/Cluster.h"
#include <pcd_proc/GetClusters.h>






class PointCloudProcessor
{
public:
  PointCloudProcessor(ros::NodeHandle& nh)
  {
    nh_ = nh;
    ROS_INFO("Perception service running");

    nh.getParam("pcd_processing_node/cloud_topic1", cloud_topic1);
    nh.getParam("pcd_processing_node/cloud_topic2", cloud_topic2);
    nh.getParam("pcd_processing_node/cloud_topic3", cloud_topic3);
    nh.getParam("pcd_processing_node/world_frame", world_frame);
    nh.getParam("pcd_processing_node/camera_frame", camera_frame);
    nh.getParam("pcd_processing_node/voxel_leaf_size", voxel_leaf_size);
    nh.getParam("pcd_processing_node/x_filter_min", x_filter_min);
    nh.getParam("pcd_processing_node/x_filter_max", x_filter_max);
    nh.getParam("pcd_processing_node/y_filter_min", y_filter_min);
    nh.getParam("pcd_processing_node/y_filter_max", y_filter_max);
    nh.getParam("pcd_processing_node/z_filter_min", z_filter_min);
    nh.getParam("pcd_processing_node/z_filter_max", z_filter_max);
    nh.getParamCached("pcd_processing_node/plane_max_iterations", plane_max_iter);
    nh.getParamCached("pcd_processing_node/plane_distance_threshold", plane_dist_thresh);
    nh.getParam("pcd_processing_node/cluster_tolerance", cluster_tol);
    nh.getParam("pcd_processing_node/cluster_min_size", cluster_min_size);
    nh.getParam("pcd_processing_node/cluster_max_size", cluster_max_size);
    nh.getParam("cloud_debug", debug_);

    // TODO (for SwRI. Not for the student) Add check parameters
    //    if (!nh.getParam("stuff", data_)) throw std::runtime_error("Must provide parameter 'stuff'");

    // Create ROS interfaces
    //server_ = nh.advertiseService("find_pick", &PointCloudProcessor::findPickPose, this);
    server_ = nh.advertiseService("get_clusters", &PointCloudProcessor::getObjectClusters, this);

    if (debug_)
    {
      ROS_INFO("Perception Debug Enabled: Intermediate clouds are being published");
      cropped_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
      object_pub_ = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1, true);
      cluster_pub_ = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1, true);
      pick_surface_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pick_surface", 1, true);
    }
  }






  bool getObjectClusters(pcd_proc::GetClustersRequest& req,
                    pcd_proc::GetClustersResponse& res)
  {

    res.succeeded = true;
    ROS_INFO("Get clusters service running");


    // I THINK YOU CAN MAKE ALL YOUR FUNCTINOS TAKE IN A CONST cloudPtr, also you maybe able to mark the functions as const

    // NEED TO MAKE RES.SUCCESSED FALSE WHEN FAILING, EITHER PASS INTO FUNCTIONS OR DO THE CHECKS I NTHIS FUNCTION OR MAKE THE FUNCTIONS BOOL

    // obtain pointcloud
    std::string topic1 = nh_.resolveName(cloud_topic1);
    ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic1);
    sensor_msgs::PointCloud2::ConstPtr rosCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic1, nh_);

    // transform from camera to base_footprint
    // TRANSFORM POINTCLOUDS FROM CAMERA FRAME TO WORLD FRAME
    tf::TransformListener listener;
    tf::StampedTransform stransform;
    try
    {
      listener.waitForTransform(world_frame, rosCloud->header.frame_id, ros::Time::now(), ros::Duration(6.0));
      listener.lookupTransform(world_frame, rosCloud->header.frame_id, ros::Time(0), stransform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    sensor_msgs::PointCloud2 transformedRoscloud;
    pcl_ros::transformPointCloud(world_frame, stransform, *rosCloud, transformedRoscloud);

    // convert from ros PointCloud to pcl PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(transformedRoscloud, cloud);
    // input cloud must be a pointer, so we make a new cloud_ptr from the cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    ROS_INFO_STREAM("Original cloud  had " << cloudPtr->size() << " points");

    // pass through filter or cropbox filter
    cropBoxFilter(cloudPtr);

    // voxel grid downsample
    voxelGridDownsample(cloudPtr);

    // plane segmentation and removal
    removePlane(cloudPtr);

    // cluster extraction
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclClusters = extractClusters(cloudPtr);

    // convert clusters to ros pointlcouds
    // iterate through each cluster in clusters, convert, set header and timestamp
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCluster : pclClusters){
      pcd_proc::Cluster cluster;
      pcl::toROSMsg(*pclCluster, cluster.pointcloud);
      cluster_pub_.publish(cluster.pointcloud);
      cluster.pointcloud.header.frame_id = world_frame;
      cluster.pointcloud.header.stamp = ros::Time::now();
      cluster_pub_.publish(cluster.pointcloud);
      // MIGHT GOTTA COMBINE THSES TWO INTO ONE FUNCTION LATER
      extractTopPose(cloudPtr, cluster);
      extract3DBox(cloudPtr, cluster);
      res.clusters.push_back(cluster);
    }
    //cluster_pub_.publish(res.clusters.at(0).pointcloud);


    // optional statistical outlier removal (MAYBE DO WITHING EXTRACT CLUSTERS)

    return true;
  }









  void cropBoxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr){
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloudPtr);
    Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    crop.filter(*cloudPtr);

    ROS_INFO_STREAM("Cropped point cloud: " << (*cloudPtr).points.size() << " data points.");
    // Publish cropped cloud
    if (debug_)
    {
      sensor_msgs::PointCloud2::Ptr cropped(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloudPtr, *cropped);
      //    pcl::toROSMsg(xyz_filtered_cloud, *cropped);
      cropped->header.frame_id = world_frame;
      cropped->header.stamp = ros::Time::now();
      cropped_pub_.publish(*cropped);
    }
  }

  void voxelGridDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr){
    // create an instance of the pcl VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloudPtr);
    voxel_filter.setLeafSize(float(voxel_leaf_size), float(voxel_leaf_size), float(voxel_leaf_size));
    voxel_filter.filter(*cloudPtr);

    ROS_INFO_STREAM("Downsampled cloud  with " << cloudPtr->size() << " points");
  }


  void removePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr){

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));  // this passes in either passthrough or crop filtered cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_max_iter);
    seg.setDistanceThreshold(plane_dist_thresh);
    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud(cloudPtr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() > 0)
    {
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloudPtr);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Get the points associated with the planar surface
      extract.filter(*cloud_plane);
      ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size()
                                                                       << " data points.");

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloudPtr);
    }
    else
    {
      ROS_WARN_STREAM("Could not estimate a planar model for the given dataset. Proceeding with full point cloud.");
      //*cloud_f = *cropped_cloud;
      //res.succeeded = false;
    }
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr){

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloudPtr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tol);  // 2cm
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloudPtr);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);

      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        cloudCluster->points.push_back(cloudPtr->points[*pit]);

      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      std::cout << "Cluster has " << cloudCluster->points.size() << " points.\n";
      clusters.push_back(cloudCluster);

    }

    if (clusters.size() == 0)
    {
      ROS_WARN_STREAM("Clustering failed. Proceeding with full point cloud.");
      //res.succeeded = false;
    }

    return clusters;
  }


  void extractTopPose(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr, pcd_proc::Cluster& cluster){
    geometry_msgs::Pose part_pose;
    part_pose.position.x = 1;
    part_pose.position.y = 1;
    part_pose.position.z = 1;
    part_pose.orientation.x = 0;
    part_pose.orientation.y = 0;
    part_pose.orientation.z = 0;
    part_pose.orientation.w = 1;
    cluster.top_center_pose = part_pose;
  }

  void extract3DBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr, pcd_proc::Cluster& cluster){
    //Eigen::Vector4f min_pt, max_pt;
    //pcl::getMinMax3D(*cloud_planeTop, min_pt, max_pt);
    cluster.min_pt.x = 0;
    cluster.min_pt.y = 0;
    cluster.min_pt.z = 0;
    cluster.max_pt.x = 0;
    cluster.max_pt.y = 0;
    cluster.max_pt.z = 0;
  }



private:
  ros::ServiceServer server_;
  ros::ServiceServer server2_;
  ros::Publisher cropped_pub_, object_pub_, cluster_pub_, pick_surface_pub_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;

  // Configuration data
  std::string cloud_topic1, cloud_topic2, cloud_topic3, world_frame, camera_frame;
  double voxel_leaf_size;
  double x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  double plane_max_iter, plane_dist_thresh;
  double cluster_tol;
  int cluster_min_size, cluster_max_size;
  bool debug_ = false;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pcd_processing_node");
  ros::NodeHandle nh;

  PointCloudProcessor pointCloudProcessor(nh);
  ROS_INFO("PointCloudProcessor server ready to find pick targets");
  ros::spin();

  return 0;
}
