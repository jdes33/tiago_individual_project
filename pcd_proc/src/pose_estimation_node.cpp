

// this node will provide a service for pose estimation
// the node will take as input a pointcloud2
// it will also take in the name of a pcd file
// the pcd files will be stored in a directory in this package

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

// NEEDED FOR SERVICE
#include <pcd_proc/GetPoseEst.h>

#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "pcl_ros/transforms.h"
/*
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
*/


class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputSource (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());
      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);
      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }
      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};




class PoseEstimator
{
public:
  PoseEstimator(ros::NodeHandle& nh)
  {
    nh_ = nh;
    ROS_INFO("Pose estimator service running");

    nh.getParam("pose_estimation_node/robot_base_frame", robotBaseFrame);
    //nh.getParam("pcd_processing_node/cloud_topic1", cloud_topic1);
    //if (!nh.getParam("stuff", data_)) throw std::runtime_error("Must provide parameter 'stuff'");

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("pose_est_output", 1, true);
    objectLibraryPath = ros::package::getPath("pcd_proc") + "/data/object_library/";
    // Create ROS interfaces
    server_ = nh.advertiseService("get_pose_estimate", &PoseEstimator::getPoseEstimate, this);

  }


  bool getPoseEstimate(pcd_proc::GetPoseEstRequest& req,
                    pcd_proc::GetPoseEstResponse& res)
  {


    std::cout << "wazzup kid" << std::endl;
    std::cout << objectLibraryPath << std::endl;


    std::vector<FeatureCloud> objectTemplates;
    objectTemplates.resize(0);
    // LOAD THE TEMPLATE FROM THE DATA FOLDER, FILE NAME FROM REQUEST
    for(auto templateName : req.file_names)
    {
      FeatureCloud template_cloud;
      template_cloud.loadInputCloud (objectLibraryPath + templateName.data + ".pcd");
      std::cout << "HEY THERE I loaded" << objectLibraryPath + templateName.data + ".pcd" << std::endl;
      objectTemplates.push_back (template_cloud);

    }

    // transform from camera frame (or whatevere frame it is in) to base_footprint
    tf::TransformListener listener;
    tf::StampedTransform stransform;
    try
    {
      listener.waitForTransform(robotBaseFrame, req.detected_object.header.frame_id, ros::Time::now(), ros::Duration(6.0));
      listener.lookupTransform(robotBaseFrame, req.detected_object.header.frame_id, ros::Time(0), stransform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    sensor_msgs::PointCloud2 transformedRoscloud;
    pcl_ros::transformPointCloud(robotBaseFrame, stransform, req.detected_object, transformedRoscloud);

    // Convert target PointCloud to pcl PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(transformedRoscloud, cloud);
    // input cloud must be a pointer, so we make a new cloudPtr from the cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    ROS_INFO_STREAM("Detected object cloud has " << cloudPtr->size() << " points");

    std::cout << "CHECKPOINT 1" << std::endl;

      // Preprocess the cloud by...
      // ...removing distant points
      const float depth_limit = 1.0;
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloudPtr);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0, depth_limit);
      pass.filter (*cloudPtr);

      // ... and downsampling the point cloud
      const float voxel_grid_size = 0.005f;
      pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
      vox_grid.setInputCloud (cloudPtr);
      vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
      //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
      vox_grid.filter (*tempCloud);
      cloudPtr = tempCloud;

      // Assign to the target FeatureCloud
      FeatureCloud targetCloud;
      targetCloud.setInputCloud (cloudPtr);

      // Set the TemplateAlignment inputs
      TemplateAlignment templateAlign;
      for (size_t i = 0; i < objectTemplates.size (); ++i)
      {
        templateAlign.addTemplateCloud (objectTemplates[i]);
      }
      templateAlign.setTargetCloud (targetCloud);

      // Find the best template alignment
      TemplateAlignment::Result bestAlignment;
      int bestIndex = templateAlign.findBestAlignment (bestAlignment);
      const FeatureCloud &bestTemplate = objectTemplates[bestIndex];

      // Print the alignment fitness score (values less than 0.00002 are good)
      printf ("Best fitness score: %f\n", bestAlignment.fitness_score);

      // Print the rotation matrix and translation vector
      Eigen::Matrix3f rotation = bestAlignment.final_transformation.block<3,3>(0, 0);
      Eigen::Vector3f translation = bestAlignment.final_transformation.block<3,1>(0, 3);

      printf ("\n");
      printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
      printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
      printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
      printf ("\n");
      printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

      //res.estimated_pose.position.x = translation(0);
      //res.estimated_pose.position.x = translation(1);
      //res.estimated_pose.position.x = translation(2);


      // Save the aligned template for visualization
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
      pcl::transformPointCloud (*bestTemplate.getPointCloud (), transformed_cloud, bestAlignment.final_transformation);
      pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);

      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(transformed_cloud, output);
      output.header.frame_id = "base_footprint";


/*
      Eigen::Matrix4d md(bestAlignment.final_transformation.cast<double>());
      Eigen::Affine3d affine(md);
      tf::poseEigenToMsg(affine, res.estimated_pose);
      res.transformed_cloud = output;
      res.index = bestIndex;
      res.fitness_score = bestAlignment.fitness_score;
*/
    Eigen::Matrix4d md(bestAlignment.final_transformation.cast<double>());
    Eigen::Affine3d affine(md);
    tf::transformEigenToMsg(affine, res.estimated_transform);
    res.transformed_cloud = output;
    res.index = bestIndex;
    res.fitness_score = bestAlignment.fitness_score;

      // send the pose estimation transform (for visualisation and interaction purposes, i.e pulling drawers open/closed)
      //tf::Transform objectTransformFromBase;
      //tf::poseEigenToMsg(affine, objectTransformFromBase);
      //transformBroadcaster_.sendTransform(tf::StampedTransform(objectTransformFromBase, ros::Time::now(), "base_footprint", "estimated pose"));

    // Publish the data.
    pub.publish (output);

    return true;

  }

  private:
    ros::ServiceServer server_;
    //ros::Publisher croppedPub_, objectPub_, clusterPub_, planePub_, nonPlanePub_;
    ros::NodeHandle nh_;
    //tf::TransformBroadcaster transformBroadcaster_;



    std::string objectLibraryPath, robotBaseFrame;

    // Configuration data
    ros::Publisher pub;

  };



int
main (int argc, char** argv)
{
  std::cout << "STARTING";

  // Initialize ROS
  ros::init (argc, argv, "pose_estimation_node");
  ros::NodeHandle nh;

  PoseEstimator poseEstimator(nh);

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);



  ROS_DEBUG("Hello %s", "World");

  // Spin
  ros::spin ();
}
