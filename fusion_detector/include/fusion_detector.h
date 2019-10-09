#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <cmath>

#include "ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <sensor_msgs/PointCloud2.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_utils/geo/cube.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "draw_rects.h"
#include "rect_class_score.h"
#include "MvFusion.h"
#include "MvFusionUtil.h"

#define __APP_NAME__ "fusion_dectetor"

#define CAMERA_COUNT 4  //最大融合相机的数目

using namespace std;
using namespace cv;
using namespace miivii;
using namespace fusion;

namespace enc = sensor_msgs::image_encodings;

class fusion_detector
{
public:
  ros::NodeHandle                                  nh_;

  //相机参数相关
  std::string                                      calibration_file_[CAMERA_COUNT];
  MvCameraParameter                                *camerasParam;
  MvFusion                                         m_MiiViiFusion;
public:

  uint                                             camCount;//当前处理相机数目，最多CAMERA_COUNT台相机
  int                                              camCount_;

  bool                                             lidar_processing_;
  autoware_msgs::DetectedObjectArray::ConstPtr     detected_objects_msg_[CAMERA_COUNT];
  bool                                             _pose_estimation;
  bool                                             debug_time;           //是否在终端打印单帧处理耗时信息

  pcl::PointCloud<pcl::PointXYZ>::Ptr              downsample_incloud;
  bool                                             m_downsample_cloud ;//是否对点云进行采样

  pcl::PointCloud<pcl::PointXYZ>::Ptr              tempdetectObj_cloud_ptr;//输出检测物体的点云

  bool                                             bGetingImage[CAMERA_COUNT];
  bool                                             bGettedImage[CAMERA_COUNT];
  cv::Mat                                          *InImage[CAMERA_COUNT];

  std_msgs::Header                                 lidar_msg_header;

  //std::vector<cv::Scalar>                          _colors;
  cv_bridge::CvImagePtr                            cv_ptr;
  int                                              marker_id_;
  double                                           merge_threshold;

public://订阅的topic
  //订阅相机图像topic
  std::string                                      camera_topic_name_[CAMERA_COUNT];
  image_transport::Subscriber                      image_sub_[CAMERA_COUNT];

  //订阅lidar数据topic
  std::string                                      lidar_topic_name_;
  ros::Subscriber                                  lidar_sub_;

  //订阅的miivii_detector topic
  std::string                                      detected_objects_vision[CAMERA_COUNT];
  ros::Subscriber                                  miivii_detector_sub_[CAMERA_COUNT];

public://发布的topic及相关数据定义
  //发布2d融合结果 图像topic
  bool                                             visualize_fusion;//是否发布2d融合结果topic
  image_transport::Publisher                       fusion_results_image_pub_[CAMERA_COUNT];//2d融合结果图像topic

  //发布bbox结果topic
  std::string                                      pub_jsk_boundingboxes_topic_;
  ros::Publisher                                   _pub_jsk_boundingboxes;

  //发布点云中label信息topic
  std::string                                      pub_combined_objects_labels;
  ros::Publisher                                   publisher_fused_text_;

  //发布剔除检测物体后的点云topic
  bool                                             enbable_publish_points_clounds_without_detectObjs;
  std::string                                      points_clounds_withour_detectObjs_;
  ros::Publisher                                   _publisher_points_clounds_without_detectObjs;
  pcl::PointCloud<pcl::PointXYZ>::Ptr              in_point_clouds;


public:
  fusion_detector():nh_("~")
  {
    Initialize_Camera_lidar();
  }
public:
  ~fusion_detector()
  {
    ReleaseBuffer();
  }
public:
  void Initialize_Camera_lidar();

  void ReleaseBuffer();

  void cameraCallback(const sensor_msgs::ImageConstPtr& msg,int cameraID);

  void miivii_detector_Callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg, int cameraID);

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr & msg);

  void fusion_lidar_cameras(const pcl::PointCloud<pcl::PointXYZ>::Ptr  incloud);

  void points_to_image( const pcl::PointCloud<pcl::PointXYZ>::Ptr  in_origin_cloud_ptr);

  void publishBoundingBoxArray( const ros::Publisher* in_publisher,
                                const jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array);

  void publishCloud(const ros::Publisher *in_publisher,
                    const  std_msgs::Header& in_header,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr);

  visualization_msgs::MarkerArray ObjectsToMarkers(const autoware_msgs::DetectedObjectArray &in_objects);
};

