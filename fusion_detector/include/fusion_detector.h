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
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "draw_rects.h"
#include "gencolors.h"
#include "cluster.h"
#include "rect_class_score.h"


#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_utils/geo/cube.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "miivii_fusion.h"
#include "miivii_util.h"

#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#define __APP_NAME__ "fusion_dectetor"

#define CAMERA_COUNT 4  //最大融合相机的数目

using namespace std;
using namespace cv;
using namespace miivii;

namespace enc = sensor_msgs::image_encodings;

class fusion_detector
{
public:
  ros::NodeHandle                                  nh_;
  MiiViiCameraParameterWrap                        m_cameraParameter[CAMERA_COUNT];
  MiiViiFusion                                     m_MiiViiFusion[CAMERA_COUNT];
public:
  uint                                             camCount;
  int                                              camCount_;
  std::string                                      calibration_file_[CAMERA_COUNT];
  std::string                                      camera_topic_name_[CAMERA_COUNT];

  bool                                             lidar_processing_;
  bool                                             draw_fusion_pts_[CAMERA_COUNT];
  bool                                             enable_show_results_window_;


  image_transport::Subscriber                      image_sub_[CAMERA_COUNT];
  ros::Subscriber                                  lidar_sub_;

  ros::Subscriber                                  yolo_detect_sub_[CAMERA_COUNT];
  image_transport::Publisher                       fusion_results_image_pub_[CAMERA_COUNT];
  ros::Publisher                                   _pub_jsk_boundingboxes;
  ros::Publisher                                   publisher_fused_text_;

  std::string                                      detected_objects_vision[CAMERA_COUNT];
  std::string                                      pub_jsk_boundingboxes_topic_;
  std::string                                      lidar_topic_name_;

  autoware_msgs::DetectedObjectArray::ConstPtr     detected_objects_msg_[CAMERA_COUNT];

  bool                                             bGetingImage[CAMERA_COUNT];
  bool                                             _pose_estimation;
  bool                                             _use_gpu;
  bool                                             debug_time;           //是否在终端打印单帧处理耗时信息

  cv::Mat                                          *InImage[CAMERA_COUNT];

  std_msgs::Header                                 lidar_msg_header;

  std::vector<cv::Scalar>                          _colors;

  int                                              marker_id_;
public:
  fusion_detector():nh_("~")
  {
    Initialize_Camera_lidar();
  }
public:
  ~fusion_detector()
  {
    for(int kk=0; kk<CAMERA_COUNT; kk++)
      detected_objects_msg_[kk] = nullptr;
  }
public:
  void Initialize_Camera_lidar();

  void cameraCallback(const sensor_msgs::ImageConstPtr& msg,int cameraID);

  void yolo_detect_Callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg,int cameraID);

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr & msg);

  void fusion_lidar_cameras(sensor_msgs::PointCloud2ConstPtr LiDarData);

  void points_to_image_new( sensor_msgs::PointCloud2ConstPtr LiDarData);

  void publishBoundingBoxArray( const ros::Publisher* in_publisher,
                                const jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array);

  visualization_msgs::MarkerArray ObjectsToMarkers(const autoware_msgs::DetectedObjectArray &in_objects);

  jsk_recognition_msgs::BoundingBoxArray ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects);

};

