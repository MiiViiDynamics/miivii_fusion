#include "fusion_detector.h"
#include <unistd.h>

#define USE_DEBUG 1
#if USE_DEBUG
#define DEBUG_PRINT \
  ROS_INFO("<MIIVII-DEBUG> [%s] %d", __FUNCTION__, __LINE__);
#endif

double what_time_is_it_now() {
  struct timeval time;
  if (gettimeofday(&time, NULL)) {
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void fusion_detector::ReleaseBuffer()
{
  in_point_clouds = nullptr;
  downsample_incloud = nullptr;
  tempdetectObj_cloud_ptr = nullptr;
  for(int kk=0; kk<camCount_; kk++){
    detected_objects_msg_[kk] = nullptr;
    InImage[kk]->release();
    if(InImage[kk] !=NULL)
    {
      InImage[kk] = NULL;
      delete InImage[kk];
    }
  }
}

void fusion_detector::Initialize_Camera_lidar(){

  nh_.param("camCount", camCount_, CAMERA_COUNT);
  camCount = camCount_;
  if (camCount > CAMERA_COUNT || camCount < 0) {
    camCount = 1;
  }

  for(int kk=0; kk<camCount_; kk++)
  {
    detected_objects_msg_[kk] = nullptr;
    detected_objects_msg_[kk] = boost::make_shared<autoware_msgs::DetectedObjectArray>();
  }

  in_point_clouds = nullptr;
  in_point_clouds = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  downsample_incloud = nullptr;
  downsample_incloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tempdetectObj_cloud_ptr = nullptr;
  tempdetectObj_cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  lidar_processing_ = false;
  image_transport::ImageTransport it_(nh_);


  nh_.param("camera1_cali_file", calibration_file_[0], std::string("/opt/miivii/data/calibration/camera_lidar/perception_front.yaml"));
  nh_.param("camera2_cali_file", calibration_file_[1], std::string("/opt/miivii/data/calibration/camera_lidar/perception_left.yaml"));
  nh_.param("camera3_cali_file", calibration_file_[2], std::string("/opt/miivii/data/calibration/camera_lidar/perception_right.yaml"));
  nh_.param("camera4_cali_file", calibration_file_[3], std::string("/opt/miivii/data/calibration/camera_lidar/perception_rear.yaml"));

  nh_.param("camera1_topic", camera_topic_name_[0], std::string("/camera_perception/perception/front/image_raw"));
  nh_.param("camera2_topic", camera_topic_name_[1], std::string("/camera_perception/perception/left/image_raw"));
  nh_.param("camera3_topic", camera_topic_name_[2], std::string("/camera_perception/perception/right/image_raw"));
  nh_.param("camera4_topic", camera_topic_name_[3], std::string("/camera_perception/perception/rear/image_raw"));

  nh_.param("camera1_detect_results", detected_objects_vision[0], std::string("/camera_perception/perception/front/detection_results"));
  nh_.param("camera2_detect_results", detected_objects_vision[1], std::string("/camera_perception/perception/left/detection_results"));
  nh_.param("camera3_detect_results", detected_objects_vision[2], std::string("/camera_perception/perception/right/detection_results"));
  nh_.param("camera4_detect_results", detected_objects_vision[3], std::string("/camera_perception/perception/rear/detection_results"));

  for(int i=0;i<camCount;i++)
  {
    ROS_INFO("%s",calibration_file_[i].c_str());
    ROS_INFO("%s",camera_topic_name_[i].c_str());
    ROS_INFO("%s",detected_objects_vision[i].c_str());

    m_cameraParameter[i].LoadCameraParameter(calibration_file_[i]);
    bGetingImage[i] = false;
    bGettedImage[i] = false;
    InImage[i] = new cv::Mat();
    InImage[i]->create(m_cameraParameter[i].imageSize.width, m_cameraParameter[i].imageSize.height, CV_8UC3);
  }

  // debug
  nh_.param<bool>("debug_time", debug_time, false);
  ROS_INFO("[%s] debug_time: %d", __APP_NAME__, debug_time);

  // 是否对点云进行采样
  nh_.param<bool>("downsample_cloud", m_downsample_cloud , false);
  ROS_INFO("[%s] m_downsample_cloud: %d", __APP_NAME__, m_downsample_cloud);


  nh_.param<bool>("visualize_fusion", visualize_fusion, false);

  nh_.param("pose_estimation", _pose_estimation, false);
  ROS_INFO("[%s] pose_estimation: %d", __APP_NAME__, _pose_estimation);

  nh_.param("cluster_merge_threshold", merge_threshold, 0.5);
  ROS_INFO("cluster_merge_threshold: %f", merge_threshold);

  //-----------------------------订阅topic相关------------------------------
  nh_.param("lidar_topic", lidar_topic_name_, std::string("/lidar_points"));
  ROS_INFO("%s",lidar_topic_name_.c_str());
  lidar_sub_ = nh_.subscribe(lidar_topic_name_.c_str(), 1, &fusion_detector::lidarCallback,this);

  for(int i=0;i<camCount;i++)
  {
    image_sub_[i] = it_.subscribe(camera_topic_name_[i].c_str(), 1, boost::bind(&fusion_detector::cameraCallback, this, _1, i));

    miivii_detector_sub_[i] = nh_.subscribe<autoware_msgs::DetectedObjectArray>(detected_objects_vision[i].c_str(), 1, boost::bind(&fusion_detector::miivii_detector_Callback, this, _1, i));
  }
  //-----------------------------订阅topic相关----------------------------------

  //-----------------------------发布topic相关----------------------------------
  if(visualize_fusion){
    std::string   fusion_results_image;
    for(int i=0;i<camCount;i++)
    {
      fusion_results_image = "/fusion_detector/fusion_results_image_" +std::to_string(i);
      fusion_results_image_pub_[i] = it_.advertise(fusion_results_image.c_str(), 1);
    }
  }

  nh_.param("combined_objects_labels", pub_combined_objects_labels, std::string("/fusion_detector/bounding_boxess"));
  publisher_fused_text_    = nh_.advertise<visualization_msgs::MarkerArray>(pub_combined_objects_labels.c_str(), 1);

  //
  nh_.param<bool>("enbable_publish_points_clounds_without_detectObjs", enbable_publish_points_clounds_without_detectObjs, false);
  if(enbable_publish_points_clounds_without_detectObjs)
  {
    nh_.param("points_clounds_without_detectObjs", points_clounds_withour_detectObjs_, std::string("/points_clounds_without_detectObjs"));
    ROS_INFO("%s",points_clounds_withour_detectObjs_.c_str());
    _publisher_points_clounds_without_detectObjs = nh_.advertise<sensor_msgs::PointCloud2>(points_clounds_withour_detectObjs_.c_str(), 1);
  }


  nh_.param("bounding_boxes_topic", pub_jsk_boundingboxes_topic_, std::string("/fusion_detector/bounding_boxess"));
  ROS_INFO("%s",pub_jsk_boundingboxes_topic_.c_str());
  _pub_jsk_boundingboxes   = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(pub_jsk_boundingboxes_topic_.c_str(), 1);
  //-----------------------------发布topic相关----------------------------------

  ROS_INFO("MiiVii Fusion initial done");
}

void fusion_detector::fusion_lidar_cameras(const pcl::PointCloud<pcl::PointXYZ>::Ptr  incloud)
{
  if(m_downsample_cloud){
    downsample_incloud->points.clear();
    MiiViiFusion::Down_Sampling(incloud,downsample_incloud,1);
  }
  else
    downsample_incloud = incloud;

  if(visualize_fusion){
    points_to_image(downsample_incloud);
  }

  autoware_msgs::DetectedObjectArray fused_objects;
  fused_objects.objects.clear();
  fused_objects.header = lidar_msg_header;
  jsk_recognition_msgs::BoundingBoxArray boundingbox_array;

  std::vector<std::vector<MiiViiObject2d> >        allresult2d;
  std::vector<MiiViiCameraParameterWrap>           camerasParam;

  for(int i=0; i<camCount; i++)
  {
    MiiViiObject2d temp_result2d;
    std::vector<MiiViiObject2d> result2d;
    for (const auto &detected_object : detected_objects_msg_[i]->objects)
    {
      if(detected_object.label == "person" )
        temp_result2d.type = PERSON;
      else if(detected_object.label == "car" )
        temp_result2d.type = CAR;
      else if(detected_object.label == "truck" )
        temp_result2d.type = TRUCK;
      else if(detected_object.label == "bus" )
        temp_result2d.type = BUS;
      // else if(detected_object.label == "bicycle" )
      //   temp_result2d.type = BICYCLE;
      // else if(detected_object.label == "motorbike" )
      //   temp_result2d.type = MOTORBIKE;
      else
        temp_result2d.type = UNKOWN;

      temp_result2d.objectID      = detected_object.id;
      temp_result2d.object.x      = detected_object.x;
      temp_result2d.object.y      = detected_object.y;
      temp_result2d.object.width  = detected_object.width;
      temp_result2d.object.height = detected_object.height;
      temp_result2d.score         = detected_object.score;

      result2d.push_back(temp_result2d);
    }
    allresult2d.push_back(result2d);
    camerasParam.push_back(m_cameraParameter[i]);
  }
  autoware_msgs::DetectedObject fusion_detected_object;

  std::vector<MiiViiObject3d> outputresult;
  m_MiiViiFusion.Fuse_Lidar_MultiCameras( downsample_incloud,
                                              camerasParam,
                                              allresult2d,
                                              outputresult,
                                              1,//融合方法 0 最大聚类；1 中心最优聚类
                                              merge_threshold);

  for(int kkk=0;kkk<outputresult.size();kkk++)
  {
    if (outputresult[kkk].IsValid())
    {
      fusion_detected_object.header                 = lidar_msg_header;
      //detected_object.space_frame                 = out_cluster.header.frame_id;
      fusion_detected_object.score                  = outputresult[kkk].object2d.score;
      fusion_detected_object.label                  = outputresult[kkk].GetLabel();
      fusion_detected_object.id                     = outputresult[kkk].object2d.objectID;

      //fusion_detected_object.image_frame          = detected_object.image_frame;
      fusion_detected_object.x                      = outputresult[kkk].object2d.object.x;
      fusion_detected_object.y                      = outputresult[kkk].object2d.object.y;
      fusion_detected_object.width                  = outputresult[kkk].object2d.object.width;
      fusion_detected_object.height                 = outputresult[kkk].object2d.object.height;
      //fusion_detected_object.angle                = detected_object.angle;
      fusion_detected_object.id                     = outputresult[kkk].object2d.objectID;


      fusion_detected_object.pose.position.x        = outputresult[kkk].bounding_box_.position.x;
      fusion_detected_object.pose.position.y        = outputresult[kkk].bounding_box_.position.y;
      fusion_detected_object.pose.position.z        = outputresult[kkk].bounding_box_.position.z;

      fusion_detected_object.dimensions.x           = outputresult[kkk].bounding_box_.dimensions.x;
      fusion_detected_object.dimensions.y           = outputresult[kkk].bounding_box_.dimensions.y;
      fusion_detected_object.dimensions.z           = outputresult[kkk].bounding_box_.dimensions.z;

      sensor_msgs::PointCloud2 cloud_msg;

      pcl::toROSMsg(*(outputresult[kkk].pointcloud), cloud_msg);
      fusion_detected_object.pointcloud             = cloud_msg;

      //fusion_detected_object.convex_hull          = tempcloud_cluster.convex_hull;
      fusion_detected_object.valid                  = outputresult[kkk].valid_;

      fused_objects.objects.push_back(fusion_detected_object);

      jsk_recognition_msgs::BoundingBox bounding_box;
      bounding_box.pose.position.x        = outputresult[kkk].bounding_box_.position.x;
      bounding_box.pose.position.y        = outputresult[kkk].bounding_box_.position.y;
      bounding_box.pose.position.z        = outputresult[kkk].bounding_box_.position.z;

      bounding_box.dimensions.x  = outputresult[kkk].bounding_box_.dimensions.x;
      bounding_box.dimensions.y  = outputresult[kkk].bounding_box_.dimensions.y;
      bounding_box.dimensions.z  = outputresult[kkk].bounding_box_.dimensions.z;

      bounding_box.header = lidar_msg_header;
      bounding_box.label  = outputresult[kkk].object2d.objectID;
      bounding_box.value  = outputresult[kkk].object2d.score;

      if(bounding_box.dimensions.x <0.2
      || bounding_box.dimensions.y <0.2
      || bounding_box.dimensions.z<0.1 )
      {
        continue;
      }
      else
      {
        if(outputresult[kkk].GetLabel() == "person"  )
        {
          bounding_box.dimensions.x = 0.4;
          bounding_box.dimensions.y = 0.4;
        }
        boundingbox_array.boxes.push_back(bounding_box);
      }
    }
  }

  boundingbox_array.header = lidar_msg_header;
  publishBoundingBoxArray(&_pub_jsk_boundingboxes, boundingbox_array);

  visualization_msgs::MarkerArray           fused_objects_labels;
  marker_id_ = 0;

  fused_objects_labels = ObjectsToMarkers(fused_objects);

  fused_objects_labels.markers.insert(fused_objects_labels.markers.end(), fused_objects_labels.markers.begin(), fused_objects_labels.markers.end());

  publisher_fused_text_.publish(fused_objects_labels);

  if(enbable_publish_points_clounds_without_detectObjs){
    tempdetectObj_cloud_ptr->points.clear();
    m_MiiViiFusion.Remove_Objects_Pointclouds(incloud,outputresult, tempdetectObj_cloud_ptr );
    publishCloud(&_publisher_points_clounds_without_detectObjs, lidar_msg_header, tempdetectObj_cloud_ptr);
  }
}

void fusion_detector::points_to_image( const pcl::PointCloud<pcl::PointXYZ>::Ptr  in_origin_cloud_ptr)
{
  for(int tempCameraId=0; tempCameraId<camCount; tempCameraId++)
  {
    if (InImage[tempCameraId]->empty()  && !bGettedImage[tempCameraId] )
      continue;
    std::vector<MiiViiObject2d> result2d;
    MiiViiObject2d temp_result2d;
    for (const auto &detected_object : detected_objects_msg_[tempCameraId]->objects)
    {
      if( detected_object.label == "person" ||
          detected_object.label == "car"    ||
          detected_object.label == "truck"  ||
          detected_object.label == "bus"    //||
          //detected_object.label == "bicycle"||
          //detected_object.label == "motorbike"
        )
      {

        if(detected_object.label == "person" )
          temp_result2d.type = PERSON;
        else if(detected_object.label == "car" )
          temp_result2d.type = CAR;
        else if(detected_object.label == "truck" )
          temp_result2d.type = TRUCK;
        else if(detected_object.label == "bus" )
          temp_result2d.type = BUS;
        // else if(detected_object.label == "bicycle" )
        //   temp_result2d.type = BICYCLE;
        // else if(detected_object.label == "motorbike" )
        //   temp_result2d.type = MOTORBIKE;
        else
          temp_result2d.type = UNKOWN;

        temp_result2d.objectID      = detected_object.id;
        temp_result2d.object.x      = detected_object.x;
        temp_result2d.object.y      = detected_object.y;
        temp_result2d.object.width  = detected_object.width;
        temp_result2d.object.height = detected_object.height;
        temp_result2d.score         = detected_object.score;

        result2d.push_back(temp_result2d);
      }
    }
    cv::Mat fusion_resultImage;
    if(!bGetingImage[tempCameraId]){
      cv_bridge::CvImage out_msg;
      out_msg.header = lidar_msg_header;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      if(result2d.size()>0){
        m_MiiViiFusion.Fuse_Lidar_Camera_to_image(in_origin_cloud_ptr,
                                                              m_cameraParameter[tempCameraId],
                                                              result2d,
                                                              *InImage[tempCameraId],
                                                              fusion_resultImage);
        DrawRects  rects_drawer;
        rects_drawer.DrawImageRect(result2d, fusion_resultImage,2);
        out_msg.image = fusion_resultImage.clone();
      }
      else{
        out_msg.image = InImage[tempCameraId]->clone();
      }
      fusion_results_image_pub_[tempCameraId].publish(out_msg.toImageMsg());
    }
  }
}

void fusion_detector::publishBoundingBoxArray(
    const ros::Publisher* in_publisher,
    const jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array)
{
  in_publisher->publish(in_boundingbox_array);
}

void fusion_detector::cameraCallback(const sensor_msgs::ImageConstPtr& msg, int cameraID)
{
  if (msg == NULL)
    return;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    bGetingImage[cameraID] = true;
    *InImage[cameraID] = cv_ptr->image.clone();
    bGettedImage[cameraID] = true;
    bGetingImage[cameraID] = false;
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception is %s", e.what());
      return;
  }
}

void fusion_detector::publishCloud(
      const ros::Publisher *in_publisher,
      const std_msgs::Header& in_header,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = in_header;
  in_publisher->publish(cloud_msg);
}

void fusion_detector::lidarCallback(
    const sensor_msgs::PointCloud2ConstPtr & msg)
{
  if (msg == NULL)
    return;
  lidar_msg_header = msg->header;

  lidar_processing_ = true;
  double time;
  if (debug_time)
    time = what_time_is_it_now();

  pcl::fromROSMsg (*msg, *in_point_clouds);
  fusion_lidar_cameras(in_point_clouds);
  lidar_processing_ = false;

  if (debug_time)
    ROS_INFO("miivii fusion cost %f ms\n", (what_time_is_it_now() - time) * 1000);
}

void fusion_detector::miivii_detector_Callback(
    const autoware_msgs::DetectedObjectArray::ConstPtr &msg, int cameraID)
{
  if (msg == NULL)
    return;
  detected_objects_msg_[cameraID] = msg;
}

visualization_msgs::MarkerArray fusion_detector::ObjectsToMarkers(
    const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray markerArray;

  for(const autoware_msgs::DetectedObject& object : in_objects.objects)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar_3d";
    marker.lifetime = ros::Duration(0.1);
    marker.header = lidar_msg_header;
    marker.ns = "";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 1.0;

    marker.id = marker_id_++;

    if(!object.label.empty() && object.label != "unknown")
      marker.text = object.label + " ";

    //if (object.id != 0)
    //{
    //  marker.text += " " + std::to_string(object.id)+"  ";
   // }

    std::stringstream distance_stream;
    distance_stream << std::fixed << std::setprecision(1)
                    << sqrt((object.pose.position.x * object.pose.position.x) +
                            (object.pose.position.y * object.pose.position.y));
    std::string distance_str = distance_stream.str() + "m";

    marker.text += distance_str;
    marker.pose.position.x = object.pose.position.x;
    marker.pose.position.y = object.pose.position.y;
    marker.pose.position.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.5;

    if (!marker.text.empty())
    {
      markerArray.markers.push_back(marker);
    }
  }
  return markerArray;
}
