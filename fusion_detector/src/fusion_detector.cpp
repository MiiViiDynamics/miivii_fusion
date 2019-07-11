#include "fusion_detector.h"
#include <unistd.h>

#define USE_DEBUG 1
#if USE_DEBUG
#define DEBUG_PRINT \
  ROS_INFO("<MIIVII-DEBUG> [%s] %d (%s)", __FUNCTION__, __LINE__, __FILE__);
#endif

double what_time_is_it_now() {
  struct timeval time;
  if (gettimeofday(&time, NULL)) {
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void fusion_detector::Initialize_Camera_lidar(){
  for(int kk=0; kk<CAMERA_COUNT; kk++)
  {
    detected_objects_msg_[kk] = nullptr;
    detected_objects_msg_[kk] = boost::make_shared<autoware_msgs::DetectedObjectArray>();

    draw_fusion_pts_[kk] = false;
  }

  lidar_processing_ = false;
  image_transport::ImageTransport it_(nh_);

  nh_.param("camCount", camCount_, 4);
  camCount = camCount_;
  if (camCount > 4 || camCount < 0) {
    camCount = 1;
  }

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

    image_sub_[i] = it_.subscribe(camera_topic_name_[i].c_str(), 1, boost::bind(&fusion_detector::cameraCallback, this, _1, i));

    yolo_detect_sub_[i] = nh_.subscribe<autoware_msgs::DetectedObjectArray>(detected_objects_vision[i].c_str(), 1, boost::bind(&fusion_detector::yolo_detect_Callback, this, _1, i));

    std::string   fusion_results_image;
    fusion_results_image = "fusion_results_image_" +std::to_string(i);
    fusion_results_image_pub_[i] = it_.advertise(fusion_results_image.c_str(), 1);

    InImage[i] = new cv::Mat();
    InImage[i]->create(m_cameraParameter[i].imageSize.width, m_cameraParameter[i].imageSize.height, CV_8UC3);
  }

  nh_.param("lidar_topic", lidar_topic_name_, std::string("/lidar_points"));
  ROS_INFO("%s",lidar_topic_name_.c_str());
  lidar_sub_ = nh_.subscribe(lidar_topic_name_.c_str(), 1, &fusion_detector::lidarCallback,this);

  std::string fused_text_str  = "/detection/combined_objects_labels";
  publisher_fused_text_    = nh_.advertise<visualization_msgs::MarkerArray>(fused_text_str.c_str(), 1);

  // debug
  nh_.param<bool>("debug_time", debug_time, false);

  ROS_INFO("cluster initial start");

  nh_.param<bool>("visualize_fusion", enable_show_results_window_, false);


#if (CV_MAJOR_VERSION == 3)
    generateColors(_colors, 255);
#else
    cv::generateColors(_colors, 255);
#endif

  nh_.param("pub_jsk_boundingboxes_topic", pub_jsk_boundingboxes_topic_, std::string("/detection/lidar_detector/bounding_boxess"));
  ROS_INFO("%s",pub_jsk_boundingboxes_topic_.c_str());

  nh_.param("pose_estimation", _pose_estimation, false);
  ROS_INFO("[%s] pose_estimation: %d", __APP_NAME__, _pose_estimation);

  nh_.param("use_gpu", _use_gpu, false);
  ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, _use_gpu);

  _pub_jsk_boundingboxes   = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(pub_jsk_boundingboxes_topic_.c_str(), 1);

  ROS_INFO("MiiVii Fusion initial done");
}

void fusion_detector::fusion_lidar_cameras(sensor_msgs::PointCloud2ConstPtr LiDarData)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*LiDarData, *incloud);
  MiiViiObject2d temp_result2d;

  int k=0;

  autoware_msgs::DetectedObjectArray fused_objects;
  fused_objects.objects.clear();
  fused_objects.header = lidar_msg_header;
  autoware_msgs::Centroids centroids;
  jsk_recognition_msgs::BoundingBoxArray boundingbox_array;
  std::vector<ClusterPtr> final_clusters;

  for(int tempCameraId=0; tempCameraId<camCount; tempCameraId++)
  {
    std::vector<MiiViiObject2d> result2d;

    autoware_msgs::DetectedObject fusion_detected_object;
    for (const auto &detected_object : detected_objects_msg_[tempCameraId]->objects)
    {
      //temp_result2d.label = detected_object.label;

      if(detected_object.label == "person" )
        temp_result2d.type = PERSON;
      else if(detected_object.label == "car" )
        temp_result2d.type = CAR;
      else if(detected_object.label == "truck" )
        temp_result2d.type = TRUCK;
      else if(detected_object.label == "bus" )
        temp_result2d.type = BUS;
      else if(detected_object.label == "bicycle" )
        temp_result2d.type = BICYCLE;
      else if(detected_object.label == "motorbike" )
        temp_result2d.type = MOTORBIKE;
      else
      {
        temp_result2d.type = UNKOWN;
      }
      temp_result2d.objectID = detected_object.id;

      temp_result2d.object.x      = detected_object.x;
      temp_result2d.object.y      = detected_object.y;
      temp_result2d.object.width  = detected_object.width;
      temp_result2d.object.height = detected_object.height;
      temp_result2d.score         = detected_object.score;

      result2d.push_back(temp_result2d);
    }

    //ROS_INFO("%d detected_objects_msg_:%d result2d objectsNum:%d",  __LINE__, tempCameraId, result2d.size());

    std::vector<MiiViiObject3d> outputresult;
    m_MiiViiFusion[tempCameraId].Fuse_Lidar_Camera( incloud,  m_cameraParameter[tempCameraId],
                                      result2d,    outputresult );
    //ROS_INFO("[%s]  %d outputresult objectsNum:%d", __APP_NAME__, __LINE__,outputresult.size());

    for(int kkk=0;kkk<outputresult.size();kkk++)
    {
      ClusterPtr out_cluster(new Cluster());
      std::string in_label;
      if(outputresult[kkk].object2d.type == PERSON  )
        in_label = "person";
      else if(outputresult[kkk].object2d.type == CAR)
        in_label = "car";
      else if(outputresult[kkk].object2d.type == TRUCK)
        in_label = "truck";
      else if(outputresult[kkk].object2d.type == BUS)
        in_label = "bus";
      else if(outputresult[kkk].object2d.type == BICYCLE  )
        in_label = "bicycle";
      else if(outputresult[kkk].object2d.type == MOTORBIKE)
        in_label = "motorbike";
      else
        in_label = "unknow";

      //ROS_INFO("[%s]  %d ", __APP_NAME__, __LINE__);
      out_cluster->SetCloud_z(outputresult[kkk].object,
                      lidar_msg_header,
                      k,   //id
                      (int)_colors[k].val[0],
                      (int)_colors[k].val[1],
                      (int)_colors[k].val[2],
                      in_label,
                      _pose_estimation);
      //ROS_INFO("[%s]  %d ", __APP_NAME__, __LINE__);
      k++;
      if(out_cluster!=NULL)
      {
        final_clusters.push_back(out_cluster);

        //detected_object.space_frame = out_cluster.header.frame_id;
        fusion_detected_object.score       = outputresult[kkk].object2d.score;
        fusion_detected_object.label       = in_label;//outputresult[kkk].object2d.label;
        fusion_detected_object.id          = outputresult[kkk].object2d.objectID;

        //fusion_detected_object.image_frame = detected_object.image_frame;
        fusion_detected_object.x           = outputresult[kkk].object2d.object.x;
        fusion_detected_object.y           = outputresult[kkk].object2d.object.y;
        fusion_detected_object.width       = outputresult[kkk].object2d.object.width;
        fusion_detected_object.height      = outputresult[kkk].object2d.object.height;
        //fusion_detected_object.angle       = detected_object.angle;
        //fusion_detected_object.id          = detected_object.id;

        autoware_msgs::CloudCluster tempcloud_cluster;

        out_cluster->ToROSMessage(lidar_msg_header, tempcloud_cluster);

        fusion_detected_object.header      = tempcloud_cluster.header;
        fusion_detected_object.pose        = tempcloud_cluster.bounding_box.pose;
        fusion_detected_object.dimensions  = tempcloud_cluster.dimensions;
        fusion_detected_object.pointcloud  = tempcloud_cluster.cloud;
        fusion_detected_object.convex_hull = tempcloud_cluster.convex_hull;
        fusion_detected_object.valid       = true;
        fused_objects.objects.push_back(fusion_detected_object);
      }
    }
  }
  //ROS_INFO("[%s]  %d  %d", __APP_NAME__, __LINE__, final_clusters.size());

  for (unsigned int i = 0; i < final_clusters.size(); i++)
  {
    jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();

    bounding_box.header = lidar_msg_header;
    if (final_clusters[i]->IsValid())
    {
      boundingbox_array.boxes.push_back(bounding_box);

    }
  }


  boundingbox_array.header = lidar_msg_header;
  publishBoundingBoxArray(&_pub_jsk_boundingboxes, boundingbox_array);

  visualization_msgs::MarkerArray           fused_objects_labels;
  marker_id_ = 0;

  fused_objects_labels = ObjectsToMarkers(fused_objects);

  fused_objects_labels.markers.insert(fused_objects_labels.markers.end(),
                                      fused_objects_labels.markers.begin(), fused_objects_labels.markers.end());

  publisher_fused_text_.publish(fused_objects_labels);
}

void fusion_detector::points_to_image_new( sensor_msgs::PointCloud2ConstPtr LiDarData)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*LiDarData, *in_origin_cloud_ptr);

  for(int tempCameraId=0; tempCameraId<camCount; tempCameraId++)
  {
    std::vector<MiiViiObject2d> result2d;
    MiiViiObject2d temp_result2d;
    for (const auto &detected_object : detected_objects_msg_[tempCameraId]->objects)
    {
      if( detected_object.label == "person" || detected_object.label == "car"    ||
          detected_object.label == "truck"  || detected_object.label == "bus"    ||
          detected_object.label == "bicycle"  || detected_object.label == "motorbike"       )   {

        if(detected_object.label == "person" )
          temp_result2d.type = PERSON;
        else if(detected_object.label == "car" )
          temp_result2d.type = CAR;
        else if(detected_object.label == "truck" )
          temp_result2d.type = TRUCK;
        else if(detected_object.label == "bus" )
          temp_result2d.type = BUS;
        else if(detected_object.label == "bicycle" )
          temp_result2d.type = BICYCLE;
        else if(detected_object.label == "motorbike" )
          temp_result2d.type = MOTORBIKE;
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
      m_MiiViiFusion[tempCameraId].Fuse_Lidar_Camera_to_image(in_origin_cloud_ptr,
                                                              m_cameraParameter[tempCameraId],
                                                              result2d,
                                                              *InImage[tempCameraId],
                                                              fusion_resultImage);

      DrawRects  rects_drawer;
      rects_drawer.DrawImageRect(result2d, fusion_resultImage,2);

      cv_bridge::CvImage out_msg;
      out_msg.header = LiDarData->header;
      //out_msg.header.stamp = msg->header.stamp;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = fusion_resultImage;

      fusion_results_image_pub_[tempCameraId].publish(out_msg.toImageMsg());
      if(enable_show_results_window_)
      {
        std::string    window_name;
        window_name = "fusion_results" +std::to_string(tempCameraId);
        cv::imshow(window_name.c_str(),fusion_resultImage);
        cv::waitKey(30);
      }
    }
  }
}

void fusion_detector::publishBoundingBoxArray(const ros::Publisher* in_publisher,
                                              const jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array)
{
  in_publisher->publish(in_boundingbox_array);
}

void fusion_detector::cameraCallback(const sensor_msgs::ImageConstPtr& msg, int cameraID)
{
  int tempCameraId = cameraID;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    bGetingImage[tempCameraId] = true;
    *InImage[tempCameraId] = cv_ptr->image.clone();
    bGetingImage[tempCameraId] = false;
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception is %s", e.what());
      return;
  }
}

void fusion_detector::lidarCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
  if (msg == NULL)
    return;
  lidar_msg_header = msg->header;
  if(!draw_fusion_pts_[0] && !draw_fusion_pts_[1] && !draw_fusion_pts_[2]  && !draw_fusion_pts_[3])
  {
    lidar_processing_ = true;

    double time;
    if (debug_time)
      time = what_time_is_it_now();
    points_to_image_new(msg);

    fusion_lidar_cameras(msg);
    lidar_processing_ = false;

    if (debug_time)
      ROS_INFO("infer cost %f ms\n", (what_time_is_it_now() - time) * 1000);
  }
}

void fusion_detector::yolo_detect_Callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg, int cameraID)
{
  if (msg == NULL)
    return;
  detected_objects_msg_[cameraID] = msg;
}

visualization_msgs::MarkerArray fusion_detector::ObjectsToMarkers( const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray markerArray;

  for(const autoware_msgs::DetectedObject& object : in_objects.objects)
  {
    {
      visualization_msgs::Marker marker;

      marker.lifetime = ros::Duration(0.1);
      marker.header = in_objects.header;
      marker.ns = "/label_markers";
      marker.action = visualization_msgs::Marker::ADD;
      marker.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 1.0;

      marker.id = marker_id_++;
      if (object.id != 0)
      {
        marker.text += " " + std::to_string(object.id);
      }

      if(!object.label.empty() && object.label != "unknown")
        marker.text = object.label + " ";

      std::stringstream distance_stream;
      distance_stream << std::fixed << std::setprecision(2)
                      << sqrt((object.pose.position.x * object.pose.position.x) +
                              (object.pose.position.y * object.pose.position.y));
      std::string distance_str = distance_stream.str() + " m";

      marker.text += distance_str;
      marker.pose.position.x = object.pose.position.x;
      marker.pose.position.y = object.pose.position.y;
      marker.pose.position.z = 1.0;

      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;

      marker.scale.x = 0.7;//1.5;
      marker.scale.y = 0.7;//1.5;
      marker.scale.z = 0.5;//1.0;

      if (!marker.text.empty())
      {
        markerArray.markers.push_back(marker);
      }
    }
  }
  return markerArray;
}

jsk_recognition_msgs::BoundingBoxArray fusion_detector::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
  jsk_recognition_msgs::BoundingBoxArray final_boxes;
  final_boxes.header = in_objects.header;

  for(const autoware_msgs::DetectedObject& object : in_objects.objects)
  {
    jsk_recognition_msgs::BoundingBox box;

    box.header     = in_objects.header;
    box.label      = object.id;
    box.dimensions = object.dimensions;
    box.pose       = object.pose;
    box.value      = object.score;

    if (box.dimensions.x > 0 && box.dimensions.y > 0 && box.dimensions.z > 0)
    {
      final_boxes.boxes.push_back(box);
    }
  }
  return final_boxes;
}


