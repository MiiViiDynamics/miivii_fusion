FUSION DETECTOR ROS Node

本节点功能为：相机和Lidar数据融合目标检测。

[1、编译]

mkdir -p [Path of your directory]/catkin_ws/src  #建立工程目录
cp -r [path of miivii_gmsl_ros] [Path of your directory]/catkin_ws/src  #将 miivii_gmsl_ros 拷贝到src目录中
cp -r [path of lidar_ros_node] [Path of your directory]/catkin_ws/src   #将 lidar_ros_node  拷贝到src目录中
cp -r [path of fusion_detector] [Path of your directory]/catkin_ws/src  #将 fusion_detector 拷贝到src目录中
cp -r [path of miivii_detector] [Path of your directory]/catkin_ws/src  #将 miivii_detector 拷贝到src目录中
cd [Path of your directory]/catkin_ws
catkin_make

[2、需要条件]

必须条件
---需要预先获取lidar和camera联合标定结果，即获取内外参数

可选条件：
  为了获得更好的识别效果，如果在Apex平台上运行，可以运行如下指令：

  ```
  sudo jetson_clocks
  ```
[3、启动步骤]

  第1步、启动相机节点, GMSL相机节点启动，请参阅ros_miivii_gmsl的readme

  第2步、新打开一个终端，启动 lidar节点；

  第3步、新打开一个终端，启动 miivii_detector 检测节点；

    ```
    cd [Path of your directory]/catkin_ws
    source devel/setup.bash
    roslaunch miivii_detector miivii_detector.launch
    ```
  第4步、启动fusion_detector节点;

    ```
    cd [Path of your directory]/catkin_ws
    source devel/setup.bash
    roslaunch fusion_detector test.launch
    ```
  第5步：启动rviz

    ```
    cd [Path of your directory]/catkin_ws
    source devel/setup.bash
    rosrun rviz rviz
    ```
    启动rviz后，
    ---添加MarkerArray,将其topic设置为"detection/combined_objects_labels"，显示识别目标的labels
    ---添加BoundingBoxArray，将其topic设置为：”/detection/lidar_detector/bounding_boxes“，用于显示目标的box位置


[4、launch文件配置]

4.1 miivii_detector launch文件.
  修改”camera_topic“参数为用到的相机topic

2.2 fusion detector launch文件配

  ---修改”camera_topic“参数为用到的相机topic，例如  "/usb_cam/image_raw"

  ---修改”calibration_file“参数为：标定文件存储位置，例如  "/home/nvidia/zhh/miivii_perception/src/detection/fusion_detector/calibration_result/autoware01.yml"；

  ---修改”lidar_topic“参数为使用的lidar的topic，例如 "/points_raw"；