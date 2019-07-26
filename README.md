# MiiVii Fusion

MiiVii Fusion Solution is the solution familiy base on hardware S2 Pro and Apex.

## Demo Video

## MiiVii Fusion Unit

MiiVii Fusion Unit is a series of modulized hardware for fusion fast deployment. It's designed for various robotics usecases, and carefully calibrated. It can be used as a basic robot perception unit.

| Profile      | Apperance     | Camera Detail     | IMU     | Coverage     | Size     |
| ---------- | :-----------:  | :-----------:  | :-----------:  | :-----------:  | :-----------:  |
| Profile_Standard     | <img src="images/fusion_unit.png" width="100"> |120° x 4, 60° x 1| (optional)| 360°  |160mm x 160mm x 90mm |
| Profile_MarmorF    | <img src="images/fusion_unit.png" width="100"> | 120° x 2, 60° x 1, 190° x 1      | (optional)  | 360°  |160mm x 160mm x 90mm |
| Profile_120X2    | <img src="images/fusion_unit.png" width="100"> | 120° x 2      |(optional)| 240°  |160mm x 160mm x 90mm |
| Profile_120X3    | <img src="images/fusion_unit.png" width="100"> | 120° x 3      |(optional)| ~ 300°  |160mm x 160mm x 90mm |
| Profile_60X8    | Coming soon | 60° x 8      |(optional)| 360°  |160mm x 160mm x 90mm |

## MiiVii Fusion Detector User Guide

The MiiVii GMSL Camera contains only one ROS package miivii_gmsl_ros for now.

### Compile
Clone the repository and build:
```
    mkdir -p ~/catkin_ws/src & cd ~/catkin_ws/src
    git clone https://github.com/MiiViiDynamics/miivii_fusion
    cd ../..
    catkin_make_isolated
    source devel_isolated/setup.bash
```

### Run

### Visualize

## Synchronization Comparation
### see more result [here](docs/synchronization.md)
| Sensor      | Jitter Compare<br/>not sync(red), MArmorF data(blue)     | Sensor      | Jitter Compare<br/>not sync(red), MArmorF data(blue)|
| ----------  | :-----------:  | :-----------:  | :-----------:  |
| IMU<br/>(need imu support)     | <img src="https://github.com/MiiViiDynamics/marmorf/raw/master/images/sync/imu_marmorf_vs_unsync.png" width="400">      |3d Lidar|<img src="https://github.com/MiiViiDynamics/marmorf/raw/master/images/sync/lidar_marmorf_vs_unsync.png" width="400">|
| Camera<br/>     | <img src="https://github.com/MiiViiDynamics/marmorf/raw/master/images/sync/camera_marmorf_vs_unsync.png" width="400">      |||


## Contact
For technology issue, please file bugs on github directly.
For busniess contact, you can either 
1. visit our [taobao shop](https://shop324175547.taobao.com/?spm=a230r.7195193.1997079397.2.3154636cYGG7Vj)
2. mail to bd#miivii.com