# mid360_hikcamera_calibration

## This pack is still constructing...

### mid360 10hz publish
<p align="center">
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/lidar_caliboard_detect_demo_10hz_1.gif" alt="lidar_caliboard_detect_demo_10hz_1.gif"/>
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/lidar_caliboard_detect_demo_10hz_2.gif" alt="lidar_caliboard_detect_demo_10hz_1.gif"/>
</p>

### mid360 5hz publish
<p align="center">
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/lidar_caliboard_detect_demo_5hz_1.gif" alt="lidar_caliboard_detect_demo_10hz_1.gif"/>
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/lidar_caliboard_detect_demo_5hz_2.gif" alt="lidar_caliboard_detect_demo_10hz_1.gif"/>
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/lidar_caliboard_detect_demo_5hz_3.gif" alt="lidar_caliboard_detect_demo_10hz_1.gif"/>
</p>
        


<p align="center">
        <a href="https://github.com/QuintinUmi/mid360_hikcamera_calibration/">
            <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/rviz_caliboard_cloud_collection.png?raw=true" alt="rviz_caliboard_cloud_collection.png"/>
            <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/caliboard_cloud_detection.png?raw=true" alt="caliboard_cloud_detection.png"/>
            <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/caliboard_cloud_detection_2.png?raw=true" alt="caliboard_cloud_detection_2.png"/>
                <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/caliboard_cloud_detection_detailed_1.png?raw=true" alt="caliboard_cloud_detection_2.png"/>
                <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/caliboard_cloud_detection_detailed_2.png?raw=true" alt="caliboard_cloud_detection_2.png"/>
        </a>
</p>

## Files Structure Description

```
mid360_hikcamera_calibration
    ├── doc
    │   └── img
    │       ├── caliboard_cloud_detection_2.png
    │       ├── caliboard_cloud_detection_detailed_1.png
    │       ├── caliboard_cloud_detection_detailed_2.png
    │       ├── caliboard_cloud_detection.png
    │       ├── readme.txt
    │       └── rviz_caliboard_cloud_collection.png
    ├── myRemoval.pcd
    ├── README.md
    └── src
        ├── livox_hikcamera_cal
        │   ├── bag
        │   │   └── 20240509061321
        │   ├── cfg
        │   │   └── PointcloudFilter.cfg
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── livox_hikcamera_cal
        │   │       ├── pointcloud2_opr
        │   │       │   ├── dynamic_reconfigure.h
        │   │       │   ├── pcd_saver.h
        │   │       │   ├── point_cloud_process.h
        │   │       │   └── point_cloud_subscriber_publisher.h
        │   │       └── recorder.h
        │   ├── launch
        │   ├── lib
        │   │   ├── dynamic_reconfigure.cpp
        │   │   ├── pcd_saver.cpp
        │   │   ├── point_cloud_process.cpp
        │   │   ├── point_cloud_subscriber_publisher.cpp
        │   │   └── recorder.cpp
        │   ├── package.xml
        │   ├── pcd
        │   │   ├── 20240509061323.pcd
        │   │   ├── 20240509061325.pcd
        │   │   ├── 20240509061326.pcd
        │   │   ├── 20240509061327.pcd
        │   │   ├── 20240509061328.pcd
        │   │   ├── 20240509061333.pcd
        │   │   ├── 20240509061340.pcd
        │   │   ├── 20240509061346.pcd
        │   │   ├── 20240509061349.pcd
        │   │   ├── 20240509061354.pcd
        │   │   └── 20240509061358.pcd
        │   └── src
        │       ├── caliboard_cloud_detection.cpp
        │       ├── point_cloud_filter.cpp
        │       ├── rosbag_record_example.cpp
        │       ├── test.cpp
        │       ├── test_outer2.cpp
        │       └── test_outer.cpp
        └── readme.md


```
