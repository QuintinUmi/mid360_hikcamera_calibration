# mid360_hikcamera_calibration

## This pack is still constructing...


### mid360 5hz publish

<p align="center">
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/main/doc/img/lidar_camera_cal1_compressed.gif" alt="lidar_camera_cal1_compressed.gif"/>
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/main/doc/img/lidar_camera_cal2_compressed.gif" alt="lidar_camera_cal2_compressed.gif"/>
</p>

### mid360 2hz publish
<p align="center">
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/main/doc/img/lidar_camera_cal3_compressed.gif" alt="lidar_camera_cal3_compressed.gif"/>
        <img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/main/doc/img/lidar_camera_cal4_compressed.gif" alt="lidar_camera_cal4_compressed.gif"/>
</p>


<img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/main/doc/img/lidar_camera_cal1_compressed.gif" alt="lidar_camera_cal1_compressed.gif"/>

<img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/main/doc/img/cal2.gif" alt="caliborad_detection.gif"/>

<img src="https://github.com/QuintinUmi/mid360_hikcamera_calibration/blob/QuintinUmi/doc/img/caliborad_detection.png" alt="caliborad_detection.gif"/>


        


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
    │   └── img
    │       ├── caliboard_cloud_detection_2.png
    │       ├── caliboard_cloud_detection_detailed_1.png
    │       ├── caliboard_cloud_detection_detailed_2.png
    │       ├── caliboard_cloud_detection.png
    │       ├── caliborad_detection.png
    │       ├── lidar_caliboard_detect_demo_10hz_1.gif
    │       ├── lidar_caliboard_detect_demo_10hz_2.gif
    │       ├── lidar_caliboard_detect_demo_5hz_1.gif
    │       ├── lidar_caliboard_detect_demo_5hz_2.gif
    │       ├── lidar_caliboard_detect_demo_5hz_3.gif
    │       ├── readme.txt
    │       └── rviz_caliboard_cloud_collection.png
    ├── README.md
    └── src
        ├── livox_hikcamera_cal
        │   ├── aruco_img
        │   │   ├── id_150--dictName_10.png
        │   │   ├── id_230--dictName_10.png
        │   │   ├── id_4--dictName_10.png
        │   │   └── id_80--dictName_10.png
        │   ├── bag
        │   ├── cfg
        │   │   ├── calibration_param.yaml
        │   │   ├── PointcloudFilter.cfg
        │   │   ├── setup_aruco.yaml
        │   │   └── setup_ros.yaml
        │   ├── CMakeLists.txt
        │   ├── include
        │   │   └── livox_hikcamera_cal
        │   │       ├── calibration_solver.h
        │   │       ├── dynamic_reconfigure.h
        │   │       ├── image_opr
        │   │       │   ├── aruco_tool.h
        │   │       │   └── drawing_tool.h
        │   │       ├── pointcloud2_opr
        │   │       │   ├── pcd_saver.h
        │   │       │   ├── point_cloud_process.h
        │   │       │   └── point_cloud_subscriber_publisher.h
        │   │       ├── recorder.h
        │   │       └── rviz_drawing.h
        │   ├── launch
        │   │   └── aruco_video_ext_calib.launch
        │   ├── lib
        │   │   ├── calibration_solver.cpp
        │   │   ├── dynamic_reconfigure.cpp
        │   │   ├── image_opr
        │   │   │   ├── aruco_tool.cpp
        │   │   │   └── drawing_tool.cpp
        │   │   ├── pointcloud2_opr
        │   │   │   ├── pcd_saver.cpp
        │   │   │   ├── point_cloud_process.cpp
        │   │   │   └── point_cloud_subscriber_publisher.cpp
        │   │   ├── recorder.cpp
        │   │   └── rviz_drawing.cpp
        │   ├── miscellaneous
        │   │   └── 90885055.jpeg
        │   ├── package.xml
        │   ├── pcd
        │   │   ├── 20240509061323.pcd
        │   │   ├── 20240509061325.pcd
        │   │   ├── 20240509061326.pcd
        │   │   ├── 20240509061327.pcd
        │   │   ├── 20240509061328.pcd
        │   │   ├── 20240509061333.pcd
        │   │   ├── 20240509061340.pcd
        │   │   ├── 20240509061346.pcd
        │   │   ├── 20240509061349.pcd
        │   │   ├── 20240509061354.pcd
        │   │   └── 20240509061358.pcd
        │   └── src
        │       ├── aruco_detection_example.cpp
        │       ├── caliboard_cloud_detection.cpp
        │       ├── caliboard_cloud_detection_dynamic.cpp
        │       ├── pcd_saver_example.cpp
        │       ├── point_cloud_filter.cpp
        │       ├── rosbag_record_example.cpp
        │       ├── test.cpp
        │       ├── test_outer2.cpp
        │       └── test_outer.cpp
        └── readme.md


```
