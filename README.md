# mid360_hikcamera_calibration

## Files Structure Description

```
livox_pointcloud2_opr
│   ├── bag
│   │   └── 20240509061321
│   ├── cfg
│   │   └── PointcloudFilter.cfg
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── livox_pc2_opr
│   │   │   ├── dynamic_reconfigure.h
│   │   │   ├── pcd_saver.h
│   │   │   ├── point_cloud_process.h
│   │   │   ├── point_cloud_subscriber_publisher.h
│   │   │   └── recorder.h
│   │   └── rectangular_plane_fitting.h
│   ├── launch
│   ├── lib
│   │   ├── dynamic_reconfigure.cpp
│   │   ├── pcd_saver.cpp
│   │   ├── point_cloud_process.cpp
│   │   ├── point_cloud_subscriber_publisher.cpp
│   │   ├── recorder.cpp
│   │   └── rectangular_plane_fitting.cpp
│   ├── package.xml
│   ├── pcd
│   │   ├── 20240509061322.pcd
│   │   ├── 20240509061323.pcd
│   │   ├── 20240509061331.pcd
│   │   ├── 20240509061332.pcd
│   │   ├── 20240509061333.pcd
│   │   ├── 20240509061334.pcd
│   │   ├── 20240509061335.pcd
│   │   ├── 20240509061336.pcd
│   │   ├── 20240509061337.pcd
│   │   ├── 20240509061338.pcd
│   │   ├── 20240509061340.pcd
│   │   ├── 20240509061349.pcd
│   │   ├── 20240509061350.pcd
│   │   └── 20240509061351.pcd
│   └── src
│       ├── plane_gain.cpp
│       ├── point_cloud_filter.cpp
│       ├── rosbag_record_example.cpp
│       ├── test.cpp
│       ├── test_outer1.cpp
│       └── test_outer.cpp

```
