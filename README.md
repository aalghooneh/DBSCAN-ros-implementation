# DBSCAN_ROS

A high-performance **Adaptive DBSCAN clustering** ROS node that segments point clouds into clusters for downstream perception tasks. It processes a filtered LiDAR/RADAR point cloud, runs adaptive DBSCAN clustering, and outputs:

1. **Clustered Point Clouds** (via `autoware_perception_msgs::DynamicObjectWithFeatureArray`)
2. **Markers** visualizing bounding boxes (via `visualization_msgs::MarkerArray`)
3. **Noise Points** (points not belonging to any cluster)

This node is especially useful for 3D LiDAR-based perception in autonomous driving or robotics systems, where you need to:
- Identify objects in point cloud data,
- Publish bounding boxes for visualization,
- And keep track of any points that do not belong to relevant objects.

## Table of Contents

1. [Features](#features)
2. [Dependencies](#dependencies)
3. [Installation and Build Instructions](#installation-and-build-instructions)
4. [Usage](#usage)
5. [Node Parameters](#node-parameters)
6. [Key Parts of the Code](#key-parts-of-the-code)
7. [Performance](#performance)
8. [Contribution](#contribution)
9. [License](#license)

## Dependencies

This package depends on common ROS and perception libraries:
- **ROS** (tested with ROS melodic/noetic)
- **PCL** (Point Cloud Library)
- **sensor_msgs**
- **visualization_msgs**
- **autoware_perception_msgs** (from Autoware, or your custom message definitions)

Ensure you have these dependencies in your `package.xml`:

```xml
<depend>roscpp</depend>
<depend>sensor_msgs</depend>
<depend>visualization_msgs</depend>
<depend>pcl_conversions</depend>
<depend>pcl_ros</depend>
<depend>autoware_perception_msgs</depend>
```

## Installation and Build Instructions

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/dbscan_ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

1. **Start ROS Master**:
   ```bash
   roscore
   ```
2. **Launch LiDAR input** (e.g., bag file, driver, or simulation)
3. **Run dbscan_ros**:
   ```bash
   rosrun dbscan_ros dbscan_ros_node
   ```
4. **Visualize outputs** in `rviz` with topics:
   - `/dbscan_clustering/marker` (bounding boxes)
   - `/dbscan_clustering/noise_points` (outliers)

## Node Parameters

| Parameter Name | Type | Default Value | Description |
|---------------|------|---------------|-------------|
| `topic_input` | string | `/rslidar_points_front/ROI/no_ground` | Input point cloud topic |
| `topic_output` | string | `/dbscan_clustering/clusters` | Output topic for clusters |
| `obj_width_thre` | double | `0.15` | Approximate object width threshold |
| `obj_height_thre` | double | `0.15` | Approximate object height threshold |
| `downsample_resolution` | double | `0.03` | Resolution for downsampling |
| `lidar_horizon_resolution` | double | `0.2` | LiDAR horizontal resolution (degrees) |
| `lidar_vertical_resolution` | double | `1.0` | LiDAR vertical resolution (degrees) |

## Key Parts of the Code

### 1. Parameter Initialization
```cpp
nh_private.param<std::string>("topic_input", topic_input_, "/rslidar_points_front/ROI/no_ground");
nh_private.param<double>("obj_width_thre", obj_width_thre_, 0.15);
```

### 2. Subscriber and Publishers
```cpp
pc_sub = nh.subscribe(topic_input_, 1, &dbscan_ros::lidar_callback, this);
cluster_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/dbscan_clustering/marker", 10);
```

### 3. The Clustering Process
```cpp
AdaptiveDBSCANCluster<pcl::PointXYZ> db;
db.setInputCloud(ground_remo);
db.setSearchMethod(tree);
db.setClusteringParams(obj_width_thre_, num_points_per_line_, max_num_lines_, eps_with_distance_factor_, num_line_with_distance_factor_);
db.setMinClusterSize(8);
db.setMaxClusterSize(20000);
db.extract(indices);
```

### 4. Bounding Box and Center Calculation
```cpp
for (pcl::PointIndices ind : indices) {
    // Compute min, max, center, and create markers
}
```

### 5. Noise Points Extraction
```cpp
pcl::PointIndices::Ptr all_cluster_indices_ptr(new pcl::PointIndices(all_cluster_indices));
ext.setIndices(all_cluster_indices_ptr);
pcl::PointCloud<pcl::PointXYZ> noise_cloud;
ext.setNegative(true);
ext.filter(noise_cloud);
```

### 6. Performance Measurement
```cpp
auto start = std::chrono::steady_clock::now();
auto end = std::chrono::steady_clock::now();
auto duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
if (duration_ >= 10 ) std::cout << "time took is, " << duration_ << std::endl;
```

## Performance

- **Real-time** or near real-time performance depending on:
  - CPU capabilities
  - Density of point cloud
  - Complexity of scene
- **Adaptive** factors keep clusters well-defined even with different LiDAR resolutions.

## Contribution

Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature-name`).
3. Commit your changes (`git commit -m 'Add new feature'`).
4. Push the branch (`git push origin feature-name`).
5. Open a pull request.

## License

This code is provided under [MIT](LICENSE). Please refer to the [LICENSE](LICENSE) file for details.

## Thank You

Enjoy using **dbscan_ros** for your LiDAR clustering needs! If you run into issues, feel free to open an issue or contribute via pull requests.

**Happy Clustering!**

