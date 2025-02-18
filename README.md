# DBSCAN ROS C++ &mdash; 

**Welcome to the ultimate clustering carnage.** This ROS package harnesses the power of DBSCAN (Density-Based Spatial Clustering of Applications with Noise) to obliterate unorganized data points in your 2D/3D space and group them into meaningful clusters. No more messing around—we're going full throttle. Buckle up.

---

## Table of Contents
1. [About the Package](#about-the-package)
2. [Features](#features)
3. [Dependencies](#dependencies)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Code Breakdown](#code-breakdown)
7. [Roadmap](#roadmap)
8. [Contributing](#contributing)
9. [License](#license)

---

## About the Package
This is a ROS (Robot Operating System) node written in C++ that implements the DBSCAN clustering algorithm. Whether you need to:
- Identify objects in LiDAR data.
- Segment point clouds from 3D sensors.
- Detect anomalies or noise in a dataset.

This node's got you covered. And it's **fast**, because we all hate waiting.

---

## Features
- **ROS Integration**: Subscribes to a point cloud (or any vector data) topic and publishes cluster results.
- **DBSCAN Implementation**: Pure C++ implementation that you can tweak.
- **High Customizability**: Tune the eps and minPts to get your perfect cluster segmentation.
- **Header-only** *option* for easily embedding the DBSCAN logic in other projects (if you want to pull it out).
- **Verbose Debugging**: Print out cluster stats or hush it entirely; it's up to you.

---

## Dependencies
- [ROS (Robot Operating System)](https://www.ros.org/) (tested on ROS Noetic, but should work on others if you tweak CMake)
- [catkin](http://wiki.ros.org/catkin)
- [C++11 or higher](https://isocpp.org/)
- [PCL (Point Cloud Library)](http://pointclouds.org/) (optional but recommended for handling 3D point data)
- [Eigen](http://eigen.tuxfamily.org/) (usually comes along with PCL in typical ROS installs)

---

## Installation

1. **Clone the Repo**  
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your-username/dbscan_ros_cpp.git
   ```

2. **Install Dependencies**  
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-pcl-ros \
                        libpcl-dev \
                        libeigen3-dev
   ```
   *(Adjust `noetic` to your ROS distro name if needed.)*

3. **Build**  
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
   or
   ```bash
   catkin build
   ```

4. **Source**  
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

---

## Usage
1. **Launch the Node**  
   ```bash
   rosrun dbscan_ros_cpp dbscan_ros_node
   ```
   or create a launch file:
   ```xml
   <launch>
     <node pkg="dbscan_ros_cpp" type="dbscan_ros_node" name="dbscan_ros_node" output="screen">
       <param name="cluster_tolerance" value="0.5" />
       <param name="min_cluster_size" value="5" />
     </node>
   </launch>
   ```

2. **Input Topic**  
   - By default, listens to `/point_cloud_in` (type `sensor_msgs/PointCloud2`).
   - Remap if needed:
     ```bash
     rosrun dbscan_ros_cpp dbscan_ros_node \
       _cluster_tolerance:=0.7 \
       _min_cluster_size:=10 \
       point_cloud_in:=/scan_points
     ```

3. **Output**  
   - Publishes cluster IDs (or debug cloud) to `/clusters_out`.
   - Use it for RViz, logging, or a higher-level perception pipeline.

---

## Code Breakdown

Below is a **single-file** example that combines both DBSCAN logic and ROS integration. You can split it into multiple files or refine as you like.

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <iostream>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class DBSCAN {
public:
    DBSCAN(double eps, int minPts)
        : eps_(eps), minPts_(minPts) {}

    std::vector<int> cluster(const PointCloudT::Ptr& cloud) {
        int n = cloud->points.size();
        std::vector<int> labels(n, UNVISITED);

        int clusterId = 0;
        for (int i = 0; i < n; ++i) {
            if (labels[i] == UNVISITED) {
                auto neighbors = regionQuery(cloud, i);
                if (neighbors.size() < minPts_) {
                    labels[i] = NOISE;
                } else {
                    ++clusterId;
                    expandCluster(cloud, labels, i, neighbors, clusterId);
                }
            }
        }
        return labels;
    }

private:
    double eps_;
    int minPts_;
    const int UNVISITED = -1;
    const int NOISE = -2;

    void expandCluster(const PointCloudT::Ptr& cloud,
                       std::vector<int>& labels,
                       int pointIdx,
                       const std::vector<int>& neighbors,
                       int clusterId) {
        labels[pointIdx] = clusterId;
        std::vector<int> seeds = neighbors;

        for (size_t i = 0; i < seeds.size(); ++i) {
            int currIdx = seeds[i];
            if (labels[currIdx] == UNVISITED || labels[currIdx] == NOISE) {
                labels[currIdx] = clusterId;
                auto currNeighbors = regionQuery(cloud, currIdx);
                if (currNeighbors.size() >= minPts_) {
                    seeds.insert(seeds.end(), currNeighbors.begin(), currNeighbors.end());
                }
            }
        }
    }

    std::vector<int> regionQuery(const PointCloudT::Ptr& cloud, int pointIdx) {
        std::vector<int> neighbors;
        PointT p = cloud->points[pointIdx];
        for (int i = 0; i < (int)cloud->points.size(); ++i) {
            if (i == pointIdx) continue;
            double dist = euclideanDistance(p, cloud->points[i]);
            if (dist <= eps_) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    inline double euclideanDistance(const PointT& p1, const PointT& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

class DBSCANNode {
public:
    DBSCANNode(ros::NodeHandle& nh) {
        nh.param("cluster_tolerance", eps_, 0.5);
        nh.param("min_cluster_size", minPts_, 5);

        input_topic_ = nh.resolveName("point_cloud_in");
        output_topic_ = nh.resolveName("clusters_out");

        sub_ = nh.subscribe(input_topic_, 1, &DBSCANNode::pointCloudCallback, this);
        pub_ = nh.advertise<PointCloudT>(output_topic_, 1);

        ROS_INFO_STREAM("DBSCANNode ready. Subscribed to " << input_topic_
                        << ", publishing to " << output_topic_);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double eps_;
    int minPts_;
    std::string input_topic_;
    std::string output_topic_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        PointCloudT::Ptr cloud_in(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud_in);

        DBSCAN dbscan(eps_, minPts_);
        auto labels = dbscan.cluster(cloud_in);

        PointCloudT::Ptr cloud_out(new PointCloudT);
        cloud_out->header = cloud_in->header;
        cloud_out->height = 1;
        cloud_out->width = cloud_in->points.size();

        for (size_t i = 0; i < cloud_in->points.size(); ++i) {
            PointT p;
            p.x = cloud_in->points[i].x;
            p.y = cloud_in->points[i].y;
            // Put cluster ID in z just for debug
            p.z = labels[i];
            cloud_out->points.push_back(p);
        }

        pub_.publish(cloud_out);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dbscan_ros_node");
    ros::NodeHandle nh("~");
    DBSCANNode node(nh);

    ros::spin();
    return 0;
}
```

**Key Highlights:**
- **`DBSCAN` Class**: Encapsulates clustering logic.
- **`DBSCANNode`**: ROS integration (sub + pub).
- **Cluster Visualization**: Uses z-coordinate for debug labeling.

---

## Roadmap
- [ ] Dynamic reconfigure for runtime tuning.
- [ ] More efficient neighbor search (KD-Tree).
- [ ] Example launch files for different data types.
- [ ] RViz marker support for cluster visualization.

---

## Contributing
1. Fork the project.
2. Create a feature branch.
3. Commit your changes.
4. Open a Pull Request.

---

## License
[MIT License](LICENSE)

Use this code responsibly and give credit where it's due.

---

**Now go forth and cluster everything in sight.**
