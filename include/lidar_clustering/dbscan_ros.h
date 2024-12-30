#ifndef _DBSCAN_ROS_H
#define _DBSCAN_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include "DBSCAN_adaptive.h"




class dbscan_ros
{
    public:
        dbscan_ros(ros::NodeHandle nh, ros::NodeHandle nh_private);
        void lidar_callback(const sensor_msgs::PointCloud2 lidar_msg);
    private:
        double obj_width_thre_;
        double obj_height_thre_;
        double downsample_resolution_;
        double lidar_horizon_resolution_;
        double lidar_vertical_resolution_;

        // Clustering parameters
        int num_points_per_line_;
        int max_num_lines_;
        double eps_with_distance_factor_;
        double num_line_with_distance_factor_;
        
        std::string topic_input_;
        std::string topic_output_;

        ros::Subscriber pc_sub;
        ros::Publisher cluster_marker_pub;
        ros::Publisher cluster_array_pub;
        ros::Publisher noise_points_pub;
};



#endif
