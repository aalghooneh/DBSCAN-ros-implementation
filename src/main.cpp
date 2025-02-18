#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <lidar_clustering/dbscan_ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_clustering");

    // Create node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // std::string topic_input="/rslidar_points_front/ROI/no_ground";
    // std::string topic_output="/dbscan_clustering/clusters";
    // topic_input, topic_output,
    //     10, 6, 40000, 0.2

    // args: topic_input, topic_output, queue_size,
    // min_pts, max_pts, tolerance
    dbscan_ros cluster_node(nh, nh_private);

    ros::spin();

    return 0;
}
