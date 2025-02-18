#include <lidar_clustering/dbscan_ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <pcl/octree/octree_search.h>
#include <autoware_perception_msgs/DynamicObjectWithFeature.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <math.h>

dbscan_ros::dbscan_ros(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
    nh_private.param<std::string>("topic_input", topic_input_, "/rslidar_points_front/ROI/no_ground");
    nh_private.param<std::string>("topic_output", topic_output_, "/dbscan_clustering/clusters");
    nh_private.param<double>("obj_width_thre", obj_width_thre_, 0.15);
    nh_private.param<double>("obj_height_thre", obj_height_thre_, 0.15);
    nh_private.param<double>("downsample_resolution", downsample_resolution_, 0.03);
    nh_private.param<double>("lidar_horizon_resolution", lidar_horizon_resolution_, 0.2);
    nh_private.param<double>("lidar_vertical_resolution", lidar_vertical_resolution_, 1.0);

    // Calculate the clustering parameters
    lidar_horizon_resolution_ = lidar_horizon_resolution_ * M_PI / 180.0;
    lidar_vertical_resolution_ = lidar_vertical_resolution_ * M_PI / 180.0;
    num_points_per_line_ = static_cast<int>(obj_width_thre_ / downsample_resolution_);
    max_num_lines_ = static_cast<int>(obj_height_thre_ / downsample_resolution_);
    eps_with_distance_factor_ = num_points_per_line_ * lidar_horizon_resolution_;
    num_line_with_distance_factor_ = obj_height_thre_ / lidar_vertical_resolution_;

    // Publish and subscribe
    pc_sub = nh.subscribe(topic_input_, 1, &dbscan_ros::lidar_callback, this);
    cluster_marker_pub=nh.advertise<visualization_msgs::MarkerArray>("/dbscan_clustering/marker",10);
    cluster_array_pub = nh.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>(topic_output_, 10);
    noise_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/dbscan_clustering/noise_points", 10);
}

void dbscan_ros::lidar_callback(const sensor_msgs::PointCloud2 pc2_msg)
{
    
    auto start=std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_remo 
                (new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::fromROSMsg(pc2_msg, *ground_remo);

    visualization_msgs::MarkerArray vec_markers;
    autoware_perception_msgs::DynamicObjectWithFeatureArray cluster_array;
    cluster_array.header = pc2_msg.header;

    if (ground_remo->size() ==0)
    {
        cluster_marker_pub.publish(vec_markers);
        cluster_array_pub.publish(cluster_array);
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>(*ground_remo));

    for (auto& p:*projected) p.z = p.z/4.0; //reduce z importance

    pcl::PointCloud<pcl::PointXYZ>::Ptr clusters
                        (new pcl::PointCloud<pcl::PointXYZ>());


    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    // tree->setInputCloud(projected);
    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    // tree->setInputCloud(projected);
    //change to octree
    float resolution = 0.03;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr tree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution));
    tree->setInputCloud(projected);
    tree->addPointsFromInputCloud();

    std::vector<pcl::PointIndices> indices;
    //save the indices of all the clusters then find the noise points
    pcl::PointIndices all_cluster_indices;

    AdaptiveDBSCANCluster<pcl::PointXYZ> db;
    db.setInputCloud(ground_remo);
    db.setSearchMethod(tree);
    db.setClusteringParams(obj_width_thre_, num_points_per_line_, max_num_lines_, eps_with_distance_factor_, num_line_with_distance_factor_);
    db.setMinClusterSize(8);
    db.setMaxClusterSize(20000);
    db.extract(indices);


    int i=0;
    pcl::ExtractIndices<pcl::PointXYZ> ext;
    ext.setInputCloud(ground_remo);
    for (pcl::PointIndices ind:indices)
    {
        // save the indices of all the clusters
        all_cluster_indices.indices.insert(all_cluster_indices.indices.end(), ind.indices.begin(), ind.indices.end());

        pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
        pcl::PointIndices::Ptr ind_ptr(new pcl::PointIndices(ind));

        ext.setIndices(ind_ptr);
        ext.filter(cluster_cloud);

        //calculate the min max of the cluster
        float xsum=0.0f;
        float ysum=0.0f;
        float zsum=0.0f;
        float xmin=1000.0f;
        float ymin=1000.0f;
        float zmin=1000.0f;
        float xmax=-1000.0f;
        float ymax=-1000.0f;
        float zmax=-1000.0f;
        for (auto& p:cluster_cloud)
        {
            xsum+=p.x;
            ysum+=p.y;
            zsum+=p.z;
            if (p.x<xmin) xmin=p.x;
            if (p.y<ymin) ymin=p.y;
            if (p.z<zmin) zmin=p.z;
            if (p.x>xmax) xmax=p.x;
            if (p.y>ymax) ymax=p.y;
            if (p.z>zmax) zmax=p.z;
        }
        //calculate the center of the cluster
        float xcenter=xsum/cluster_cloud.size();
        float ycenter=ysum/cluster_cloud.size();
        float zcenter=zsum/cluster_cloud.size();

        //create objectwithfeature
        autoware_perception_msgs::DynamicObjectWithFeature cluster;
        cluster.object.state.pose_covariance.pose.position.x = xcenter;
        cluster.object.state.pose_covariance.pose.position.y = ycenter;
        cluster.object.state.pose_covariance.pose.position.z = zcenter;
        sensor_msgs::PointCloud2 ros_pointcloud;
        pcl::toROSMsg(cluster_cloud, ros_pointcloud);
        cluster.feature.cluster = ros_pointcloud;
        cluster_array.feature_objects.push_back(cluster);

        //create marker
        visualization_msgs::Marker marker;
        marker.action=visualization_msgs::Marker::ADD;
        marker.color.a=1.0f;
        // marker.color.r=1.0f;
        marker.color.g = 1.0f;
        marker.scale.z = zmax -zmin + 0.1f;
        marker.scale.x = xmax - xmin + 0.1f;
        marker.scale.y = ymax - ymin + 0.1f;
        marker.ns="clusters";
        marker.type=visualization_msgs::Marker::CUBE;
        marker.id=i++;
        marker.header.frame_id="rslidar_front";
        marker.header.stamp=ros::Time::now();
        marker.pose.orientation.w=1.0f;
        marker.pose.position.x=(xmin+xmax)/2;
        marker.pose.position.y=(ymin+ymax)/2;
        marker.pose.position.z=(zmin+zmax)/2;
        marker.lifetime=ros::Duration(0.1);
        vec_markers.markers.push_back(marker);
    }

    cluster_marker_pub.publish(vec_markers);
    cluster_array_pub.publish(cluster_array);

    //find the noise points and publish them
    pcl::PointIndices::Ptr all_cluster_indices_ptr(new pcl::PointIndices(all_cluster_indices));
    ext.setIndices(all_cluster_indices_ptr);
    pcl::PointCloud<pcl::PointXYZ> noise_cloud;
    ext.setNegative(true);
    ext.filter(noise_cloud);
    sensor_msgs::PointCloud2 noise_ros_pointcloud;
    pcl::toROSMsg(noise_cloud, noise_ros_pointcloud);
    noise_ros_pointcloud.header = pc2_msg.header;
    noise_points_pub.publish(noise_ros_pointcloud);

    auto end=std::chrono::steady_clock::now();


    auto duration_=std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();


    if (duration_ >= 10 ) std::cout<<"time took is, "<< duration_<<std::endl;


}
