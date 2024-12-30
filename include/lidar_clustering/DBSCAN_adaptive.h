#ifndef DBSCAN_H
#define DBSCAN_H

#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <time.h>
#include <iostream>



#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
    
}

template <typename PointT>
class AdaptiveDBSCANCluster {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    // typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    // typedef typename pcl::KdTreeFLANN<PointT>::Ptr KdTreePtr;
    typedef typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr KdTreePtr;
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method_ = tree;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        int nn_size = 0;
        double eps = 0.0;
        int minPts = 0;
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        
        for (int i = 0; i < input_cloud_->points.size(); i++)
        {
            if (types[i] == PROCESSED) {
                continue;
            }
            //check if the point is in noise removal region
            
            std::tuple<double, int> eps_and_num_points = get_eps_and_num_points(input_cloud_->points[i]);
            double eps = std::get<0>(eps_and_num_points);
            int minPts = std::get<1>(eps_and_num_points);
            // std::cout << "distance_to_center: " << distance_to_center << ", eps: " << eps << ", minPts: " << minPts << "\n";

            nn_size = radiusSearch(i, eps, nn_indices, nn_distances);
            if (nn_size < minPts) {
                is_noise[i] = true;
                continue;
            }

            // if code goes to here, the point is not noise, and it is a core point.
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            
            for (int j = 0; j < nn_size; j++) {
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                // // option 1, recalculate eps and minPts for each point
                std::tuple<double, int> eps_and_num_points = get_eps_and_num_points(input_cloud_->points[cloud_index]);
                double eps = std::get<0>(eps_and_num_points);
                int minPts = std::get<1>(eps_and_num_points);
                nn_size = radiusSearch(cloud_index, eps, nn_indices, nn_distances);

                // option 2, use the same eps and minPts for all points near to the core point.
                // nn_size = radiusSearch(cloud_index, eps, nn_indices, nn_distances);
                if (nn_size >= minPts) {
                    for (int j = 0; j < nn_size; j++) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                // // These two lines should not be needed: (can anyone confirm?) -FF
                // std::sort (r.indices.begin (), r.indices.end ());
                // r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

                r.header = input_cloud_->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        // std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
    }

    void setClusteringParams(double obj_width_thre, int num_points_per_line, int max_num_lines, double eps_with_distance_factor, double num_line_with_distance_factor) {
        obj_width_thre_ = obj_width_thre;
        num_points_per_line_ = num_points_per_line;
        max_num_lines_ = max_num_lines;
        eps_with_distance_factor_ = eps_with_distance_factor;
        num_line_with_distance_factor_ = num_line_with_distance_factor;
    }

    std::tuple<double, int> get_eps_and_num_points(const PointT &point) {

        double i_x = point.x;
        double i_y = point.y;
        double i_z = point.z;
        double s = std::sqrt(i_x * i_x + i_y * i_y + i_z * i_z);
        double eps = std::max(eps_with_distance_factor_ * s, obj_width_thre_);
        int num_points = std::min(std::max(static_cast<int>(num_line_with_distance_factor_ / s), 1), max_num_lines_) * num_points_per_line_;
        if (i_x < 0) {
            eps = eps * 1.5; // the back side only has the BP lidar
        }
        else
        {
            eps = eps * 1.0;
        }
        return std::make_tuple(eps, num_points);
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }

protected:
    PointCloudPtr input_cloud_;
    
    // Clustering parameters
    double obj_width_thre_{0.15};
    int num_points_per_line_{4};
    int max_num_lines_{1};
    double eps_with_distance_factor_{0.0};
    double num_line_with_distance_factor_{0.0};

    int min_pts_per_cluster_{10};
    int max_pts_per_cluster_{1000};

    KdTreePtr search_method_;

    virtual int radiusSearch (
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }
}; // class DBSCANCluster

#endif // DBSCAN_H
