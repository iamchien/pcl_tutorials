#include <bits/stdc++.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_rgb_t;

void extract_indices(point_cloud_rgb_t::Ptr &cloud_rgb,
                point_cloud_rgb_t::Ptr &inlierPoints_neg,
                pcl::PointIndices::Ptr &inliers);

void clustering(point_cloud_rgb_t::Ptr &cloud_rgb,
    vector<pcl::PointIndices> &cluster_indices);

point_cloud_rgb_t projectedPCD(point_cloud_rgb_t::Ptr &inlierPoints,
                            pcl::ModelCoefficients::Ptr &coefficient);

void segmen_plane(point_cloud_rgb_t::Ptr &cloud_rgb, int idx,
                bool do_cluster, vector<point_cloud_rgb_t> &segments,
                vector<pcl::ModelCoefficients> &coeffs);

