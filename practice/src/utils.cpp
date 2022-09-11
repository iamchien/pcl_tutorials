#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>
#include "utils.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_rgb_t;

int color_picker[20][3] = {
    {0  , 153, 204},{255,   0, 255},
    {0  , 255, 255},{153,   0, 255},
    {51 , 204, 51 },{102, 102, 153},
    {204, 204, 0  },{  0,   0, 204},
    {255, 102, 0  },{  0,  51, 153},
    {255, 102, 204},{  0,  51, 102},
    {51 , 102, 255},{ 51, 153, 102},
    {153, 51 , 153},{  0, 102,   0},
    {153, 51 , 51 },{102, 153,   0},
    {255, 204, 0  },{255, 255,   0}
};

void extract_indices(point_cloud_rgb_t::Ptr &cloud_rgb,
                point_cloud_rgb_t::Ptr &inlierPoints_neg,
                pcl::PointIndices::Ptr &inliers)
{
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_rgb);
    extract.setIndices (inliers);
    extract.setNegative (true);//false
    extract.filter (*inlierPoints_neg);
}

void clustering(point_cloud_rgb_t::Ptr &inlierPoints,
    vector<pcl::PointIndices> &cluster_indices)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    tree->setInputCloud (inlierPoints);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.1); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inlierPoints);
    ec.extract (cluster_indices);

    // cout << cluster_indices.size() << endl;
}

point_cloud_rgb_t projectedPCD(point_cloud_rgb_t& inlierPoints,
                            pcl::ModelCoefficients& coefficient)
{
    point_cloud_rgb_t::Ptr cloud_projected (new point_cloud_rgb_t());

    point_cloud_rgb_t::Ptr tmpPoint (new point_cloud_rgb_t());
    *tmpPoint = inlierPoints;

    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients ());
    *coeff = coefficient;

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud(tmpPoint);
    proj.setModelCoefficients(coeff);
    proj.filter (*cloud_projected);

    return *cloud_projected;
}

void segmen_plane(point_cloud_rgb_t::Ptr &cloud_rgb, int idx,
                bool do_cluster, vector<point_cloud_rgb_t> &segments,
                vector<pcl::ModelCoefficients> &coeffs)
{
    point_cloud_rgb_t::Ptr inlierPoints (new point_cloud_rgb_t ()),
                        inlierPoints_neg (new point_cloud_rgb_t ()),
                        cloud_hull (new point_cloud_rgb_t ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    uint8_t r = color_picker[idx][0];
    uint8_t g = color_picker[idx][1];
    uint8_t b = color_picker[idx][2];
    
    // Create the segmentation object.
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);       // Enable model coefficient refinement (optional).
    seg.setInputCloud (cloud_rgb);
    seg.setModelType (pcl::SACMODEL_PLANE);   // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);      // Use RANSAC method.
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);          // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);     // cylinder, Set minimum and maximum radius of the cylinder.
    seg.segment (*inliers, *coefficients);

    coeffs.push_back(*coefficients);

    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_rgb, *inliers, *inlierPoints);
    // cout << inliers->indices.size() << endl;
    // cout << inlierPoints->points.size() << endl;
    
    stringstream ss;
    if (do_cluster)
    {
        vector<pcl::PointIndices> cluster_indices;
        clustering(inlierPoints, cluster_indices);
        // cout << "PointCloud representing the Cluster: " << cluster_indices.size() << " data points." << endl;
        size_t re[2] = {cluster_indices[0].indices.size(), 0};
        for (size_t j = 1; j < cluster_indices.size(); j++)
        {
            size_t tmp = cluster_indices[j].indices.size();
            // cout << tmp << endl;
            if (re[0] <= tmp)
            {
                re[0] = tmp;
                re[1] = j;
            }
        }
        *inliers = cluster_indices[re[1]];
        // cout << inliers->indices.size() << endl;
        pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_rgb, *inliers, *inlierPoints);

        // ss << "../data/output/inlier_" << i << ".pcd";
        // pcl::io::savePCDFile<pcl::PointXYZRGB>(ss.str(), *inlierPoints);
        // ss.str("");
    }

    // concave_hull(inlierPoints, inliers, cloud_hull, coefficient, do_projected);
    // cerr << "Concave hull has: " << cloud_hull->size () << " data points." << endl;
    
    // ss << "../data/output/cloud_projected_" << i << ".pcd";
    // cout << ss.str() << endl;

    // pcl::io::savePCDFile<pcl::PointXYZRGB>(ss.str(), *cloud_hull);

    // for (int m=0; m<inlierPoints->points.size(); m++)
    // {   
    //     uint32_t rgb = (r << 16) | (g << 8) | b;
    //     inlierPoints->points[m].rgba = *(uint32_t *)(&rgb); // makes the point red
    // }

    segments.push_back(*inlierPoints);
    extract_indices(cloud_rgb, inlierPoints_neg, inliers);
    pcl::copyPointCloud<pcl::PointXYZRGB>(*inlierPoints_neg, *cloud_rgb);
}



