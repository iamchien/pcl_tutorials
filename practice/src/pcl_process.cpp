#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
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
                pcl::PointIndices::Ptr &inliers);

void segmen_plane(point_cloud_rgb_t::Ptr &cloud_rgb,
                point_cloud_rgb_t::Ptr &segment, int start_idx,
                int max_plane_idx, bool do_cluster, bool do_projected);

void clustering(point_cloud_rgb_t::Ptr &cloud_rgb,
    std::vector<pcl::PointIndices> &cluster_indices);

void concave_hull(point_cloud_rgb_t::Ptr &inlierPoints,
                pcl::PointIndices::Ptr &inliers,
                point_cloud_rgb_t::Ptr &cloud_hull,
                pcl::ModelCoefficients::Ptr &coefficients,
                bool do_projected);

int main (int argc, char** argv)
{
    point_cloud_rgb_t::Ptr cloud_rgb (new point_cloud_rgb_t),
                        segment (new point_cloud_rgb_t);

    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../../data/kinect_robot/world_filtered.pcd", *cloud_rgb);
    // XYZtoXYZRGB(*cloud, *cloud_rgb);

    segmen_plane(cloud_rgb, segment, 0, 1, false, false);
    segmen_plane(cloud_rgb, segment, 1, 8, true, true);

    // pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/world_filtered_segmen_cpp.pcd", *segment);

    return (0);
}

void segmen_plane(point_cloud_rgb_t::Ptr &cloud_rgb,
                point_cloud_rgb_t::Ptr &segment, int start_idx,
                int max_plane_idx, bool do_cluster, bool do_projected)
{
    point_cloud_rgb_t::Ptr inlierPoints (new point_cloud_rgb_t ()),
                        inlierPoints_neg (new point_cloud_rgb_t ()),
                        cloud_hull (new point_cloud_rgb_t ());

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    point_cloud_rgb_t *segments;
    segments = new point_cloud_rgb_t[max_plane_idx];

    for (int i = start_idx; i < max_plane_idx; i++)
    {
        uint8_t r = color_picker[i][0];
        uint8_t g = color_picker[i][1];
        uint8_t b = color_picker[i][2];
        
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

        pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_rgb, *inliers, *inlierPoints);
        // std::cout << inliers->indices.size() << std::endl;
        // std::cout << inlierPoints->points.size() << std::endl;
        
        std::stringstream ss;
        if (do_cluster)
        {
            std::vector<pcl::PointIndices> cluster_indices;
            clustering(inlierPoints, cluster_indices);
            std::cout << "PointCloud representing the Cluster: " << cluster_indices.size() << " data points." << std::endl;
            size_t re[2] = {cluster_indices[0].indices.size(), 0};
            for (size_t j = 1; j < cluster_indices.size(); j++)
            {
                size_t tmp = cluster_indices[j].indices.size();
                // std::cout << tmp << std::endl;
                if (re[0] <= tmp)
                {
                    re[0] = tmp;
                    re[1] = j;
                }
            }
            *inliers = cluster_indices[re[1]];
            // std::cout << inliers->indices.size() << std::endl;
            pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_rgb, *inliers, *inlierPoints);

            ss << "../data/output/inlier_" << i << ".pcd";
            pcl::io::savePCDFile<pcl::PointXYZRGB>(ss.str(), *inlierPoints);
            ss.rdbuf()->pubseekpos(0);
        }

        concave_hull(inlierPoints, inliers, cloud_hull, coefficients, do_projected);
        std::cerr << "Concave hull has: " << cloud_hull->size () << " data points." << std::endl;
        
        ss << "../data/output/cloud_projected_" << i << ".pcd";
        std::cout << ss.str() << std::endl;

        pcl::io::savePCDFile<pcl::PointXYZRGB>(ss.str(), *cloud_hull);

        for (int m=0; m<inlierPoints->points.size(); m++)
        {   
            uint32_t rgb = (r << 16) | (g << 8) | b;
            inlierPoints->points[m].rgba = *(uint32_t *)(&rgb); // makes the point red
        }

        pcl::copyPointCloud<pcl::PointXYZRGB>(*inlierPoints, segments[i]);
        *segment += segments[i];
        extract_indices(cloud_rgb, inlierPoints_neg, inliers);
        pcl::copyPointCloud<pcl::PointXYZRGB>(*inlierPoints_neg, *cloud_rgb);
    }
}

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
    std::vector<pcl::PointIndices> &cluster_indices)
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

    // std::cout << cluster_indices.size() << std::endl;
}

void concave_hull(point_cloud_rgb_t::Ptr &inlierPoints,
                pcl::PointIndices::Ptr &inliers,
                point_cloud_rgb_t::Ptr &cloud_hull,
                pcl::ModelCoefficients::Ptr &coefficients,
                bool do_projected)
{
    if (do_projected)
    {
        point_cloud_rgb_t::Ptr cloud_projected (new point_cloud_rgb_t());
        
        // Project the model inliers
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (inlierPoints);
        proj.setIndices (inliers);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_projected);
        // proj.filter (*cloud_hull);

        std::cerr << "Peojected has: " << cloud_projected->size () << " data points." << std::endl;
        pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_projected, *inlierPoints);
    }

    pcl::ConcaveHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (inlierPoints);
    // chull.setInputCloud (inlierPoints);
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);

    // std::cerr << "Concave hull has: " << cloud_hull->size () << " data points." << std::endl;
}