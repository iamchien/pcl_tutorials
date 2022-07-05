#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Plane model segmentation
//http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_rgb_t;

int color_picker[10][3] = {
    {0  , 153, 204},
    {0  , 255, 255},
    {51 , 204, 51 },
    {204, 204, 0  },
    {255, 102, 0  },
    {255, 102, 204},
    {51 , 102, 255},
    {153, 51 , 153},
    {153, 51 , 51 },
    {255, 204, 0  }
};

void estimate_normals(boost::shared_ptr<point_cloud_t> &cloud,
                    boost::shared_ptr<pcl::PointCloud<pcl::Normal>> &normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // For every point, use all neighbors in a radius of 3cm.
    ne.setRadiusSearch(0.03);
    // A kd-tree is a data structure that makes searches efficient. More about it later.
    // The normal estimation object will use it to find nearest neighbors.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(kdtree);

    // Calculate the normals.
    ne.compute(*normals);
}

void segmen_plane(boost::shared_ptr<point_cloud_rgb_t> &cloud,
                boost::shared_ptr<pcl::PointIndices> &inliers,
                boost::shared_ptr<pcl::ModelCoefficients> &coefficients)
{
    // Create the segmentation object.
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);       // Enable model coefficient refinement (optional).
    seg.setInputCloud (cloud);
    seg.setModelType (pcl::SACMODEL_PLANE);   // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);      // Use RANSAC method.
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);          // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);     // cylinder, Set minimum and maximum radii of the cylinder.
    seg.segment (*inliers, *coefficients);
}


int main (int argc, char** argv)
{
    point_cloud_t::Ptr cloud (new point_cloud_t);

    point_cloud_rgb_t::Ptr cloud_rgb (new point_cloud_rgb_t),
                        inlierPoints (new point_cloud_rgb_t),
                        inlierPoints_neg (new point_cloud_rgb_t);

    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    // pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

    // Load *.PCD file (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop_passthrough.pcd)
    // pcl::io::loadPCDFile<pcl::PointXYZ> ("../../data/kinect_robot/TLS_kitchen.pcd", *cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/output/outlier/cpp/rest_8.pcd", *cloud);

    cloud_rgb->points.resize(cloud->points.size());
    cloud_rgb->width = cloud->width;
    cloud_rgb->height = cloud->height;
    // std::cout<<cloud_rgb->header<<" "<<cloud_rgb->is_dense<<std::endl;
    // std::cout<<cloud->header<<" "<<cloud->is_dense<<std::endl;

    for (size_t i = 0; i < cloud->points.size(); i++) 
    {
        cloud_rgb->points[i].x = cloud->points[i].x;
        cloud_rgb->points[i].y = cloud->points[i].y;
        cloud_rgb->points[i].z = cloud->points[i].z;
        
        cloud_rgb->points[i].r = 255;
        cloud_rgb->points[i].g = 0;
        cloud_rgb->points[i].b = 0;
    }

    // pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_rgb, *inlierPoints);
    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // uint8_t max_plane_idx = 10;
    // for (uint8_t i = 0; i < max_plane_idx; i++){
    //     uint8_t r = color_picker[i][0];
    //     uint8_t g = color_picker[i][1];
    //     uint8_t b = color_picker[i][2];
        
    //     segmen_plane(inlierPoints, inliers, coefficients);

    //     for (int m=0; m<inliers->indices.size(); m++)
    //     {
    //         cloud_rgb->points[inliers->indices[m]].r = r;
    //         cloud_rgb->points[inliers->indices[m]].g = g;
    //         cloud_rgb->points[inliers->indices[m]].b = b;
    //     }

    //     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    //     extract.setInputCloud (inlierPoints);
    //     extract.setIndices (inliers);
    //     extract.setNegative (true);//false
    //     extract.filter (*inlierPoints_neg);

    //     pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/cpp/outlier/rest_0.pcd", *cloud_rgb);

    //     inlierPoints = inlierPoints_neg;
    // }

    segmen_plane(cloud_rgb, inliers, coefficients);

    int r = color_picker[9][0];
    int g = color_picker[9][1];
    int b = color_picker[9][2];

    for (int m=0; m<inliers->indices.size(); m++)
    {
        cloud_rgb->points[inliers->indices[m]].r = r;
        cloud_rgb->points[inliers->indices[m]].g = g;
        cloud_rgb->points[inliers->indices[m]].b = b;
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_rgb);
    extract.setIndices (inliers);

    // extract.setNegative (true);//false
    extract.filter (*inlierPoints);

    extract.setNegative (true);//false
    extract.filter (*inlierPoints_neg);

    pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/outlier/cpp/rest_9.pcd", *inlierPoints_neg);
    pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/inlier/cpp/in_9.pcd", *inlierPoints);

    // estimate_normals(cloud, normals);

    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // ne.setInputCloud(cloud);
    // // For every point, use all neighbors in a radius of 3cm.
    // ne.setRadiusSearch(0.1);
    // // A kd-tree is a data structure that makes searches efficient. More about it later.
    // // The normal estimation object will use it to find nearest neighbors.
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    // ne.setSearchMethod(kdtree);

    // // Calculate the normals.
    // ne.compute(*normals);

    // pcl::concatenateFields (*cloud, *normals, p_n_cloud_c);
    // pcl::io::saveLYFile<pcl::PointNormal>("../data/output/output.pcd", p_n_cloud_c);

    return (0);
}
