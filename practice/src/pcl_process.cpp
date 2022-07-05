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

// void XYZtoXYZRGB(boost::shared_ptr<point_cloud_t> &cloud,
//                 boost::shared_ptr<point_cloud_rgb_t> &cloud_rgb)
void XYZtoXYZRGB(point_cloud_t::Ptr input,
                point_cloud_rgb_t::Ptr output)
{
    output->points.resize(input->points.size());
    output->width = input->width;
    output->height = input->height;
    // std::cout<<cloud_rgb->header<<" "<<cloud_rgb->is_dense<<std::endl;
    // std::cout<<cloud->header<<" "<<cloud->is_dense<<std::endl;

    for (size_t i = 0; i < input->points.size(); i++) 
    {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].z;
        
        output->points[i].r = 255;
        output->points[i].g = 0;
        output->points[i].b = 0;
    }
}

// void segmen_plane(boost::shared_ptr<point_cloud_rgb_t> &cloud_rgb,
//                 boost::shared_ptr<point_cloud_rgb_t> &inlierPoints,
//                 boost::shared_ptr<pcl::PointIndices> &inliers,
//                 boost::shared_ptr<pcl::ModelCoefficients> &coefficients)
void segmen_plane(point_cloud_rgb_t::Ptr input,
                pcl::PointIndices::Ptr inliers,
                pcl::ModelCoefficients::Ptr coefficients)
{
    // Create the segmentation object.
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);       // Enable model coefficient refinement (optional).
    seg.setInputCloud (input);
    seg.setModelType (pcl::SACMODEL_PLANE);   // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);      // Use RANSAC method.
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);          // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);     // cylinder, Set minimum and maximum radii of the cylinder.
    seg.segment (*inliers, *coefficients);

    // pcl::copyPointCloud<pcl::PointXYZRGB>(*input, *inliers, *inlierPoints);
}


int main (int argc, char** argv)
{
    uint8_t max_plane_idx = 1;
    point_cloud_t::Ptr cloud (new point_cloud_t ());

    point_cloud_rgb_t::Ptr cloud_rgb (new point_cloud_rgb_t),
                        inlierPoints (new point_cloud_rgb_t),
                        inlierPoints_neg (new point_cloud_rgb_t);
    point_cloud_rgb_t *segments;
    segments = new point_cloud_rgb_t[max_plane_idx];

    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../../data/kinect_robot/world_filtered.pcd", *cloud_rgb);
    // XYZtoXYZRGB(cloud, cloud_rgb);

    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    for (uint8_t i = 0; i < max_plane_idx; i++)
    {
        uint8_t r = color_picker[i][0];
        uint8_t g = color_picker[i][1];
        uint8_t b = color_picker[i][2];
        
        segmen_plane(cloud_rgb, inliers, coefficients);
        pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_rgb, *inliers, *inlierPoints);
        for (int m=0; m<inliers->indices.size(); m++)
        {
            inlierPoints->points[inliers->indices[m]].r = r;
            inlierPoints->points[inliers->indices[m]].g = g;
            inlierPoints->points[inliers->indices[m]].b = b;
        }
        pcl::copyPointCloud<pcl::PointXYZRGB>(*inlierPoints, segments[i]);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_rgb);
        extract.setIndices (inliers);
        extract.setNegative (true);//false
        extract.filter (*inlierPoints_neg);

        pcl::copyPointCloud<pcl::PointXYZRGB>(*inlierPoints_neg, *cloud_rgb);

        // std::cout<<"passed"<<std::endl;
        // std::cout<<inlierPoints_neg->points.size()<<std::endl;
        // std::cout<<cloud_rgb->points.size()<<std::endl;

        // std::cout<<inlierPoints_neg->header<<" "<<inlierPoints_neg->is_dense<<std::endl;
        // std::cout<<cloud_rgb->header<<" "<<cloud_rgb->is_dense<<std::endl;

        // std::cout<<inlierPoints_neg->height<<" "<<inlierPoints_neg->width<<std::endl;
        // std::cout<<cloud_rgb->height<<" "<<cloud_rgb->width<<std::endl;
        // std::cout<<"\n"<<std::endl;

    }

    std::cout<<segments[0].points.size()<<std::endl;

    pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/TLS_kitchen_segmen_cpp.pcd", segments[0]);

    delete [] segments;

    return (0);
}
