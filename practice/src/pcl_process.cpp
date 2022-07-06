#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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

// void XYZtoXYZRGB(boost::shared_ptr<point_cloud_t> &input,
//                 boost::shared_ptr<point_cloud_rgb_t> &output)
void XYZtoXYZRGB(point_cloud_t &input,
                point_cloud_rgb_t &output)
{
    output.points.resize(input.points.size());
    output.width = input.width;
    output.height = input.height;
    // std::cout<<cloud_rgb->header<<" "<<cloud_rgb->is_dense<<std::endl;
    // std::cout<<cloud->header<<" "<<cloud->is_dense<<std::endl;

    for (size_t i = 0; i < input.points.size(); i++) 
    {
        output.points[i].x = input.points[i].x;
        output.points[i].y = input.points[i].y;
        output.points[i].z = input.points[i].z;
        
        output.points[i].r = 255;
        output.points[i].g = 0;
        output.points[i].b = 0;
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

void segmen_plane(point_cloud_rgb_t::Ptr &cloud_rgb,
                point_cloud_rgb_t::Ptr &segment,
                int max_plane_idx)
{
    point_cloud_rgb_t::Ptr inlierPoints (new point_cloud_rgb_t ()),
                        inlierPoints_neg (new point_cloud_rgb_t ());

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    point_cloud_rgb_t *segments;
    segments = new point_cloud_rgb_t[max_plane_idx];

    for (int i = 0; i < max_plane_idx; i++)
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
        // for (int m=0; m<inliers->indices.size(); m++)
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


int main (int argc, char** argv)
{
    point_cloud_rgb_t::Ptr cloud_rgb (new point_cloud_rgb_t),
                        segment (new point_cloud_rgb_t);

    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../../data/kinect_robot/TLS_kitchen_rgb.pcd", *cloud_rgb);
    // XYZtoXYZRGB(*cloud, *cloud_rgb);
    segmen_plane(cloud_rgb, segment, 10);

    pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/TLS_kitchen_segmen_cpp.pcd", *segment);

    return (0);
}
