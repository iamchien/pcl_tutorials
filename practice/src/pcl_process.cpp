#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "utils.cpp"
// #include "render.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_rgb_t;

int main (int argc, char** argv)
{
    point_cloud_rgb_t::Ptr cloud_rgb (new point_cloud_rgb_t),
                        segment (new point_cloud_rgb_t);

    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../../data/kinect_robot/world_filtered.pcd", *cloud_rgb);
    // XYZtoXYZRGB(*cloud, *cloud_rgb);

    segmen_plane(cloud_rgb, segment, 0, 1, false, false);
    segmen_plane(cloud_rgb, segment, 1, 8, true, true);

    pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/world_filtered_segmen_cpp.pcd", *segment);

    return (0);
}

