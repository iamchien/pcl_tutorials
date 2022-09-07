#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "utils.cpp"
#include "rotating_calipers.cpp"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_rgb_t;
typedef pair<double, double> point;

int main (int argc, char** argv)
{
    point_cloud_rgb_t::Ptr cloud_rgb (new point_cloud_rgb_t),
                        segment (new point_cloud_rgb_t);

    vector<pcl::ModelCoefficients> coeffs;
    vector<point_cloud_rgb_t> segments;

    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("../../data/kinect_robot/world_filtered.pcd", *cloud_rgb);
    // XYZtoXYZRGB(*cloud, *cloud_rgb);

    segmen_plane(cloud_rgb, 0, false, segments, coeffs);
    for (int i = 1; i < 8; i++){
        segmen_plane(cloud_rgb, i, true, segments, coeffs);
    }

    int s_size = segments.size();
    vector<vector<double>> circles;
    for (int i = 1; i < s_size; i++){
        point_cloud_rgb_t cloud_projected = projectedPCD(segments[i], coeffs[0]);
        vector<point> points;

        double first = 0.0, second = 0.0;
        int cp_size = cloud_projected.size();

        for (int j = 0; j < cp_size; j++){
            points.push_back({cloud_projected[j].x, cloud_projected[j].y});
            first += cloud_projected[j].x;
            second += cloud_projected[j].y;
        }
        
        double d = diameter(points);
        circles.push_back({first/cp_size, second/cp_size, d});

        // cout << circles.size() << endl;
        // exit(0);
        // *segment += cloud_projected;
    }

    // pcl::io::savePCDFile<pcl::PointXYZRGB>("../data/output/world_filtered_segmen_cpp.pcd", *segment);

    return (0);
}

