#include<iostream>
#include<chrono>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

using namespace std;
using namespace std::chrono;

int main(){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read("../../../data/asia_dragon/data.pcd", *cloud); // Remember to download the file first!
    cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        << " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;
    
    // Get starting timepoint
    auto start = high_resolution_clock::now();

    // Cereate the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (10.0f,10.0f,10.0f);
    sor.filter (*cloud_filtered);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
 
    cerr << "Time taken by function: "
         << duration.count() << " microseconds" << endl;

    cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << endl;

    pcl::PCDWriter writer;
    writer.write("../data/output/data_10.pcd", *cloud_filtered,
        Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
    
    return (0);
}

