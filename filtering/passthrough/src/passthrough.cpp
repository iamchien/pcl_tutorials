// #include <iostream>
// #include <pcl/point_types.h>
// #include <pcl/filters/passthrough.h>

// int main ()
// {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//   // Fill in the cloud data
//   cloud->width  = 5;
//   cloud->height = 1;
//   cloud->points.resize (cloud->width * cloud->height);

//   for (auto& point: *cloud)
//   {
//     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }

//   std::cerr << "Cloud before filtering: " << std::endl;
//   for (const auto& point: *cloud)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;

//   // Create the filtering object
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (.0, 1.0);
//   //pass.setFilterLimitsNegative (true);
//   pass.filter (*cloud_filtered);

//   std::cerr << "Cloud after filtering: " << std::endl;
//   for (const auto& point: *cloud_filtered)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;

//   return (0);
// }

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main ()
{
     pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
     pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
     pcl::PCDReader reader;
     // Replace the path below with the path where you saved your file
     reader.read("../../../data/asia_dragon/data.pcd", *cloud); // Remember to download the file first!

     std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
          << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

     // Create the filtering object
     pcl::PassThrough<pcl::PCLPointCloud2> pass;
     pass.setInputCloud (cloud);
     pass.setFilterFieldName ("y");
     pass.setFilterLimits (-10.0, 10.0);
     // pass.setFilterLimitsNegative (true);
     pass.filter (*cloud_filtered);

     std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
          << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
     pcl::PCDWriter writer;
     writer.write("../data/output/data_00_10_inside.pcd", *cloud_filtered,
          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

     return (0);
}