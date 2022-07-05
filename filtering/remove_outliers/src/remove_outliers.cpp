// #include <iostream>
// #include <pcl/point_types.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/conditional_removal.h>

// int main (int argc, char** argv)
// {
//   if (argc != 2)
//   {
//     std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
//     exit(0);
//   }
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//   // Fill in the cloud data
//   cloud->width  = 5;
//   cloud->height = 1;
//   cloud->resize (cloud->width * cloud->height);

//   for (auto& point: *cloud)
//   {
//     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }

//   if (strcmp(argv[1], "-r") == 0){
//     pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//     // build the filter
//     outrem.setInputCloud(cloud);
//     outrem.setRadiusSearch(0.8);
//     outrem.setMinNeighborsInRadius (2);
//     outrem.setKeepOrganized(true);
//     // apply filter
//     outrem.filter (*cloud_filtered);
//   }
//   else if (strcmp(argv[1], "-c") == 0){
//     // build the condition
//     pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
//       pcl::ConditionAnd<pcl::PointXYZ> ());

//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//       pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//       pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//       pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.5)));
//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//       pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.0)));

//     // build the filter
//     pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//     condrem.setCondition (range_cond);
//     condrem.setInputCloud (cloud);
//     condrem.setKeepOrganized(true);
//     // apply filter
//     condrem.filter (*cloud_filtered);
//   }
//   else{
//     std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
//     exit(0);
//   }
//   std::cerr << "Cloud before filtering: " << std::endl;
//   for (const auto& point: *cloud)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;
//   // display pointcloud after filtering
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
#include <pcl/filters/radius_outlier_removal.h>

//Removing outliers using a Conditional or RadiusOutlier removal
//http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Reading *.PCD files (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
  // pcl::io::loadPCDFile<pcl::PointXYZ> ("../../../data/asia_dragon/data.pcd", *cloud);
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read("../../voxel_grid/data/output/data_1.pcd", *cloud); // Remember to download the file first!
  // number of points output
  std::cerr << "Loaded " << cloud->width * cloud->height  << std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(2);
  outrem.setMinNeighborsInRadius (20);
  // outrem.setKeepOrganized(true);
  // apply filter
  outrem.filter (*cloud_filtered);

  // number of points output  
  std::cerr << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;

  // save 
  pcl::io::savePCDFile<pcl::PointXYZ>("../data/output/data_radius.pcd", *cloud_filtered); // Default binary mode save

  return 0;
}