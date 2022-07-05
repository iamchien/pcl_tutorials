#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

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
   reader.read("../../../data/table/tabletop.pcd", *cloud); // Remember to download the file first!
   // number of points output
   std::cerr << "Loaded " << cloud->width * cloud->height  << std::endl;

   // condition definition
   pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new // condition 1
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));  //eg. Values greater than 0.00 on the z-axis (GT: Greater Than)
   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new // condition 2
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));  //eg. z-axis less than 0.08 (LT: Less Than)

   //create object
   pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
   condrem.setInputCloud (cloud);        // input
   condrem.setCondition (range_cond);    // set condition  
   // condrem.setKeepOrganized(true);       //
   condrem.filter (*cloud_filtered);     // apply filter

   // number of points output  
   std::cerr << "Filtered " << cloud_filtered->width * cloud_filtered->height  << std::endl;

   // save 
   pcl::io::savePCDFile<pcl::PointXYZ>("../data/output/data_conditional.pcd", *cloud_filtered); // Default binary mode save

   return 0;
}