#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <math.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;
typedef pcl::PointCloud<pcl::PointNormal> point_cloud_rgb_t;

void RandomMapGenerate() {
  point_cloud_rgb_t::Ptr cloudMap (new point_cloud_rgb_t);

  // pcl::search::KdTree<pcl::PointNormal> kdtreeLocalMap;
  pcl::KdTreeFLANN<pcl::PointNormal> kdtreeLocalMap;

  default_random_engine eng(6);
  // default_random_engine eng(rd()); 
  uniform_real_distribution<double> rand_x;
  uniform_real_distribution<double> rand_y;
  uniform_real_distribution<double> rand_w;
  uniform_real_distribution<double> rand_h;
  uniform_real_distribution<double> rand_inf;

  int _obs_num=105, circle_num_=40;

  double _x_size=42.0, _y_size=40.0, z_l_=0.7, z_h_=0.8, _w_l=1.0, _w_h=1.3, _h_l=0.5, _h_h=3.0, radius_l_=0.7, radius_h_=0.5, _resolution = 0.1, theta_=0.5;
  double _x_l, _x_h, _y_l, _y_h;

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  bool _map_ok = false;

  uniform_real_distribution<double> rand_radius_;
  uniform_real_distribution<double> rand_radius2_;
  uniform_real_distribution<double> rand_theta_;
  uniform_real_distribution<double> rand_z_;

  pcl::PointNormal pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          if(pt_random.z<0) continue;//hzc
          cloudMap->points.push_back(pt_random);
        }
      }
  }

  cerr << "Generated polar obs" << endl;

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                            ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            if(pt_random.z<0) continue;//hzc
            cloudMap->push_back(pt_random);
          }
    }
  }

  cerr << "Generated circle obs" << endl;

  cloudMap->width = cloudMap->points.size();
  cloudMap->height = 1;
  cloudMap->is_dense = true;

  kdtreeLocalMap.setInputCloud(cloudMap->makeShared());

  _map_ok = true;

  pcl::io::savePCDFile<pcl::PointNormal>("../data/output/random_forest.pcd", *cloudMap);
}

int main(int argc, char** argv)
{
  RandomMapGenerate();
  return 0;
}
