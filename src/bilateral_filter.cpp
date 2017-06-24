// ----------------------------------------------------------------------------
// Copyright (c) 2017, Kentaro Wada, University of Tokyo
// ----------------------------------------------------------------------------

#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/fast_bilateral.h>

#include "utils.hpp"

int
main(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("%s IN_FILE\n", argv[0]);
    return 1;
  }
  std::string in_file(argv[1]);

  // parameters used in kinfu
  float sigma_s = 4.5f;
  float sigma_r = 0.03f;

  // load cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile(in_file, *cloud);

  // bilateral filter
  pcl::FastBilateralFilter<pcl::PointXYZRGB>::Ptr filter(new pcl::FastBilateralFilter<pcl::PointXYZRGB>);
  filter->setInputCloud(cloud);
  filter->setSigmaS(sigma_s);
  filter->setSigmaR(sigma_r);
  //
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  filter->applyFilter(*out_cloud);

  // save filtered output
  std::string basename;
  std::string ext;
  utils::splitext(in_file, basename, ext);
  std::string out_file = basename + ".bf" + ext;
  pcl::io::savePCDFile(out_file, *out_cloud);
  std::cout << "Wrote point cloud to: " << out_file << std::endl;
}
