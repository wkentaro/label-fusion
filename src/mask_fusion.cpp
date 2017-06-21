#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <octomap/CountingOcTree.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utils.hpp"

void
usage(const char** argv)
{
  printf("Usage: %s [--depth] DATA_PATH\n", argv[0]);
}

int
main(int argc, const char** argv)
{
  bool use_depth;
  std::string data_path;
  if (argc == 3 && (std::string(argv[1]) == "--depth"))
  {
    use_depth = true;
    data_path = std::string(argv[2]);
  }
  else if (argc == 2)
  {
    use_depth = false;
    data_path = std::string(argv[1]);
  }
  else
  {
    usage(argv);
    return 1;
  }

  int n_views = 15;
  double resolution = 0.01;
  double threshold = 0.9;
  unsigned int ksize = 10;

  octomap::CountingOcTree octree(/*resolution=*/resolution);

  // cam_info: intrinsic parameter of color camera
  std::string cam_K_file = data_path + "/camera-intrinsics.color.txt";
  Eigen::Matrix3f cam_K = utils::loadMatrixFromFile(cam_K_file, 3, 3);
  std::cout << "cam_K" << std::endl << cam_K << std::endl << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (int frame_idx = 0; frame_idx < n_views; frame_idx++)
  {
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
    std::cout << "frame-" + curr_frame_prefix.str() << std::endl << std::endl;

    // mask file
    std::string mask_file = data_path + "/frame-" + curr_frame_prefix.str() + ".mask.png";
    cv::Mat mask = cv::imread(mask_file, 0);

    cv::Mat depth;
    if (use_depth)
    {
      std::string depth_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
      depth = utils::loadDepthFile(depth_file);
      // cv::Mat depth_viz = utils::colorizeDepth(depth);
      // cv::imshow("depth_viz", depth_viz);
      // cv::waitKey(0);
    }

    // pose: world -> camera
    std::string pose_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    Eigen::Matrix4f cam_pose = utils::loadMatrixFromFile(pose_file, 4, 4);
    std::cout << "cam_pose" << std::endl << cam_pose << std::endl << std::endl;

    // camera origin
    Eigen::Vector4f origin_(0, 0, 0, 1);
    origin_ = cam_pose * origin_;
    Eigen::Vector3f origin(origin_(0), origin_(1), origin_(2));

    // visualize camera origin
    pcl::PointXYZRGB pt(255, 0, 0);
    pt.x = origin(0);
    pt.y = origin(1);
    pt.z = origin(2);
    cloud.push_back(pt);

    octomap::KeySet occupied_cells;
    octomap::KeySet unoccupied_cells;
#pragma omp parallel for
    for (int v = 0; v < mask.rows; v += ksize)
    {
      for (int u = 0; u < mask.cols; u += ksize)
      {
        float d = std::numeric_limits<float>::quiet_NaN();
        if (use_depth)
        {
          d = depth.at<float>(v, u);
        }

        Eigen::Vector3f uv(u, v, 1);
        uv = cam_K.inverse() * uv;
        Eigen::Vector4f direction_(uv(0), uv(1), uv(2), 1);
        if (!std::isnan(d))
        {
          direction_(0) *= d;
          direction_(1) *= d;
          direction_(2) = d;
        }
        direction_ = cam_pose * direction_;
        Eigen::Vector3f direction(direction_(0), direction_(1), direction_(2));

        if (std::isnan(d))
        {
          // visualize ray direction
          pcl::PointXYZRGB pt(0, 0, 255);
          pt.x = direction(0);
          pt.y = direction(1);
          pt.z = direction(2);
#pragma omp critical
          cloud.push_back(pt);
        }

        octomap::point3d pt_origin(origin(0), origin(1), origin(2));
        octomap::point3d pt_direction(direction(0), direction(1), direction(2));
        if (std::isnan(d))
        {
          if (mask.at<unsigned char>(v, u) > 127)
          {
            octomap::KeyRay key_ray;
            octree.computeRayKeys(pt_origin, pt_direction, key_ray);
#pragma omp critical
            occupied_cells.insert(key_ray.begin(), key_ray.end());
          }
        }
        else
        {
          octomap::KeyRay key_ray;
          if (octree.computeRayKeys(pt_origin, pt_direction, key_ray))
          {
#pragma omp critical
            unoccupied_cells.insert(key_ray.begin(), key_ray.end());
          }
          octomap::OcTreeKey key;
          if (octree.coordToKeyChecked(pt_direction, key))
          {
#pragma omp critical
            occupied_cells.insert(key);
          }
        }
      }
    }
    for (octomap::KeySet::iterator it = unoccupied_cells.begin(), end = unoccupied_cells.end(); it != end; ++it)
    {
      octree.updateNode(*it, /*hit=*/false, /*reset=*/true);
    }
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    {
      if (unoccupied_cells.find(*it) == unoccupied_cells.end())
      {
        octree.updateNode(*it, /*hit=*/true);
      }
    }
  }

  // visualize 3d segmentation
  octomap::point3d_list node_centers;
  octree.getCentersMinHits(node_centers, static_cast<int>(threshold * n_views));
  for (octomap::point3d_list::iterator it = node_centers.begin(), end = node_centers.end(); it != end; ++it)
  {
    pcl::PointXYZRGB pt(0, 255, 0);
    pt.x = (*it).x();
    pt.y = (*it).y();
    pt.z = (*it).z();
    cloud.push_back(pt);
  }
  std::string out_file("mask_fusion.pcd");
  pcl::io::savePCDFile(out_file, cloud);
  std::cout << "Wrote mask fusion result to: " << out_file << std::endl;
}
