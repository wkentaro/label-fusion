#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <octomap/CountingOcTree.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utils.hpp"

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    printf("Usage: %s DATA_DIR\n", argv[0]);
    return 1;
  }
  std::string data_path(argv[1]);

  int n_views = 15;

  octomap::CountingOcTree octree(/*resolution=*/0.01);

  // cam_info: intrinsic parameter of color camera
  std::string cam_K_file = data_path + "/camera-intrinsics.txt";
  Eigen::MatrixXf cam_K = utils::loadMatrixFromFile(cam_K_file, 3, 3);
  std::cout << "cam_K" << std::endl;
  std::cout << cam_K << std::endl;
  std::cout << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (int frame_idx = 0; frame_idx < n_views; frame_idx++)
  {
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
    std::cout << "frame-" + curr_frame_prefix.str() << std::endl;
    std::cout << std::endl;

    std::string mask_file = data_path + "/frame-" + curr_frame_prefix.str() + ".mask.png";
    cv::Mat mask = cv::imread(mask_file, 0);

    // pose: world -> camera
    std::string pose_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    Eigen::MatrixXf cam_pose = utils::loadMatrixFromFile(pose_file, 4, 4);
    std::cout << "cam_pose" << std::endl;
    std::cout << cam_pose << std::endl;
    std::cout << std::endl;

    // camera origin
    Eigen::Vector4f origin_(0, 0, 0, 1);
    origin_ = cam_pose * origin_;
    Eigen::Vector3f origin(origin_(0), origin_(1), origin_(2));

    // visualize camera origin
    pcl::PointXYZRGB pt;
    pt.x = origin(0);
    pt.y = origin(1);
    pt.z = origin(2);
    pt.r = 255;
    pt.g = 0;
    pt.b = 0;
    cloud.push_back(pt);

    octomap::KeySet occupied_cells;
    for (int v = 0; v < mask.rows; v+=10)
    {
      for (int u = 0; u < mask.cols; u+=10)
      {
        // printf("u: %d, v: %d\n", u, v);

        Eigen::Vector3f uv(u, v, 1);
        uv = cam_K.inverse() * uv;
        Eigen::Vector4f direction_(uv(0), uv(1), uv(2), 1);
        direction_ = cam_pose * direction_;
        Eigen::Vector3f direction(direction_(0), direction_(1), direction_(2));

        // visualize ray direction
        pcl::PointXYZRGB pt;
        pt.x = direction(0);
        pt.y = direction(1);
        pt.z = direction(2);
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;
        cloud.push_back(pt);

        if (mask.at<unsigned char>(v, u) > 127)
        {
          cv::Mat viz = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
          cv::cvtColor(mask, viz, cv::COLOR_GRAY2BGR);
          cv::circle(viz, cv::Point(u, v), 20, cv::Scalar(0, 255, 0), -1);
          cv::imshow("viz", viz);
          cv::waitKey(1);

          // ray direction
          octomap::point3d pt_origin(origin(0), origin(1), origin(2));
          octomap::point3d pt_direction(direction(0), direction(1), direction(2));
          octomap::KeyRay key_ray;
          octree.computeRayKeys(pt_origin, pt_direction, key_ray);
          occupied_cells.insert(key_ray.begin(), key_ray.end());
        }
      }
    }
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    {
      octree.updateNode(*it);
    }
  }

  // visualize 3d segmentation
  octomap::point3d_list node_centers;
  octree.getCentersMinHits(node_centers, static_cast<int>(0.95 * n_views));
  for (octomap::point3d_list::iterator it = node_centers.begin(), end = node_centers.end(); it != end; ++it)
  {
    double x = (*it).x();
    double y = (*it).y();
    double z = (*it).z();
    pcl::PointXYZRGB pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.r = 0;
    pt.g = 255;
    pt.b = 0;
    cloud.push_back(pt);
  }
  std::string out_file("out_mask_fusion.ply");
  pcl::io::savePLYFile(out_file, cloud);
  std::cout << "Wrote mask fusion result to: " << out_file << std::endl;
}
