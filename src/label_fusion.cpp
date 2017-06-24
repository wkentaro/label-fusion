// ----------------------------------------------------------------------------
// Copyright (c) 2017, Kentaro Wada, University of Tokyo
// ----------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <iomanip>
#include <map>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <octomap/LabelCountingOcTree.h>
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
  unsigned int n_label = 40;
  unsigned int ksize = 10;

  octomap::LabelCountingOcTree octree(/*resolution=*/resolution, /*n_label=*/n_label);

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

    // segmentation file
    std::string segm_file = data_path + "/frame-" + curr_frame_prefix.str() + ".segm.png";
    cv::Mat segm = utils::loadSegmFile(segm_file);
    // cv::Mat segm_viz = utils::colorizeLabel(segm, #<{(|n_label=|)}>#n_label);
    // cv::imshow("segm_viz", segm_viz);
    // cv::waitKey(0);

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

    std::map<unsigned int, octomap::KeySet> occupied_cells;
    octomap::KeySet occupied_cells_all;
    octomap::KeySet unoccupied_cells;
#pragma omp parallel for
    for (int v = 0; v < segm.rows; v += ksize)
    {
      for (int u = 0; u < segm.cols; u += ksize)
      {
        float d = std::numeric_limits<float>::quiet_NaN();
        if (use_depth)
        {
          d = depth.at<float>(v, u);
        }

        Eigen::Vector3f uv(u, v, 1);
        uv = cam_K.inverse() * uv;
        Eigen::Vector4f direction_(uv(0), uv(1), uv(2), 1);  // with depth
        Eigen::Vector4f direction_far_(direction_(0), direction_(1), direction_(2), 1);  // without depth
        if (!std::isnan(d))
        {
          direction_(0) *= d;
          direction_(1) *= d;
          direction_(2) = d;
        }
        direction_ = cam_pose * direction_;
        direction_far_ = cam_pose * direction_far_;
        Eigen::Vector3f direction(direction_(0), direction_(1), direction_(2));
        Eigen::Vector3f direction_far(direction_far_(0), direction_far_(1), direction_far_(2));

        // visualize ray direction
        pcl::PointXYZRGB pt(0, 0, 255);
        pt.x = direction_far(0);
        pt.y = direction_far(1);
        pt.z = direction_far(2);
#pragma omp critical
        cloud.push_back(pt);

        octomap::point3d pt_origin(origin(0), origin(1), origin(2));
        octomap::point3d pt_direction(direction(0), direction(1), direction(2));
        octomap::point3d pt_direction_far(direction_far(0), direction_far(1), direction_far(2));
        unsigned int label_id = static_cast<unsigned int>(segm.at<unsigned char>(v, u));
        if (label_id != 0)
        {
          octomap::KeyRay key_ray;
          octree.computeRayKeys(pt_origin, pt_direction_far, key_ray);
#pragma omp critical
          occupied_cells[label_id].insert(key_ray.begin(), key_ray.end());
        }
        if (!std::isnan(d))
        {
          octomap::KeyRay key_ray;
          if (octree.computeRayKeys(pt_origin, pt_direction, key_ray))
          {
#pragma omp critical
            unoccupied_cells.insert(key_ray.begin(), key_ray.end());
          }
        }
      }
    }
    for (octomap::KeySet::iterator it = unoccupied_cells.begin(), end = unoccupied_cells.end();
         it != end; ++it)
    {
      if (occupied_cells_all.find(*it) == occupied_cells_all.end())
      {
        octree.updateNode(*it, /*label=*/-1, /*hit=*/false, /*reset=*/true);
      }
    }
    for (unsigned int label_id = 1; label_id < n_label; label_id++)
    {
      for (octomap::KeySet::iterator it = occupied_cells[label_id].begin(), end = occupied_cells[label_id].end();
           it != end; ++it)
      {
        octree.updateNode(*it, /*label=*/label_id);
      }
    }
  }

  // visualize 3d segmentation
  octomap::point3d_list node_centers;
  std::vector<unsigned int> node_labels;
  octree.getCentersMinHits(node_centers, node_labels, static_cast<int>(threshold * n_views));
  unsigned int index = 0;
  for (octomap::point3d_list::iterator it = node_centers.begin(), end = node_centers.end(); it != end; ++it)
  {
    unsigned int label_id = node_labels[index];
    cv::Scalar color = utils::get_label_color(label_id, /*n_label=*/n_label);
    pcl::PointXYZRGB pt(color[0] * 255, color[1] * 255, color[2] * 255);
    pt.x = (*it).x();
    pt.y = (*it).y();
    pt.z = (*it).z();
    cloud.push_back(pt);
    index += 1;
  }
  assert(index == node_labels.size());
  std::string out_file("label_fusion.pcd");
  pcl::io::savePCDFile(out_file, cloud);
  std::cout << "Wrote mask fusion result to: " << out_file << std::endl;
}
