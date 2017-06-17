#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <octomap/LabelCountingOcTree.h>
#include <opencv2/opencv.hpp>
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

  octomap::LabelCountingOcTree octree(/*resolution=*/0.01, /*n_label=*/40);

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

    std::string segm_file = data_path + "/frame-" + curr_frame_prefix.str() + ".segm.png";
    cv::Mat segm = utils::loadSegmFile(segm_file);
    cv::Mat segm_viz = cv::Mat::zeros(segm.rows, segm.cols, CV_8UC3);
    for (size_t j = 0; j < segm.rows; j++)
    {
      for (size_t i = 0; i < segm.cols; i++)
      {
        unsigned int label_id = static_cast<unsigned int>(segm.at<unsigned char>(j, i));
        assert(0 <= label_id < 40);
        if (label_id != 0) {
          cv::Scalar color = utils::get_label_color(label_id, /*n_label=*/40);
          segm_viz.at<cv::Vec3b>(j, i) = cv::Vec3b(color[2] * 255, color[1] * 255, color[0] * 255);
        }
      }
    }
    // cv::imshow("segm_viz", segm_viz);
    // cv::waitKey(0);

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
    pcl::PointXYZRGB pt(255, 0, 0);
    pt.x = origin(0);
    pt.y = origin(1);
    pt.z = origin(2);
    cloud.push_back(pt);

    std::map<unsigned int, octomap::KeySet> occupied_cells;
    //octomap::KeySet occupied_cells;
    for (int v = 0; v < segm.rows; v+=10)
    {
      for (int u = 0; u < segm.cols; u+=10)
      {
        Eigen::Vector3f uv(u, v, 1);
        uv = cam_K.inverse() * uv;
        Eigen::Vector4f direction_(uv(0), uv(1), uv(2), 1);
        direction_ = cam_pose * direction_;
        Eigen::Vector3f direction(direction_(0), direction_(1), direction_(2));

        // visualize ray direction
        pcl::PointXYZRGB pt(0, 0, 255);
        pt.x = direction(0);
        pt.y = direction(1);
        pt.z = direction(2);
        cloud.push_back(pt);

        unsigned int label_id = static_cast<unsigned int>(segm.at<unsigned char>(v, u));
        if (label_id == 0)
        {
          continue;
        }

        // cv::Mat ray_viz = cv::Mat::zeros(segm.rows, segm.cols, CV_8UC3);
        // cv::cvtColor(segm, ray_viz, cv::COLOR_GRAY2BGR);
        cv::Mat ray_viz;
        segm_viz.copyTo(ray_viz);
        cv::circle(ray_viz, cv::Point(u, v), 20, cv::Scalar(0, 255, 0), -1);
        cv::imshow("ray_viz", ray_viz);
        cv::waitKey(1);

        // ray direction
        octomap::point3d pt_origin(origin(0), origin(1), origin(2));
        octomap::point3d pt_direction(direction(0), direction(1), direction(2));
        octomap::KeyRay key_ray;
        octree.computeRayKeys(pt_origin, pt_direction, key_ray);
        occupied_cells[label_id].insert(key_ray.begin(), key_ray.end());
      }
    }
    for (unsigned int label_id = 1; label_id < 40; label_id++)
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
  octree.getCentersMinHits(node_centers, node_labels, static_cast<int>(0.95 * n_views));
  unsigned int index = 0;
  for (octomap::point3d_list::iterator it = node_centers.begin(), end = node_centers.end(); it != end; ++it)
  {
    unsigned int label_id = node_labels[index];
    cv::Scalar color = utils::get_label_color(label_id, /*n_label=*/40);
    pcl::PointXYZRGB pt(color[0] * 255, color[1] * 255, color[2] * 255);
    pt.x = (*it).x();
    pt.y = (*it).y();
    pt.z = (*it).z();
    cloud.push_back(pt);
    index += 1;
  }
  assert(index == node_labels.size());
  std::string out_file("out_label_fusion.ply");
  pcl::io::savePLYFile(out_file, cloud);
  std::cout << "Wrote mask fusion result to: " << out_file << std::endl;
}
