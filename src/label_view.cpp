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

int
main(int argc, const char** argv)
{
  if (argc != 2)
  {
    printf("Usage: %s DATA_PATH\n", argv[0]);
    return 1;
  }
  std::string data_path(argv[1]);

  int n_views = 15;
  unsigned int n_label = 40;

  // cam_info: intrinsic parameter of color camera
  std::string cam_K_file = data_path + "/camera-intrinsics.color.txt";
  Eigen::Matrix3f cam_K = utils::loadMatrixFromFile(cam_K_file, 3, 3);
  std::cout << "cam_K" << std::endl << cam_K << std::endl << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_segm;
  for (int frame_idx = 0; frame_idx < n_views; frame_idx++)
  {
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
    std::cout << "frame-" + curr_frame_prefix.str() << std::endl << std::endl;

    // img file
    std::string img_file = data_path + "/frame-" + curr_frame_prefix.str() + ".color.png";
    cv::Mat img = cv::imread(img_file);

    // segmentation file
    std::string segm_file = data_path + "/frame-" + curr_frame_prefix.str() + ".segm.png";
    cv::Mat segm = utils::loadSegmFile(segm_file);
    cv::Mat segm_viz = utils::colorizeLabel(segm, /*n_label=*/n_label);

    // depth file
    std::string depth_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
    cv::Mat depth = utils::loadDepthFile(depth_file);
    cv::Mat depth_viz = utils::colorizeDepth(depth);

    cv::Mat viz;
    cv::hconcat(img, depth_viz, viz);
    cv::hconcat(viz, segm_viz, viz);
    cv::resize(viz, viz, cv::Size(), 0.5, 0.5);

    cv::imshow(argv[1], viz);
    cv::waitKey(30);

    // pose: world -> camera
    std::string pose_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    Eigen::Matrix4f cam_pose = utils::loadMatrixFromFile(pose_file, 4, 4);
    std::cout << "cam_pose" << std::endl << cam_pose << std::endl << std::endl;

    // visualize camera origin
    {
      Eigen::Vector4f origin_(0, 0, 0, 1);
      origin_ = cam_pose * origin_;
      Eigen::Vector3f origin(origin_(0), origin_(1), origin_(2));
      pcl::PointXYZRGB pt(255, 0, 0);
      pt.x = origin(0);
      pt.y = origin(1);
      pt.z = origin(2);
      cloud_color.push_back(pt);
      cloud_segm.push_back(pt);
    }

#pragma omp parallel for
    for (size_t v = 0; v < depth.rows; v++)
    {
      for (size_t u = 0; u < depth.cols; u++)
      {
        float d = depth.at<float>(v, u);
        if (std::isnan(d))
        {
          continue;
        }

        cv::Vec3b color = img.at<cv::Vec3b>(v, u);
        pcl::PointXYZRGB pt_color = utils::depthToPoint(
          cam_pose, cam_K, u, v, d, color[2], color[1], color[0]);
#pragma omp critical
        cloud_color.push_back(pt_color);

        cv::Vec3b color_label = segm_viz.at<cv::Vec3b>(v, u);
        pcl::PointXYZRGB pt_segm(color_label[2], color_label[1], color_label[0]);
        pt_segm.x = pt_color.x;
        pt_segm.y = pt_color.y;
        pt_segm.z = pt_color.z;
#pragma omp critical
        cloud_segm.push_back(pt_segm);
      }
    }
  }

  std::string out_file;
  out_file = "label_view.color.pcd";
  pcl::io::savePCDFile(out_file, cloud_color);
  std::cout << "Wrote point cloud result to: " << out_file << std::endl;

  out_file = "label_view.segm.pcd";
  pcl::io::savePCDFile(out_file, cloud_segm);
  std::cout << "Wrote point cloud result to: " << out_file << std::endl;
}
