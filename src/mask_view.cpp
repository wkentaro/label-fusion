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

  // cam_info: intrinsic parameter of color camera
  std::string cam_K_file = data_path + "/camera-intrinsics.color.txt";
  Eigen::Matrix3f cam_K = utils::loadMatrixFromFile(cam_K_file, 3, 3);
  std::cout << "cam_K" << std::endl << cam_K << std::endl << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_mask;
  for (int frame_idx = 0; frame_idx < n_views; frame_idx++)
  {
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
    std::cout << "frame-" + curr_frame_prefix.str() << std::endl << std::endl;

    // img file
    std::string img_file = data_path + "/frame-" + curr_frame_prefix.str() + ".color.png";
    cv::Mat img = cv::imread(img_file);

    // mask file
    std::string mask_file = data_path + "/frame-" + curr_frame_prefix.str() + ".mask.png";
    cv::Mat mask = cv::imread(mask_file, 0);
    cv::Mat mask_viz;
    cv::cvtColor(mask, mask_viz, cv::COLOR_GRAY2BGR);

    // depth file
    std::string depth_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
    cv::Mat depth = utils::loadDepthFile(depth_file);
    cv::Mat depth_viz = utils::colorizeDepth(depth);

    cv::Mat viz;
    cv::hconcat(img, depth_viz, viz);
    cv::hconcat(viz, mask_viz, viz);
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
      cloud_mask.push_back(pt);
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

        uint8_t m = mask.at<uint8_t>(v, u);
        pcl::PointXYZRGB pt_mask(m, m, m);
        pt_mask.x = pt_color.x;
        pt_mask.y = pt_color.y;
        pt_mask.z = pt_color.z;
        #pragma omp critical
        cloud_mask.push_back(pt_mask);
      }
    }
  }

  std::string out_file;
  out_file = "mask_view.color.pcd";
  pcl::io::savePCDFile(out_file, cloud_color);
  std::cout << "Wrote point cloud result to: " << out_file << std::endl;

  out_file = "mask_view.mask.pcd";
  pcl::io::savePCDFile(out_file, cloud_mask);
  std::cout << "Wrote point cloud result to: " << out_file << std::endl;
}
