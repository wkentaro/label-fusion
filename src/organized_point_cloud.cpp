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

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    printf("Usage: %s DATA_PATH\n", argv[0]);
    return 1;
  }
  std::string data_path(argv[1]);

  for (int frame_idx = 0; frame_idx < 15; frame_idx++)
  {
    // frame_idx
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
    std::cout << "frame-" + curr_frame_prefix.str() << std::endl << std::endl;

    // cam_info: intrinsic parameter of color camera
    std::string cam_K_file = data_path + "/camera-intrinsics.color.txt";
    Eigen::Matrix3f cam_K = utils::loadMatrixFromFile(cam_K_file, 3, 3);
    std::cout << "cam_K" << std::endl << cam_K << std::endl << std::endl;

    // img file
    std::string img_file = data_path + "/frame-" + curr_frame_prefix.str() + ".color.png";
    cv::Mat img = cv::imread(img_file);

    // depth file
    std::string depth_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
    cv::Mat depth = utils::loadDepthFile(depth_file);

    // pose: world -> camera
    std::string pose_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    Eigen::Matrix4f cam_pose = utils::loadMatrixFromFile(pose_file, 4, 4);
    std::cout << "cam_pose" << std::endl << cam_pose << std::endl << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.resize(depth.rows * depth.cols);
    cloud->height = depth.rows;
    cloud->width = depth.cols;
#pragma omp parallel for
    for (size_t v = 0; v < depth.rows; v++)
    {
      for (size_t u = 0; u < depth.cols; u++)
      {
        float d = depth.at<float>(v, u);

        pcl::PointXYZRGB pt;
        if (std::isnan(d))
        {
          pt = pcl::PointXYZRGB();
        }
        else
        {
          cv::Vec3b color = img.at<cv::Vec3b>(v, u);
          pt = utils::depthToPoint(cam_pose, cam_K, u, v, d, color[2], color[1], color[0]);
        }
        cloud->at(u, v) = pt;
      }
    }

    std::string out_file = "frame-" + curr_frame_prefix.str() + ".color.pcd";
    std::cout << "Wrote point cloud result to: " << out_file << std::endl;
    pcl::io::savePCDFile(out_file, *cloud);
  }
}
