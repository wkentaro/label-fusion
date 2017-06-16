#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace utils {

// Load an M x N matrix from a text file (numbers delimited by spaces/tabs)
// Return the matrix as a float vector of the matrix in row-major order
Eigen::MatrixXf loadMatrixFromFile(std::string filename, int M, int N) {
  Eigen::MatrixXf matrix(M, N);
  // std::vector<float> matrix;
  FILE *fp = fopen(filename.c_str(), "r");
  for (int i = 0; i < M * N; i++) {
    float tmp;
    int iret = fscanf(fp, "%f", &tmp);
    matrix(i) = tmp;
    // matrix.push_back(tmp);
  }
  fclose(fp);
  // return matrix;
  return matrix;
}

} // namespace utils


int main(int argc, char* argv[])
{
  std::string data_path = "../src/data";

  // cam_info: intrinsic parameter of color camera
  std::string cam_K_file = data_path + "/camera-intrinsics.txt";
  Eigen::MatrixXf cam_K = utils::loadMatrixFromFile(cam_K_file, 3, 3);
  std::cout << "cam_K" << std::endl;
  std::cout << cam_K << std::endl;
  std::cout << std::endl;

  for (int frame_idx = 0; frame_idx < 4; frame_idx++)
  {
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;
    std::cout << "frame-" + curr_frame_prefix.str() << std::endl;
    std::cout << std::endl;

    std::string mask_file = data_path + "/frame-" + curr_frame_prefix.str() + ".mask.png";
    cv::Mat mask = cv::imread(mask_file, 0);
    // cv::imshow("mask", mask);
    // cv::waitKey(0);

    // pose: world -> camera
    std::string pose_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    Eigen::MatrixXf cam_pose = utils::loadMatrixFromFile(pose_file, 4, 4);
    std::cout << "cam_pose" << std::endl;
    std::cout << cam_pose << std::endl;
    std::cout << std::endl;

    for (int v = 0; v < mask.rows; v++)
    {
      for (int u = 0; u < mask.cols; u++)
      {
        if (mask.at<unsigned char>(v, u) > 127)
        {
          // filled
          printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
          // http://opencv.jp/opencv-2.1/cpp/camera_calibration_and_3d_reconstruction.html
          // 3x1 = 3x3 * 3x4 * 4x1
          // pt2d = cam_K * cam_pose * pt3d;

          // ray direction
          Eigen::Vector3f direction_2d(u, v, 0);
          Eigen::Vector3f direction = cam_K.inverse() * direction_2d;
          Eigen::Vector4f direction_(direction(0), direction(1), direction(2), 1);
          direction_ = cam_pose.inverse() * direction_;
          direction << direction_(0), direction_(1), direction_(2);
          direction << 1, 0, 0;

          // camera origin
          Eigen::Vector4f origin_(0, 0, 0, 1);
          origin_ = cam_pose.inverse() * origin_;
          Eigen::Vector3f origin(origin_(0), origin_(1), origin_(2));

          std::cout << "origin: \n" << origin << std::endl << std::endl;
          std::cout << "direction: \n" << direction << std::endl << std::endl;
          printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
          return 0;
        }
      }
    }
    return 0;
  }
}
