#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace utils
{

Eigen::MatrixXf
loadMatrixFromFile(std::string filename, int M, int N)
{
  Eigen::MatrixXf matrix(M, N);
  FILE* fp;
  if ((fp = fopen(filename.c_str(), "r")) == 0)
  {
    throw std::invalid_argument("File does not exist.");
  }
  for (int j = 0; j < M; j++)
  {
    for (int i = 0; i < N; i++)
    {
      float tmp;
      int iret = fscanf(fp, "%f", &tmp);
      matrix(j, i) = tmp;
    }
  }
  fclose(fp);
  return matrix;
}

float
get_color(int c, int x, int max)
{
  float colors[6][3] = { {1,0,1}, {0,0,1}, {0,1,1}, {0,1,0}, {1,1,0}, {1,0,0} };
  float ratio = ((float)x/max)*5;
  int i = floor(ratio);
  int j = ceil(ratio);
  ratio -= i;
  float r = (1-ratio) * colors[i][c] + ratio*colors[j][c];
  return r;
}

cv::Scalar
get_label_color(unsigned int label_id, unsigned int n_label)
{
  unsigned int offset = label_id * 123457 % n_label;
  float red = get_color(2, offset, n_label);
  float green = get_color(1, offset, n_label);
  float blue = get_color(0, offset, n_label);
  return cv::Scalar(red, green, blue);
}

cv::Mat
loadSegmFile(std::string filename)
{
  cv::Mat segm = cv::imread(filename, 0);
  if (segm.empty())
  {
    std::cout << "Error: segm image file not read!" << std::endl;
    cv::waitKey(0);
  }
  // cv::imshow("segm", segm);
  // cv::waitKey(0);
  for (int j = 0; j < segm.rows; ++j)
  {
    for (int i = 0; i < segm.cols; ++i) {
      segm.at<unsigned char>(j, i) = segm.at<unsigned char>(j, i) / 6;
    }
  }
  return segm;
}

cv::Mat
loadDepthFile(std::string filename)
{
  cv::Mat depth_raw = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
  if (depth_raw.empty())
  {
    std::cout << "Error: depth image file not read!" << std::endl;
    cv::waitKey(0);
  }
  cv::Mat depth(depth_raw.rows, depth_raw.cols, CV_32FC1);
  for (size_t j = 0; j < depth_raw.rows; ++j)
  {
    for (size_t i = 0; i < depth_raw.cols; ++i)
    {
      float tmp = static_cast<float>(depth_raw.at<unsigned short>(j, i) >> 3) / 1e4;
      if (tmp < 0.3)  // nan for too small depth
      {
        depth.at<float>(j, i) = std::numeric_limits<float>::quiet_NaN();
      }
      else
      {
        depth.at<float>(j, i) = tmp;
      }
    }
  }
  return depth;
}

cv::Mat
colorizeDepth(cv::Mat depth)
{
    double min_value, max_value;
    cv::minMaxLoc(depth, &min_value, &max_value);
    cv::Mat depth_viz(depth.rows, depth.cols, CV_8UC3);
    cv::Mat(depth - min_value).convertTo(
      depth_viz, CV_8UC3, 255.0 / (max_value - min_value));
    cv::applyColorMap(depth_viz, depth_viz, 2); // JET
    for (size_t j = 0; j < depth_viz.rows; ++j)
    {
      for (size_t i = 0; i < depth_viz.cols; ++i)
      {
        if (std::isnan(depth.at<float>(j, i)))
        {
          depth_viz.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
        }
      }
    }
    return depth_viz;
}

cv::Mat
colorizeLabel(cv::Mat label, unsigned int n_label)
{
  cv::Mat label_viz = cv::Mat::zeros(label.rows, label.cols, CV_8UC3);
  for (size_t j = 0; j < label.rows; j++)
  {
    for (size_t i = 0; i < label.cols; i++)
    {
      unsigned int label_id = static_cast<unsigned int>(label.at<unsigned char>(j, i));
      assert(0 <= label_id && label_id < n_label);
      if (label_id != 0) {
        cv::Scalar color = utils::get_label_color(label_id, /*n_label=*/n_label);
        label_viz.at<cv::Vec3b>(j, i) = cv::Vec3b(color[2] * 255, color[1] * 255, color[0] * 255);
      }
    }
  }
  return label_viz;
}

}  // namespace utils
