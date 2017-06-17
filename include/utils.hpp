#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace utils
{

// Load an M x N matrix from a text file (numbers delimited by spaces/tabs)
// Return the matrix as a float vector of the matrix in row-major order
Eigen::MatrixXf loadMatrixFromFile(std::string filename, int M, int N)
{
  Eigen::MatrixXf matrix(M, N);
  FILE *fp = fopen(filename.c_str(), "r");
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

float get_color(int c, int x, int max)
{
  float colors[6][3] = { {1,0,1}, {0,0,1}, {0,1,1}, {0,1,0}, {1,1,0}, {1,0,0} };
  float ratio = ((float)x/max)*5;
  int i = floor(ratio);
  int j = ceil(ratio);
  ratio -= i;
  float r = (1-ratio) * colors[i][c] + ratio*colors[j][c];
  return r;
}

cv::Scalar get_label_color(unsigned int label_id, unsigned int n_label)
{
  unsigned int offset = label_id * 123457 % n_label;
  float red = get_color(2, offset, n_label);
  float green = get_color(1, offset, n_label);
  float blue = get_color(0, offset, n_label);
  return cv::Scalar(red, green, blue);
}

cv::Mat loadSegmFile(std::string filename, int H, int W)
{
  cv::Mat segm = cv::imread(filename, 0);
  if (segm.empty())
  {
    std::cout << "Error: segm image file not read!" << std::endl;
    cv::waitKey(0);
  }
  // cv::imshow("segm", segm);
  // cv::waitKey(0);
  for (int j = 0; j < H; ++j)
  {
    for (int i = 0; i < W; ++i) {
      segm.at<unsigned char>(j, i) = segm.at<unsigned char>(j, i) / 6;
    }
  }
  return segm;
}

}  // namespace utils
