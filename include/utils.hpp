#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utils {

// Load an M x N matrix from a text file (numbers delimited by spaces/tabs)
// Return the matrix as a float vector of the matrix in row-major order
Eigen::MatrixXf loadMatrixFromFile(std::string filename, int M, int N) {
  Eigen::MatrixXf matrix(M, N);
  FILE *fp = fopen(filename.c_str(), "r");
  for (int j = 0; j < M; j++) {
    for (int i = 0; i < N; i++) {
      float tmp;
      int iret = fscanf(fp, "%f", &tmp);
      matrix(j, i) = tmp;
    }
  }
  fclose(fp);
  return matrix;
}

} // namespace utils
