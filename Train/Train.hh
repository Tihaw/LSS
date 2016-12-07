#include "../LSS/OpenCVcfg3.1.0.hh"
#include <string>
#include <vector>

using cv::Mat;
using std::vector;
using std::string;

void load_images(const string & prefix, const string & filename, vector< Mat > & img_lst);

void compute_hog(const vector< Mat > & img_lst, vector< Mat > & gradient_lst, const Size & _winSize, const Size & _blockSize, const Size & _blockStride, const Size & _cellSize, const int _nbins);

void train_svm(const vector< Mat > & gradient_lst, const vector< int > & labels, const string &savename);

void test_it(const Size & _winSize, const Size & _blockSize, const Size & _blockStride, const Size & _cellSize, const int _nbins);