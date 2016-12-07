#include "../LSS/OpenCVcfg3.1.0.hh"
#include <string>
#include <vector>

using std::vector;
using std::string;

using cv::Size;

void detect(const vector< string > &imgsPath, const string &svm_fname, const Size &win_size, const Size &block_size, const Size &block_stride, const Size &cell_size, int nbins);