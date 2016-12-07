/************************************************************************/
/* this is an program for test SVM classification with HOG features		*/
/************************************************************************/
#include <fstream>
#include <string>
#include <vector>
#include <ml.hpp>

#include "DataMatrixClassification.hh"

using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

using cv::Ptr;
using cv::Size;

//HOG params keep with train
Size win_size(50, 50);
Size block_size(10, 10);
Size block_stride(10, 10);
Size cell_size(5, 5);
int nbins = 9;


int main(int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, "{help h|| show help message}"
		"{t||test.lst(with full path)}{sf||svm_file path}");
	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	string test_list = parser.get<string>("t");
	string svm_fname = parser.get<string>("sf");
	if (test_list.empty() || svm_fname.empty())
	{
		cout << "Wrong number of parameters." << endl
			<< "Usage: " << argv[0] << " --t=test.lst" << endl
			<< "example: " << argv[0] << " --t=./test.lst sf=../Train/SVM_DATA.xml" << endl;
		exit(-1);
	}

	//read image list
	vector< string > imgPathes;
	ifstream fin(test_list);
	string  s;

	while (fin >> s)
	{
		imgPathes.push_back(s);
	}
	fin.close();

	detect(imgPathes, svm_fname, win_size, block_size, block_stride, cell_size, nbins);

	return 0;
}