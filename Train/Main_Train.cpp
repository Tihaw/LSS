/************************************************************************/
/* this is an program for train SVM with HOG features                    
	revised from OpenCV sample*/
/************************************************************************/
#include <string>
#include <vector>

#include "HOGparameters.h"
#include "Train.hh"

using cv::Mat;
using std::vector;
using std::string;
using std::cout;
using std::endl;

string fname("SVM_DATA.xml");

int main(int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, "{help h|| show help message}"
		"{pd||pos_dir}{p||pos.lst}{nd||neg_dir}{n||neg.lst}");
	if (parser.has("help"))
	{
		parser.printMessage();
		exit(0);
	}

	vector< Mat > pos_lst;
	vector< Mat > neg_lst;
	vector< Mat > gradient_lst;
	vector< int > labels;

	string pos_dir = parser.get<string>("pd");
	string pos = parser.get<string>("p");
	string neg_dir = parser.get<string>("nd");
	string neg = parser.get<string>("n");
	if (pos_dir.empty() || pos.empty() || neg_dir.empty() || neg.empty())
	{
		cout << "Wrong number of parameters." << endl
			<< "Usage: " << argv[0] << " --pd=pos_dir -p=pos.lst --nd=neg_dir -n=neg.lst" << endl
			<< "example: " << argv[0] << " --pd=/INRIA_dataset/ -p=Train/pos.lst --nd=/INRIA_dataset/ -n=Train/neg.lst" << endl;
		exit(-1);
	}

	load_images(pos_dir, pos, pos_lst);
	labels.assign(pos_lst.size(), +1);

	const unsigned int old = (unsigned int)labels.size();
	load_images(neg_dir, neg, neg_lst);
	labels.insert(labels.end(), neg_lst.size(), -1);
	CV_Assert(old < labels.size());
	
	compute_hog(pos_lst, gradient_lst, win_size, block_size, block_stride, cell_size, nbins);
	compute_hog(neg_lst, gradient_lst, win_size, block_size, block_stride, cell_size, nbins);

	train_svm(gradient_lst, labels, fname);

	//test_it(win_size, block_size, block_stride, cell_size, nbins);

	return 0;
}