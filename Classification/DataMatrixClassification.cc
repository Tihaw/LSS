#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../LSS/LShapeDetector.hh"
#include "../LSS/OpenCVcfg3.1.0.hh"

/************************************************************************/
/*  lss lib link*/
/************************************************************************/
#ifdef _DEBUG
#define lnkLIB(name) name "d"
#else
#define lnkLIB(name) name
#endif

#pragma comment( lib, lnkLIB("LSS"))

using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

using cv::ml::SVM;
using cv::HOGDescriptor;
using cv::Mat;
using cv::Point2f;
using cv::Ptr;
using cv::Rect;

using lss::LShape;
using lss::LShapeDetector;

void detectTarget(const Mat &img, const Size &win_size, const Ptr<SVM> &svm, const Ptr<LShapeDetector> &lsd, const HOGDescriptor &hog);

//50¡Á50 pixel^2 DataMatrix
const Point2f DataMatrix[3] = { Point2f(0, 0), Point2f(0, 50), Point2f(50, 50) };

void detect(const vector< string > &imgsPath, const string &svm_fname, const Size &win_size, const Size &block_size, const Size &block_stride, const Size &cell_size, int nbins)
{
	{
		//create and init SVM
		Ptr<SVM> svm;
		svm = cv::ml::StatModel::load<SVM>(svm_fname);

		//create and init LShapeDetector
		Ptr<LShapeDetector> lsd = lss::createLShapeDetector(10, 1000);

		//create and init HOGDescriptor
		HOGDescriptor hog(win_size, block_size, block_stride, cell_size, nbins);
		
		Mat img;
		for (vector< string >::const_iterator cit = imgsPath.begin(); cit != imgsPath.end(); ++cit)
		{
			img.release();
			img = imread(*cit, IMREAD_GRAYSCALE);
			
			cout << endl;
			cout << "-----------------------------------------" << endl;
			cout << "current image : " << *cit << endl;

			detectTarget(img, win_size, svm, lsd, hog);
		}// for each imgsPath

	}//ptr block
}

Rect lshape2rect(const LShape &lshape)
{
	double x_min, y_min, x_max, y_max, length;
	x_min = std::min(lshape.vertex[0].x, lshape.vertex[1].x);
	x_min = std::min(x_min, lshape.vertex[2].x);
	y_min = std::min(lshape.vertex[0].y, lshape.vertex[1].y);
	y_min = std::min(y_min, lshape.vertex[2].y);

	x_max = std::max(lshape.vertex[0].x, lshape.vertex[1].x);
	x_max = std::max(x_max, lshape.vertex[2].x);
	y_max = std::max(lshape.vertex[0].y, lshape.vertex[1].y);
	y_max = std::max(y_max, lshape.vertex[2].y);

	length = std::max(x_max - x_min, y_max - y_min);

	return cv::Rect_<double>(x_min, y_min, length, length);
}

/************************************************************************/
/*  detect the object using SVM*/
/************************************************************************/
void detectTarget(const Mat &img, const Size &win_size, const Ptr<SVM> &svm, const Ptr<LShapeDetector> &lsd, const HOGDescriptor &hog)
{
	Mat show;
	cv::cvtColor(img, show, CV_GRAY2BGR);

	//1st stage----target pre-select
	std::vector<LShape> lshapes;
	lsd->detect(img, lshapes);

	cout << "find " << lshapes.size() << " L shapes" << endl;

	//2nd stage----target re-classification
	int count = 1, dmcount = 0;
	Mat affineImg, affineM;
	vector< float > descriptor;
	for (vector< LShape >::const_iterator lshapeIt = lshapes.begin(); lshapeIt != lshapes.end(); ++lshapeIt)
	{
		Point2f imgLshape[3] = { lshapeIt->vertex[0], lshapeIt->vertex[1], lshapeIt->vertex[2] };
		affineM = cv::getAffineTransform(imgLshape, DataMatrix);
		cv::warpAffine(img, affineImg, affineM, win_size);

		cout << "\r" << count++ << "/" << lshapes.size();

		//extract hog feature
		descriptor.clear();
		hog.compute(affineImg, descriptor);

		//svm predict
		if (svm->predict(descriptor) > 0)
		{
			++dmcount;
			cv::rectangle(show, lshape2rect(*lshapeIt), Scalar(0, 255, 0), 2);
		}

#ifdef _DEBUG
		cv::imshow("affine img", affineImg);
		cv::waitKey(10);
#endif //_DEBUG
	}//for each lShape

	cout << endl << "find " << lshapes.size() << "DataMatrix codes" << endl;
	imshow("result", show);
	waitKey(0);
	cout << endl << "-----------------------------------------" << endl;
}

/************************************************************************/
/*  fetch image files under the directory
input
path	:	where to find
output
files	:	file lists stored in vector*/
/************************************************************************/
void getImgFiles(const string &path, vector<string>& files)
{
	string savename = "u7i8y6d";
	
	/*generate some random characters for the file name*/
	int temp = abs(static_cast<int>(getTickCount()));
	temp = temp % 10;
	savename.push_back(static_cast<char>(temp + 'a'));
	temp = abs(static_cast<int>(getTickCount()));
	temp = temp % 10;
	savename.push_back(static_cast<char>(temp + 'a'));
	temp = abs(static_cast<int>(getTickCount()));
	temp = temp % 10;
	savename.push_back(static_cast<char>(temp + 'a'));

	string cmdStr = "dir ";
	string dirs(path.c_str());
	dirs.append("\\*.jpg ");
	dirs.append(path);
	dirs.append("\\*.bmp ");
	dirs.append(path);
	dirs.append("\\*.tiff ");
	dirs.append(path);
	dirs.append("\\*.png ");

	cmdStr.append(dirs);		
	cmdStr.append(" /B /S > ");
	cmdStr.append(savename);
	cout << cmdStr;
	system("cd ");
	system(cmdStr.c_str());

	ifstream fin(savename);
	string  s;
	while (fin >> s)
	{
		cout << "Read from file: " << s << endl;
		files.push_back(s);
	}
	fin.close();

	string deleteCmd = "del ";
	deleteCmd.append(savename);
	system(deleteCmd.c_str());
}