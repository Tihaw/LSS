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

const cv::Scalar blue(255, 0, 0);
const cv::Scalar red(0, 0, 255);
const cv::Scalar green(0, 255, 0);
const cv::Scalar yellow(0, 255, 255);
const cv::Scalar cyan(255, 255, 0);
const cv::Scalar pink(255, 0, 255);
const int Delay_NoWait = 10;
const int Delay_WaitKey = 0;

std::stringstream ss;

void searchTarget(const Mat &img, const Size &win_size, const Ptr<SVM> &svm, const Ptr<LShapeDetector> &lsd, const HOGDescriptor &hog, std::vector<LShape> &lshapes);

void showImg(const string &wname, const Mat &show, int delay);

//50¡Á50 pixel^2 DataMatrix
const Point2f DataMatrix[3] = { Point2f(0, 0), Point2f(0, 50), Point2f(50, 50) };

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector)
{
	// get the support vectors
	Mat sv = svm->getSupportVectors();
	const int sv_total = sv.rows;
	// get the decision function
	Mat alpha, svidx;
	double rho = svm->getDecisionFunction(0, alpha, svidx);

	//CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
	CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
		(alpha.type() == CV_32F && alpha.at<float>(0) == 1.f));
	CV_Assert(sv.type() == CV_32F);
	hog_detector.clear();

	hog_detector.resize(sv.cols + 1);
	memcpy(&hog_detector[0], sv.ptr(), sv.cols*sizeof(hog_detector[0]));
	hog_detector[sv.cols] = (float)-rho;
}

bool classifyImg(const Mat &img, const Size & win_size, const Ptr<SVM> &svm, const HOGDescriptor &hog, vector< float > &descriptor)
{
	//extract hog feature
	descriptor.clear();
	
	//svm predict
	hog.compute(img, descriptor);
	float a = svm->predict(descriptor);
	if (1 == a)
	{
		return true;
	}
	else if (-1 == a)
	{
		return false;
	}

	return false;
}

int hardEXP_count = 0;
string hardEXP_dir = "hard/";
void collectHardEXP(const Mat &img, const Size & win_size, const Ptr<LShapeDetector> &lsd, std::vector<LShape> &lshapes)
{
	//target PRE-select
	lshapes.clear();
	lsd->detect(img, lshapes);

	//collect hard examples
	Mat affineImg, affineM;
	vector< float > descriptor;//prepare vector
	for (vector< LShape >::const_iterator lshapeIt = lshapes.begin(); lshapeIt != lshapes.end(); ++lshapeIt)
	{
		Point2f imgLshape[3] = { lshapeIt->vertex[0], lshapeIt->vertex[1], lshapeIt->vertex[2] };
		affineM = cv::getAffineTransform(imgLshape, DataMatrix);
		cv::warpAffine(img, affineImg, affineM, win_size, cv::InterpolationFlags::INTER_LINEAR);
		
		ss.str("");//clear before usage
		ss << hardEXP_dir << hardEXP_count++ << ".bmp";
		imwrite(ss.str(), affineImg);
	}//for each lShape
}

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

			if (win_size.width >= img.cols || win_size.height >= img.rows)
			{
				vector< float > descriptor;
				descriptor.reserve(hog.getDescriptorSize());
				descriptor.clear();
				if (true == classifyImg(img, win_size, svm, hog, descriptor))
				{
					rectangle(img, Rect(0,0,img.size().width, img.size().height), green, 2);
					showImg("result", img, Delay_WaitKey);
				}
			}
			else
			{
				std::vector<LShape> lshapes;
				lshapes.reserve(100);
				lshapes.clear();
				//searchTarget(img, win_size, svm, lsd, hog, lshapes);
				collectHardEXP(img, win_size, lsd, lshapes);
			}
		}// for each imgsPath
	}//ptr block
}

Rect lShape2boundingRect(const LShape &lshape)
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

void showImg(const string &wname, const Mat &show, int delay)
{
	imshow(wname, show);
	waitKey(delay);
}

void showLshapes(const vector<LShape> &lshapes, const Mat &show, int delay)
{
	for (std::vector<LShape>::const_iterator it = lshapes.begin(); it != lshapes.end(); ++it)
	{
		line(show, it->vertex[0], it->vertex[1], yellow, 2);
		line(show, it->vertex[1], it->vertex[2], yellow, 2);
	}

	showImg("LShapes", show, delay);
}

/************************************************************************/
/*  detect the object using SVM*/
/************************************************************************/
void searchTarget(const Mat &img, const Size &win_size, const Ptr<SVM> &svm, const Ptr<LShapeDetector> &lsd, const HOGDescriptor &hog, std::vector<LShape> &lshapes)
{
	Mat show;
	cv::cvtColor(img, show, CV_GRAY2BGR);

	//1st stage----target PRE-select
	lshapes.clear();
	lsd->detect(img, lshapes);

	cout << "find " << lshapes.size() << " L shapes" << endl;

#ifdef _DEBUG
	showLshapes(lshapes, show, Delay_NoWait);
#endif //_DEBUG

	//2nd stage----target RE-classification
	int count = 1, dmcount = 0;
	Mat affineImg, affineM;
	vector< float > descriptor;//prepare vector
	descriptor.reserve(hog.getDescriptorSize());//reserve enghou space
	for (vector< LShape >::const_iterator lshapeIt = lshapes.begin(); lshapeIt != lshapes.end(); ++lshapeIt)
	{
		Point2f imgLshape[3] = { lshapeIt->vertex[0], lshapeIt->vertex[1], lshapeIt->vertex[2] };
		affineM = cv::getAffineTransform(imgLshape, DataMatrix);
		cv::warpAffine(img, affineImg, affineM, win_size);

		cout << "\r" << count++ << "/" << lshapes.size();

		//clear before usage
		descriptor.clear();
		if (classifyImg(affineImg, win_size, svm, hog, descriptor))
		{
			++dmcount;
			cv::rectangle(show, lShape2boundingRect(*lshapeIt), green, 3);
		}

#ifdef _DEBUG
		showImg("affine img", affineImg, Delay_NoWait);
#endif //_DEBUG
	}//for each lShape

	cout << endl << "find " << dmcount << " DataMatrix codes" << endl;
	showImg("result", show, Delay_WaitKey);
	cout << endl << "-----------------------------------------" << endl;
}

/************************************************************************/
/*  fetch image files under the directory
input
path	:	where to find
output
files	:	file lists stored in vector
CANNOT WORK IN STEP DEBUG MODE?!*/
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