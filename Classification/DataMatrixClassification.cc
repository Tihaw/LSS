#include <fstream>
#include <list>
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
using std::list;
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

const int imgTargetSizeRatio = 20;
const double nms_ratio = 0.500000;

//50¡Á50 pixel^2 DataMatrix
const Point2f DataMatrix[3] = { Point2f(0, 0), Point2f(0, 50), Point2f(50, 50) };

std::stringstream ss;

namespace{

struct targetConfidence 
{
	Rect boundingBox;
	float confidence;
};

void searchTarget(const Mat &img, const Size &win_size, const Ptr<SVM> &svm, const Ptr<LShapeDetector> &lsd, const HOGDescriptor &hog, std::vector<LShape> &lshapes);

void showImg(const string &wname, const Mat &show, int delay);

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

bool classifyImg(const Mat &img, const Size & win_size, const Ptr<SVM> &svm, const HOGDescriptor &hog, vector< float > &descriptor, CV_OUT float *confidence = NULL)
{
	//try equalizeHist()
/*
	Mat temp;
	equalizeHist(img, temp);*/

	//extract hog feature
	descriptor.clear();
	
	//svm predict
	hog.compute(img, descriptor);

	//class label return
/*
	float a = svm->predict(descriptor);
	if (1 == a)
	{
		return true;
	}
	else if (-1 == a)
	{
		return false;
	}*/

	//use confidence
	float a = svm->predict(descriptor, noArray(), cv::ml::StatModel::RAW_OUTPUT);
	
	//return the confidence
	if (NULL != confidence)
	{
		*confidence = a;
	}

	if (0 > a)
	{
		return true;
	}
	else if (0 < a)
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

//pwoblem
void lShape2boundingRect(const LShape &lshape, Rect &rect)
{
	Point2d center, far;
	center.x = (lshape.vertex[0].x + lshape.vertex[2].x)/2;
	center.y = (lshape.vertex[0].y + lshape.vertex[2].y)/2;

	far.x = 2 * center.x - lshape.vertex[1].x;
	far.y = 2 * center.y - lshape.vertex[1].y;

	double x_min, y_min, x_max, y_max, length;
	x_min = std::min(lshape.vertex[0].x, lshape.vertex[1].x);
	x_min = std::min(x_min, lshape.vertex[2].x);
	x_min = std::min(x_min, far.x);
	y_min = std::min(lshape.vertex[0].y, lshape.vertex[1].y);
	y_min = std::min(y_min, lshape.vertex[2].y);
	y_min = std::min(y_min, far.y);
	x_max = std::max(lshape.vertex[0].x, lshape.vertex[1].x);
	x_max = std::max(x_max, lshape.vertex[2].x);
	x_max = std::max(x_max, far.x);
	y_max = std::max(lshape.vertex[0].y, lshape.vertex[1].y);
	y_max = std::max(y_max, lshape.vertex[2].y);
	y_max = std::max(y_max, far.y);

	length = std::max(x_max - x_min, y_max - y_min);

	rect.x = static_cast<int>(x_min);
	rect.y = static_cast<int>(y_min);
	rect.width = static_cast<int>(length);
	rect.height = static_cast<int>(length);
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

void drawBoundingBox(const list<targetConfidence> &targtRectsWithConfdc, Mat &img)
{
	for (list<targetConfidence>::const_iterator it = targtRectsWithConfdc.begin();
		it != targtRectsWithConfdc.end(); ++it)
	{
		cv::rectangle(img, it->boundingBox, green, 1);
	}
}

//0 means not overlapping. or return the area that overlaps
int Rect1OverlapsRect2Area(const Rect &rect1, const Rect &rect2)
{
	int minx = max(rect1.tl().x, rect2.tl().x);
	int miny = max(rect1.tl().y, rect2.tl().y);
	int maxx = min(rect1.br().x, rect2.br().x);
	int maxy = min(rect1.br().y, rect2.br().y);
	if (minx > maxx && miny > maxy)
	{
		return 0;
	}
	return (maxx - minx)*(maxy - miny);
}

void nms_pickDmtxRects(list<targetConfidence> &targtRectsWithConfdc)
{
	for (list<targetConfidence>::const_iterator it = targtRectsWithConfdc.begin();
		it != targtRectsWithConfdc.end();/*!!!*/)
	{
		for (list<targetConfidence>::const_iterator itB = targtRectsWithConfdc.begin();
			itB != targtRectsWithConfdc.end();/*!!!*/)
		{
			if (itB == it)
			{
				++itB;
				continue;
			}

			int overlapArea = Rect1OverlapsRect2Area(it->boundingBox, itB->boundingBox);
			if (0 < overlapArea)
			{
				//if overlaps
				if (static_cast<double>(overlapArea) / static_cast<double>(
					it->boundingBox.area() < itB->boundingBox.area() ? 
					it->boundingBox.area() : itB->boundingBox.area()) > nms_ratio)
				{
					if (it->confidence < itB->confidence)//ex::-0.9 is more confident to -0.1
					{
						targtRectsWithConfdc.erase(itB++);
					}
					else
					{
						targtRectsWithConfdc.erase(it++);
						break;//break the inner loop for next 'it'.
					}
				}// if overlaps > nms_ratio
				else// if overlaps not much
				{
					++itB;
				}
			}// if overlaps
			else//if no overlap
			{
				++itB;
			}
		}// for each in list iter::itB

		++it;
	}// for each in list iter::it
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

	showLshapes(lshapes, show, Delay_NoWait);

	//2nd stage----target RE-classification
	int count = 1, dmcount = 0;

	Mat affineImg, affineM;
	vector< float > descriptor;//prepare vector
	descriptor.reserve(hog.getDescriptorSize());//reserve enough space

	list<targetConfidence> targtRectsWithConfdc;//will shrink to contain only true targets
	targetConfidence tmpTC;
	Point2f imgLshape[3];
	for (vector< LShape >::const_iterator lshapeIt = lshapes.begin(); lshapeIt != lshapes.end(); ++lshapeIt)
	{
		cout << "\r" << count++ << "/" << lshapes.size();
		
		//if target too big, abandon
		if (lss::distance(lshapeIt->vertex[0].x, lshapeIt->vertex[0].y, 
				lshapeIt->vertex[1].x, lshapeIt->vertex[1].y ) 
				> (img.rows > img.cols ? img.rows / imgTargetSizeRatio : img.cols / imgTargetSizeRatio))
		{
			continue;
		}

		imgLshape[0] = lshapeIt->vertex[0];
		imgLshape[1] = lshapeIt->vertex[1];
		imgLshape[2] = lshapeIt->vertex[2];

		affineM = cv::getAffineTransform(imgLshape, DataMatrix);
		cv::warpAffine(img, affineImg, affineM, win_size, InterpolationFlags::INTER_LINEAR);

		//clear before usage
		descriptor.clear();
		if (classifyImg(affineImg, win_size, svm, hog, descriptor, &tmpTC.confidence))
		{
			++dmcount;
			lShape2boundingRect(*lshapeIt, tmpTC.boundingBox);
			targtRectsWithConfdc.push_back(tmpTC);
		}

#ifdef _DEBUG
		showImg("affine img", affineImg, Delay_NoWait);
#endif //_DEBUG
	}//for each lShape
	
	//do non max suppression
	nms_pickDmtxRects(targtRectsWithConfdc);

	cout << endl << "find " << targtRectsWithConfdc.size() << " DataMatrix codes" << endl;

	drawBoundingBox(targtRectsWithConfdc, show);

	showImg("result", show, Delay_WaitKey);
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

}//empty namespace


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
		//for each img
		for (vector< string >::const_iterator cit = imgsPath.begin(); cit != imgsPath.end(); ++cit)
		{
			img.release();
			img = imread(*cit, IMREAD_GRAYSCALE);

			cout << endl;
			cout << "-----------------------------------------" << endl;
			cout << "current image : " << *cit << endl;

			//small imgs
			if (win_size.width >= img.cols || win_size.height >= img.rows)
			{
				vector< float > descriptor;
				descriptor.reserve(hog.getDescriptorSize());
				descriptor.clear();
				if (true == classifyImg(img, win_size, svm, hog, descriptor))
				{
					cout << "find DataMatrix code!" << endl;
					rectangle(img, Rect(0, 0, img.size().width, img.size().height), green, 2);
					showImg("result", img, Delay_WaitKey);
				}
			}
			//two-stage classification method
			else
			{
				std::vector<LShape> lshapes;
				lshapes.reserve(100);
				lshapes.clear();
				searchTarget(img, win_size, svm, lsd, hog, lshapes);
				//collectHardEXP(img, win_size, lsd, lshapes);
			}
			cout << endl << "-----------------------------------------" << endl;
		}// for each imgsPath
	}//ptr block
}
