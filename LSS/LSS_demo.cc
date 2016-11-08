#include "LShapeDetector.hh"
#include "SquareDetector.hh"

#include <iostream>
#include <string>

#include "OpenCVcfg3.1.0.hh"

using lss::LShape;
using lss::Square;
using lss::LShapeDetector;
using lss::SquareDetector;

Ptr<LShapeDetector> lsd = lss::createLShapeDetector(10, 1000);
Ptr<SquareDetector> sd = lss::createSquareDetector(10, 1000);

void LSDlines(Mat &imgSRC, Mat &image)
{
	double start, duration_ms;
	/************************************************************************/
	/* LSD method to detect lines*/
	/************************************************************************/
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	start = double(getTickCount());
	std::vector<Vec4f> lines_std;

	// Detect the lines
	ls->detect(image, lines_std);

	duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
	std::cout << "It took " << duration_ms << " ms. for " 
		<< lines_std.size() << " targets" << std::endl;

	for (std::vector<Vec4f>::const_iterator it = lines_std.begin(); it != lines_std.end(); ++it)
	{
		line(imgSRC, cv::Point((int)(*it)[0], (int)(*it)[1]), 
			cv::Point((int)(*it)[2], (int)(*it)[3]), Scalar(255, 0, 0), 1);
	}
}

void LSSLShape(Mat &imgSRC, Mat &image)
{
	double start, duration_ms;
	/************************************************************************/
	/* LSS method to detect L shapes*/
	/************************************************************************/
	start = double(getTickCount());

	std::vector<LShape> lshapes;
	lsd->detect(image, lshapes);

	duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
	std::cout << "It took " << duration_ms << " ms. for "
		<< lshapes.size() << " targets" << std::endl;

	for (std::vector<LShape>::const_iterator it = lshapes.begin(); it != lshapes.end(); ++it)
	{
		line(imgSRC, it->vertex[0], it->vertex[1], Scalar(0, 255, 0), 3);
		line(imgSRC, it->vertex[1], it->vertex[2], Scalar(0, 255, 0), 3);
	}
}

void LSSLShapeBB(Mat &imgSRC, Mat &image)
{
	double start, duration_ms;
	/************************************************************************/
	/* LSS method to detect targets*/
	/************************************************************************/
	start = double(getTickCount());

	std::vector<Rect> targets;
	lsd->detect(image, targets);

	duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
	std::cout << "It took " << duration_ms << " ms. for "
		<< targets.size() << " targets" << std::endl;

	for (std::vector<Rect>::const_iterator it = targets.begin(); it != targets.end(); ++it)
	{
	rectangle(imgSRC, *it, Scalar(0, 255, 0), 2);
	}
}

void LSSSquare(Mat &imgSRC, Mat &image)
{
	double start, duration_ms;

	/************************************************************************/
	/* LSS method to detect square shapes*/
	/************************************************************************/
	start = double(getTickCount());

	std::vector<Square> squares;
	sd->detect(image, squares);

	duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
	std::cout << "It took " << duration_ms << " ms. for "
		<< squares.size() << " targets" << std::endl;

	for (std::vector<Square>::const_iterator it = squares.begin(); it != squares.end(); ++it)
	{
		line(imgSRC, it->vertex[0], it->vertex[1], Scalar(0, 255, 0), 3);
		line(imgSRC, it->vertex[1], it->vertex[2], Scalar(0, 255, 0), 3);
		line(imgSRC, it->vertex[2], it->vertex[3], Scalar(0, 255, 0), 3);
		line(imgSRC, it->vertex[3], it->vertex[0], Scalar(0, 255, 0), 3);
	}
}

void LSSSquareBB(Mat &imgSRC, Mat &image)
{
	double start, duration_ms;
	/************************************************************************/
	/* LSS method to detect targets*/
	/************************************************************************/
	start = double(getTickCount());

	std::vector<Rect> targets;
	sd->detect(image, targets);

	duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
	std::cout << "It took " << duration_ms << " ms. for "
		<< targets.size() << " targets" << std::endl;

	for (std::vector<Rect>::const_iterator it = targets.begin(); it != targets.end(); ++it)
	{
		rectangle(imgSRC, *it, Scalar(0, 255, 0), 2);
	}
}

int main(int argc, char** argv)
{
	std::string in;
	cv::CommandLineParser parser(argc, argv, "{@input|../../EXP/Metal1_20160923.tiff|input image}{help h||show help message}");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}
	in = parser.get<std::string>("@input");

	Mat imgSRC = imread(in, IMREAD_COLOR);
	Mat image;
	cv::cvtColor(imgSRC, image, CV_BGR2GRAY);

	//LSSLShape(imgSRC, image);

	//LSSLShapeBB(imgSRC, image);

	//LSSSquare(imgSRC, image);

	LSSSquareBB(imgSRC, image);

	LSDlines(imgSRC, image);

	cv::resize(imgSRC, imgSRC, Size(imgSRC.cols /2 , imgSRC.rows/2));

	imshow("LSS DEMO", imgSRC);
	waitKey();

	return 0;
}