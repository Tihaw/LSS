#include "LineSegmentSkeletonDescriptor.hh"
#include "opencv2/imgproc/imgproc.hpp"

cv::Ptr<cv::LineSegmentDetector> ls;

lss::LineSegmentSkeletonDescriptor::LineSegmentSkeletonDescriptor(int _scanWindowSize /*= 5*/, int _maxLineNum /*= 1000*/)
	: COPIES(_scanWindowSize), MAXLINENUM(_maxLineNum)
{
	/************************************************************************/
	/* TRY ---- params*/
	/*	LSD_REFINE_NONE = 0
	LSD_REFINE_STD  = 1
	LSD_REFINE_ADV  = 2 */
	/************************************************************************/
	ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

	//reserve the vectors
	reset();
}

lss::LineSegmentSkeletonDescriptor::~LineSegmentSkeletonDescriptor()
{

}

void lss::LineSegmentSkeletonDescriptor::prepareWork(const Mat& img)
{
	Mat image;
	CV_Assert(img.channels() == 1 || img.channels() == 3 || img.channels() == 4);

	//convert img to singe channel image
	if (img.channels() == 4)
		cv::cvtColor(img, image, CV_BGRA2GRAY);
	else if (img.channels() == 3)
		cv::cvtColor(img, image, CV_BGR2GRAY);
	else
		image = img;//no copying data

	/************************************************************************/
	/* TRY ----- MORPHOLOGY PROCESS*/
	/************************************************************************/
	//...


	//LSD algorithm for detecting straight lines
	vector<cv::Vec4f> lsd_lines;
	ls->detect(image, lsd_lines);

	//evacuate vectors
	reset();

	//transfer to LineSegment
	for (vector<cv::Vec4f>::const_iterator it_const = lsd_lines.begin();
		it_const != lsd_lines.end(); ++it_const)
	{
		//construct LineSegment and right then push into lines
		lines.push_back(
			LineSegment(it_const->val[0], it_const->val[1], it_const->val[2], it_const->val[3])
			);
	}

	//store
	int IBIndex = 0;
	for (vector<LineSegment>::iterator it = lines.begin();
			it != lines.end(); ++it)
	{
		//throw the above one(just pushed back) in boxes:
		IBIndex = static_cast<int>(floor(it->GetTheta()));

		//push back in successive boxes
		//tricky implementation of scan window.... make copies into these boxes
		for (int j = 0; j < COPIES; ++j)
		{
			if ((IBIndex - j) >= 0)
			{
				indexBox[IBIndex - j].push_back(&(*it));
			}
			else
			{
				indexBox[ScanboxSize + IBIndex - j].push_back(&(*it));
			}
		}
	}
}

void lss::LineSegmentSkeletonDescriptor::reset()
{
	lines.clear();
	for (int i = 0; i < ScanboxSize; ++i)
		indexBox[i].clear();

	lines.reserve(MAXLINENUM);
	for (int i = 0; i < ScanboxSize; ++i)
		indexBox[i].reserve(MAXLINENUM / COPIES);
}
