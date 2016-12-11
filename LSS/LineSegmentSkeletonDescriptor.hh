#ifndef __LSS_LSSDESCRIPTOR_HH__
#define __LSS_LSSDESCRIPTOR_HH__

#include "LineSegment.hh"

#include <vector>

#include <opencv2/core/types.hpp>

using std::vector;
using cv::Mat;
using cv::Rect;

namespace lss
{
/*scanBox size, stands for 180 degree*/
const int ScanboxSize = 180;

/*reserve target num*/
const int TargetNumReserve = 100;

/*base class for any detector that uses line segment skeleton method
*/
class LineSegmentSkeletonDescriptor
{
public:
	LineSegmentSkeletonDescriptor(int _scanWindowSize = 5, int _maxLineNum = 1000);

	virtual ~LineSegmentSkeletonDescriptor();

	virtual size_t getDescriptorSize() const = 0;
	virtual bool checkDetectorSize() const = 0;

	/*found locations are the bounding rectangle, call prepareWork first in this method*/
	virtual void detect(const Mat& img, CV_OUT vector<Rect>& boundingBoxes) = 0;

protected:
	/*copy of lines*/
	vector<LineSegment > lines;

	/*easier for random access by degree*/
	vector<LineSegment *> indexBox[ScanboxSize];

	/*implemented as scanWindowSize*/
	const int COPIES;

	/*max lines that an image may contain(from LSD detection result)*/
	const int MAXLINENUM;

	/*detect lines in the image and store in LineSegment vector
	a must-be-called method in "void detect(params)"*/
	void prepareWork(const Mat& img);

private:
	/*evacuate lines and indexBox[LSS_SCANBOXSIZE]*/
	void reset();
	
	LineSegmentSkeletonDescriptor& operator=(const LineSegmentSkeletonDescriptor &rhs);//hide
};
} // namespace lss

#endif // __LSS_LSSDESCRIPTOR_HH__