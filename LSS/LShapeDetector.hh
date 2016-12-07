#ifndef __LSS_LSHAPEDETECTOR_HH_
#define __LSS_LSHAPEDETECTOR_HH_

#include "LineSegmentSkeletonDescriptor.hh"

using cv::Point2d;

namespace lss
{
/* "L" shape structure, vertex[0]\	vertex[1]\	vertex[2]\
						A point		corner		B point
						*/
struct CV_EXPORTS_W LShape
{
	Point2d vertex[3];
	void clear()
	{
		vertex[0].x = 0;	vertex[0].y = 0;
		vertex[1].x = 0;	vertex[1].y = 0;
		vertex[2].x = 0;	vertex[2].y = 0;
	}
};

/* "L" shape detector class, provide interfaces for detecting L shape in an image
*/
class CV_EXPORTS_W LShapeDetector : public LineSegmentSkeletonDescriptor
{
public:
	LShapeDetector(int _scanWindowSize = 5, int _maxLineNum = 1000);

	virtual size_t getDescriptorSize() const = 0;
	virtual bool checkDetectorSize() const = 0;

	/* this method find the bounding box of the location of target.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<Rect>& boundingBoxes) = 0;

	/* this method find the exact L shape location. which contains three vertex.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<LShape>& lShapes) = 0;

protected:
private:
	LShapeDetector& operator=(const LShapeDetector& rhs);//hide
};

/** @brief Creates a smart pointer to a LShapeDetector object and initializes it.
*/
CV_EXPORTS_W cv::Ptr<LShapeDetector> createLShapeDetector(int _scanWindowSize = 5, int _maxLineNum = 1000);
} // namespace lss

#endif // __LSS_LSHAPEDETECTOR_HH_