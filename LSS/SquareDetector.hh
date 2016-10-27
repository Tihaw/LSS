#ifndef __LSS_SQUAREDETECTOR_HH_
#define __LSS_SQUAREDETECTOR_HH_

#include "LineSegmentSkeletonDescriptor.hh"

namespace lss
{
/*Square easy data structure*/
/************************************************************************/
/* vert0---------------vert3*/
/* |                   |*/
/* |                   |*/
/* |                   |*/
/* |                   |*/
/* |				   |*/
/* |                   |*/
/* |                   |*/
/* |		           |*/
/* vert1---------------vert2 */
/************************************************************************/
struct Square
{
	cv::Point2d vertex[4];
	void clear()
	{
		vertex[0].x = 0;	vertex[0].y = 0;
		vertex[1].x = 0;	vertex[1].y = 0;
		vertex[2].x = 0;	vertex[2].y = 0;
		vertex[3].x = 0;	vertex[3].y = 0;
	}
};

/* square shape detector class, provide interfaces for detecting square shape in an image
*/
class CV_EXPORTS_W SquareDetector : public LineSegmentSkeletonDescriptor
{
public:
	SquareDetector(int _scanWindowSize = 5, int _maxLineNum = 1000);

	virtual size_t getDescriptorSize() const = 0;
	virtual bool checkDetectorSize() const = 0;

	/* this method find the bounding box of the location of target.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<Rect>& boundingBox) = 0;

	/* this method find the exact square location
	*/
	virtual void detect(const Mat& img, CV_OUT vector<Square>& squares) = 0;

protected:
private:
};

/** @brief Creates a smart pointer to a SquareDetector object and initializes it.
*/
CV_EXPORTS_W cv::Ptr<SquareDetector> createSquareDetector(int _scanWindowSize = 5, int _maxLineNum = 1000);
} // namespace lss

#endif // __LSS_SQUAREDETECTOR_HH_