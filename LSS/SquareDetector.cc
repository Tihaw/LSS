#include "SquareDetector.hh"
//#include "helper_debug.hh"

namespace lss
{
SquareDetector::SquareDetector(int _scanWindowSize /*= 5*/,
	int _maxLineNum /*= 1000*/)
	: LineSegmentSkeletonDescriptor(_scanWindowSize, _maxLineNum)
{
}

/*        class SquareDetectorImpl			*/
class SquareDetectorImpl : public SquareDetector
{
public:
	SquareDetectorImpl(int _scanWindowSize = 5, int _maxLineNum = 1000);

	virtual ~SquareDetectorImpl();

	size_t getDescriptorSize() const;
	bool checkDetectorSize() const;
		
	/* this method find the bounding box of the location of target.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<Rect>& boundingBox);

	/* this method find the exact square shape location. which contains three vertex.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<Square>& squares);

protected:
private:
	SquareDetectorImpl& operator= (const SquareDetectorImpl&); // hide
	SquareDetectorImpl(const SquareDetectorImpl& d); // hide

	/*the square shape detect criteria, if good, set "picked-up " state of lines
	*/
	bool goodSquare(LineSegment * l1a, LineSegment *l1b, LineSegment *l2a, LineSegment *l2b, CV_OUT Square &square) const;

	/*compute a square shape structure
	*/
	void computeSquare(const LineSegment *l1a, const LineSegment *l1b, const LineSegment *l2a, const LineSegment *l2b, CV_OUT Square &square) const;

	/*find if there's a line segment that is parallel, and possibly make a square
	*/
	bool findQualifiedParallelLineSegment(vector<LineSegment *>::const_iterator &curLine, vector<LineSegment *> &lines, vector<LineSegment *>::const_iterator &curLinePrlPair) const;
};

CV_EXPORTS_W cv::Ptr<SquareDetector> lss::createSquareDetector(int _scanWindowSize /*= 5*/, int _maxLineNum /*= 1000*/)
{
	return cv::makePtr<SquareDetectorImpl>(_scanWindowSize, _maxLineNum);
}

/* this method find the bounding box of the location of target.
*/
void SquareDetectorImpl::detect(const Mat& img, CV_OUT vector<Rect>& boundingBox)
{
	//call detect, find squares first
	vector<Square> squares;
	squares.reserve(TargetNumReserve);
	detect(img, squares);
	
	double x_min, y_min, x_max, y_max, length;
	//get the bounding box of each L shape
	for (std::vector<Square>::const_iterator it = squares.begin();
		it != squares.end(); ++it)
	{
		x_min = std::min(it->vertex[0].x, it->vertex[1].x);
		x_min = std::min(x_min, it->vertex[2].x);
		x_min = std::min(x_min, it->vertex[3].x);
		y_min = std::min(it->vertex[0].y, it->vertex[1].y);
		y_min = std::min(y_min, it->vertex[2].y);
		y_min = std::min(y_min, it->vertex[3].y);

		x_max = std::max(it->vertex[0].x, it->vertex[1].x);
		x_max = std::max(x_max, it->vertex[2].x);
		x_max = std::max(x_max, it->vertex[3].x);
		y_max = std::max(it->vertex[0].y, it->vertex[1].y);
		y_max = std::max(y_max, it->vertex[2].y);
		x_max = std::max(x_max, it->vertex[3].x);

		length = std::max(x_max - x_min, y_max - y_min);

		boundingBox.push_back(cv::Rect_<double>(x_min, y_min, length, length));
	}
}

/* this method find the exact square shape location. which contains three vertex.
*/
void SquareDetectorImpl::detect(const Mat& img, CV_OUT vector<Square>& squares)
{
	//a must-to-do job, prepare lines & indexBox
	prepareWork(img);

	if (lines.empty())
		return;

	//searching:
	int degree = 0;
	int degreeL;
	Square square;

	while (degree < ScanboxSize)
	{
		degreeL = degree + RightAngle;
		if (degreeL > ScanboxSize - 1)//for cyclic searching along degree dimension
			degreeL -= ScanboxSize;

		//picking up
		for (vector<LineSegment *>::const_iterator it = indexBox[degree].begin();
			it != indexBox[degree].end(); ++it)
		{
			vector<LineSegment *>::const_iterator itParaPair;
			//check if 'it' has a equal length parallel line pair
			if (!findQualifiedParallelLineSegment(it, indexBox[degree], itParaPair))
			{
				continue;
			}

			for (vector<LineSegment *>::const_iterator itL = indexBox[degreeL].begin();
				itL != indexBox[degreeL].end(); ++itL)
			{
				vector<LineSegment *>::const_iterator itLParaPair;
				//check if 'itL' has a equal length parallel line pair
				if (!findQualifiedParallelLineSegment(itL, indexBox[degreeL], itLParaPair))
				{
					continue;
				}

				if ((*itL)->IsPicked() && (*it)->IsPicked())
				{
					continue;
				}				

				//prepare
				square.clear();

				if (goodSquare(*it, *itParaPair, *itL, *itLParaPair, square))
				{
					squares.push_back(square);
				}						
			} // for indexBox[degreeL]
		} // for indexBox[degree]

		degree++;
	} // while (degree < LSS_SCANBOXSIZE)
}

SquareDetectorImpl::SquareDetectorImpl(int _scanWindowSize /*= 5*/,
	int _maxLineNum /*= 1000*/)
	: SquareDetector(_scanWindowSize, _maxLineNum)
{
}

size_t SquareDetectorImpl::getDescriptorSize() const
{
	//this impl has no descriptor size
	return 0;
}

bool SquareDetectorImpl::checkDetectorSize() const
{
	//this impl has no descriptor size
	return true;
}

SquareDetectorImpl::~SquareDetectorImpl()
{
}

/*the square shape detect criteria, if good, set "picked-up " state of lines
*/
bool SquareDetectorImpl::goodSquare(LineSegment * l1a, LineSegment *l1b, LineSegment *l2a, LineSegment *l2b, CV_OUT Square &square) const
{
	//check center distance
/*//using rho is not a precise way
	if ( fabs( fabs( l1a->GetRho() - l1b->GetRho() ) - l2a->GetLength() )
		> LengthDiffRatioThreshold )
	{
		return false;
	}
	if (fabs(fabs(l2a->GetRho() - l2b->GetRho()) - l1a->GetLength())
		> LengthDiffRatioThreshold)
	{
		return false;
	}*/

	if (fabs(l1a->CenterDistance(*l1b) - l2a->GetLength()) / l2a->GetLength() < LengthDiffRatioThreshold)
	{
		return false;
	}
	if (fabs(l2a->CenterDistance(*l2b) - l1a->GetLength()) / l1a->GetLength() < LengthDiffRatioThreshold)
	{
		return false;
	}

	//check position
	if (fabs((l1a->GetCenterX() + l1b->GetCenterX()) / 2 
		- (l2a->GetCenterX() + l2b->GetCenterX()) / 2) > CoordinateDiffRatioThreshold)
	{
		return false;
	}
	if (fabs((l1a->GetCenterY() + l1b->GetCenterY()) / 2
		- (l2a->GetCenterY() + l2b->GetCenterY()) / 2) > CoordinateDiffRatioThreshold)
	{
		return false;
	}

	l1a->SetPickOut();
	l1b->SetPickOut();
	l2a->SetPickOut();
	l2b->SetPickOut();
			
	computeSquare(l1a, l1b, l2a, l2b, square);
	return true;
}

/*compute a square shape structure
*/
void SquareDetectorImpl::computeSquare(const LineSegment *l1a, const LineSegment *l1b, const LineSegment *l2a, const LineSegment *l2b, CV_OUT Square &square) const
{
	/************************************************************************/
	/* 0-------------------3*/
	/* |       l1a         |*/
	/* |                   |*/
	/* |                   |*/
	/* |                   |*/
	/* |l2a             l2b|*/
	/* |                   |*/
	/* |                   |*/
	/* |       l1b         |*/
	/* 1-------------------2*/
	/************************************************************************/
	/*set vertex[0]
	*/
	intersectPoint(*l1a, *l2a, square.vertex[0].x, square.vertex[0].y);
	intersectPoint(*l2a, *l1b, square.vertex[1].x, square.vertex[1].y);
	intersectPoint(*l1b, *l2b, square.vertex[2].x, square.vertex[2].y);
	intersectPoint(*l2b, *l1a, square.vertex[3].x, square.vertex[3].y);
}

/*find if there's a line segment that is parallel, and possibly make a square
*/
bool SquareDetectorImpl::findQualifiedParallelLineSegment(vector<LineSegment *>::const_iterator &curLine, vector<LineSegment *> &lines, vector<LineSegment *>::const_iterator &curLinePrlPair) const
{
	for (vector<LineSegment *>::const_iterator it = lines.begin();
		it != lines.end(); ++it)
	{
		if (curLine == it)
		{
			continue;
		}

		if ((*curLine)->IsPicked() && (*it)->IsPicked())
		{
			continue;
		}

		//equal length
		if (fabs((*curLine)->GetLength()-(*it)->GetLength()) / 
			(*curLine)->GetLength() > LengthDiffRatioThreshold)
		{
			continue;
		}

		//bisector is perpendicular to curLine
		LineSegment bisector(
			(*curLine)->GetCenterX(), (*curLine)->GetCenterY(), 
			(*it)->GetCenterX(), (*it)->GetCenterY());

		if (fabs( fabs(	bisector.GetTheta() - (*curLine)->GetTheta()) - 
				RightAngle) < AngleDiffThreshold)
		{
			curLinePrlPair = it;
			return true;
		}
	}

	return false;
}
}//namespace lss