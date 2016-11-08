#include "SquareDetector.hh"
//#include "helper_debug.hh"

namespace lss
{
SquareDetector::SquareDetector(int _scanWindowSize /*= 5*/,
	int _maxLineNum /*= 1000*/)
	: LineSegmentSkeletonDescriptor(_scanWindowSize, _maxLineNum)
{
}

/*        class parallelLines			*/
class  parallelLine{
public:
	parallelLine(LineSegment *l1, LineSegment *l2);

	bool goodLines4Square() const;

	inline void setPickOut() const
	{
		line1->SetPickOut();
		line2->SetPickOut();
	};

	inline bool allPicked() const{
		if (line1->IsPicked() && line2->IsPicked())
		{
			return true;
		}

		return false;
	}

	inline double GetX() const{ return centerX; }

	inline double GetY() const{ return centerY; }

	inline double GetLength() const{ return length; }

	inline double GetDistance() const{ return distance; }

	inline LineSegment * GetLine1() const{ return line1; }

	inline LineSegment * GetLine2() const{ return line2; }

private:
	LineSegment *line1;
	LineSegment *line2;

	double centerX;
	double centerY;
	double length;
	double distance;
};

parallelLine::parallelLine(LineSegment *l1, LineSegment *l2)
{
	line1 = l1;
	line2 = l2;

	centerX = (line1->GetCenterX() + line2->GetCenterX()) / 2;
	centerY = (line1->GetCenterY() + line2->GetCenterY()) / 2;

	length = (line1->GetLength() + line2->GetLength()) / 2;

	distance = line1->CenterDistance(*line2);
}

bool parallelLine::goodLines4Square() const
{
	//1. check len1 == len2?

	if (!equalLength(line1->GetLength(), line2->GetLength()))
	{
		return false;
	}

	//2. check bisector perpendicular to lines?
	LineSegment bisector(line1->GetCenterX(), line1->GetCenterY(), line2->GetCenterX(), line2->GetCenterY());

	if (!equalAngle(fabs(bisector.GetTheta() - line1->GetTheta()), RightAngle))
	{
		return false;
	}

	return true;
}


/*        class SquareDetectorImpl			
			a final/seal class
*/
class SquareDetectorImpl : public SquareDetector
{
public:
	SquareDetectorImpl(int _scanWindowSize = 5, int _maxLineNum = 1000);

	~SquareDetectorImpl();

	size_t getDescriptorSize() const;
	bool checkDetectorSize() const;
		
	/* this method find the bounding box of the location of target.
	*/
	void detect(const Mat& img, CV_OUT vector<Rect>& boundingBox);

	/* this method find the exact square shape location. which contains three vertex.
	*/
	void detect(const Mat& img, CV_OUT vector<Square>& squares);

protected:
private:
	SquareDetectorImpl& operator= (const SquareDetectorImpl&); // hide
	SquareDetectorImpl(const SquareDetectorImpl& d); // hide

	/*compute a square shape structure
	*/
	void computeSquare(const LineSegment *l1a, const LineSegment *l1b, const LineSegment *l2a, const LineSegment *l2b, CV_OUT Square &square) const;

	void findParallelLines(vector<LineSegment *> &curLineBox, vector<parallelLine> &parallelLines) const;

	bool makeSquare(const parallelLine &pl1, const parallelLine &pl2, CV_OUT Square &square) const;
};

SquareDetectorImpl::SquareDetectorImpl(int _scanWindowSize /*= 5*/,
	int _maxLineNum /*= 1000*/)
	: SquareDetector(_scanWindowSize, _maxLineNum)
{
}

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

void SquareDetectorImpl::findParallelLines(vector<LineSegment *> &curLineBox, vector<parallelLine> &parallelLines) const
{
	for (vector<LineSegment *>::const_iterator it = curLineBox.begin();
		it != curLineBox.end(); ++it)
	{
		for (vector<LineSegment *>::const_iterator itt = it + 1;
			itt != curLineBox.end(); ++itt)
		{
			if ((*itt)->IsPicked() && (*it)->IsPicked())
			{
				continue;
			}
			parallelLine pl((*it), (*itt));
			if (pl.goodLines4Square())
			{
				parallelLines.push_back(pl);
			}
		}// for itt
	}// for it
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
	vector<parallelLine> paralelLines;
	vector<parallelLine> paralelLinesL;

	while (degree < ScanboxSize)
	{
		degreeL = degree + RightAngle;
		if (degreeL > ScanboxSize - 1)//for cyclic searching along degree dimension
			degreeL -= ScanboxSize;

		paralelLines.clear();
		paralelLinesL.clear();
		paralelLinesL.reserve(10);
		paralelLinesL.reserve(10);

		findParallelLines(indexBox[degree], paralelLines);
		findParallelLines(indexBox[degreeL], paralelLinesL);

		if (!paralelLines.empty() && !paralelLinesL.empty())
		{
			for (vector<parallelLine>::const_iterator it = paralelLines.begin();
				it != paralelLines.end(); ++it)
			{
				for (vector<parallelLine>::const_iterator itL = paralelLinesL.begin();
					itL != paralelLinesL.end(); ++itL)
				{
					square.clear();
					if (makeSquare((*it),(*itL),square))
					{
						squares.push_back(square);
					}
				}
			}

			degree++;
		}// if not null
	} // while (degree < LSS_SCANBOXSIZE)
}

bool SquareDetectorImpl::makeSquare(const parallelLine &pl1, const parallelLine &pl2, CV_OUT Square &square) const
{
	//1. check if distance2 == length1 and distance1 == length2
	if (   !equalLength(pl1.GetLength(), pl2.GetDistance())
		|| !equalLength(pl1.GetDistance(), pl2.GetLength()) )
	{
		return false;
	}

	//2. check if pl1 and pl2 have the same center
	if (!identicalPoint(pl1.GetX(), pl1.GetY(), 
		pl2.GetX(), pl2.GetY(),
		pl1.GetLength()+pl2.GetLength()))
	{
		return false;
	}

	// now we find a good rectangle
	computeSquare(pl1.GetLine1(), pl1.GetLine2(), 
		pl2.GetLine1(), pl2.GetLine2(), square);

	//set picked out state
	pl1.setPickOut();
	pl1.setPickOut();

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
}//namespace lss