#include "LShapeDetector.hh"
//#include "helper_debug.hh"

namespace lss
{
/*        LShapeDetector implementation			*/
LShapeDetector::LShapeDetector(int _scanWindowSize /*= 5*/,
	int _maxLineNum /*= 1000*/)
	: LineSegmentSkeletonDescriptor(_scanWindowSize, _maxLineNum)
{
}

/*        class LShapeDetectorImpl			*/
class LShapeDetectorImpl : public LShapeDetector
{
public:
	LShapeDetectorImpl(int _scanWindowSize = 5, int _maxLineNum = 1000);

	virtual ~LShapeDetectorImpl();

	size_t getDescriptorSize() const;
	bool checkDetectorSize() const;
		
	/* this method find the bounding box of the location of target.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<Rect>& boundingBoxes);

	/* this method find the exact L shape location. which contains three vertex.
	*/
	virtual void detect(const Mat& img, CV_OUT vector<LShape>& lShapes);

protected:
private:

	/*length ratio of the L shape line segments,
	used in goodShape() judgment*/
	const double MaxRatio = 3;

	/*center distance of the L shape line segments, to the longer line segment
	used in goodShape() judgment*/
	const double DisRatio = 1;

	LShapeDetectorImpl& operator= (const LShapeDetectorImpl&); // hide
	LShapeDetectorImpl(const LShapeDetectorImpl& d); // hide

	/*the L shape detect criteria, if good, set "picked-up " state of l1 & l2
	*/
	bool goodLShape(LineSegment * l1, LineSegment *l2, CV_OUT LShape &lShape) const;

	/*compute a L shape structure
	*/
	void computeLShape(const LineSegment * lLong, const LineSegment *lShort, CV_OUT LShape &lShape) const;
};

CV_EXPORTS_W cv::Ptr<LShapeDetector> lss::createLShapeDetector(int _scanWindowSize /*= 5*/, int _maxLineNum /*= 1000*/)
{
	return cv::makePtr<LShapeDetectorImpl>(_scanWindowSize, _maxLineNum);
}

void LShapeDetectorImpl::detect(const Mat& img, CV_OUT vector<Rect>& boundingBoxes)
{
	//call detect, find "L" shape first
	vector<LShape> lShapes;
	lShapes.reserve(TargetNumReserve);
	detect(img, lShapes);

	double x_min, y_min, x_max, y_max, length;
	//get the bounding box of each L shape
	for (std::vector<LShape>::const_iterator it = lShapes.begin();
		it != lShapes.end(); ++it)
	{
		x_min = std::min(it->vertex[0].x, it->vertex[1].x);
		x_min = std::min(x_min, it->vertex[2].x);
		y_min = std::min(it->vertex[0].y, it->vertex[1].y);
		y_min = std::min(y_min, it->vertex[2].y);

		x_max = std::max(it->vertex[0].x, it->vertex[1].x);
		x_max = std::max(x_max, it->vertex[2].x);
		y_max = std::max(it->vertex[0].y, it->vertex[1].y);
		y_max = std::max(y_max, it->vertex[2].y);

		length = std::max(x_max - x_min, y_max - y_min);

		boundingBoxes.push_back(cv::Rect_<double>(x_min, y_min, length, length));
	}
}

void LShapeDetectorImpl::detect(const Mat& img, CV_OUT vector<LShape>& lShapes)
{
	/************************************************************************/
	/* TRY ----- MORPHOLOGY PROCESS*/
	/************************************************************************/
	//...

	//a must-to-do job, prepare lines & indexBox
	prepareWork(img);

	if (lines.empty())
		return;

#ifdef _DEBUG_GETLINES_
	int t = int((double)cv::getTickCount() / cv::getTickFrequency());
	char c[8];
	_itoa(t, c, 10);
	std::string fname("lines");
	fname.append(c);
	fname.append(".txt");
	std::ofstream outFile(fname);
	for (int i = 0; i < ScanboxSize; ++i)
	{
		outFile << indexBox[i];
		outFile << "\n";
	}
	outFile.close();
#endif

	//searching:
	int degree = 0;
	int degreeL;
	LShape lShape;

	while (degree < ScanboxSize)
	{
		degreeL = degree + RightAngle;
		if (degreeL > ScanboxSize - 1)//for cyclic searching along degree dimension
			degreeL -= ScanboxSize;

		//picking up
		for (vector<LineSegment *>::const_iterator it = indexBox[degree].begin();
			it != indexBox[degree].end(); ++it)
		{
			/*if ((*it)->IsPicked())
				continue;*/

			for (vector<LineSegment *>::const_iterator itL = indexBox[degreeL].begin();
				itL != indexBox[degreeL].end(); ++itL)
			{
				if ((*itL)->IsPicked() && (*it)->IsPicked())
					continue;

				//prepare
				lShape.clear();

				if (goodLShape(*it, *itL, lShape))
				{
					lShapes.push_back(lShape);
				}						
			} // for indexBox[degreeL]
		} // for indexBox[degree]

		degree++;
	} // while (degree < LSS_SCANBOXSIZE)
}

LShapeDetectorImpl::LShapeDetectorImpl(int _scanWindowSize /*= 5*/,
	int _maxLineNum /*= 1000*/)
	: LShapeDetector(_scanWindowSize, _maxLineNum)
{
}

size_t LShapeDetectorImpl::getDescriptorSize() const
{
	//this impl has no descriptor size
	return 0;
}

bool LShapeDetectorImpl::checkDetectorSize() const
{
	//this impl has no descriptor size
	return true;
}

LShapeDetectorImpl::~LShapeDetectorImpl()
{
}

bool LShapeDetectorImpl::goodLShape(LineSegment * l1, LineSegment *l2, CV_OUT LShape &lShape) const
{
	LineSegment *lLong, *lShort;

	if (l1->GetLength() > l2->GetLength())
	{
		lLong = l1;
		lShort = l2;
	}
	else
	{
		lShort = l1;
		lLong = l2;
	}
		
	double centerDis = lLong->CenterDistance(*lShort);
	if (centerDis / lLong->GetLength() < DisRatio &&
		lLong->GetLength() / lShort->GetLength() < MaxRatio)
	{
		lLong->SetPickOut();
		lShort->SetPickOut();
			
		computeLShape(lLong, lShort, lShape);

		return true;
	}

	return false;
}

void LShapeDetectorImpl::computeLShape(const LineSegment * lLong, const LineSegment *lShort, CV_OUT LShape &lShape) const
{
	double length;
	double Ax, Ay, Bx, By, Dx, Dy;
	double Ox, Oy;

	Ax = lLong->GetSX();	Ay = lLong->GetSY();
	Bx = lLong->GetEX();	By = lLong->GetEY();

	//determine D point
	if (distance(lShort->GetSX(), lShort->GetSY(), Ax, Ay) >=
		distance(lShort->GetEX(), lShort->GetEY(), Ax, Ay))
	{
		Dx = lShort->GetSX();	Dy = lShort->GetSY();
	}
	else
	{
		Dx = lShort->GetEX();	Dy = lShort->GetEY();
	}

	/*set vertex[1]
	*/
	intersectPoint(*lLong, *lShort, Ox, Oy);
	lShape.vertex[1].x = Ox;	lShape.vertex[1].y = Oy;

	//calc length of L shape
	/*set vertex[0]
	*/
	if (distance(Ax, Ay, Ox, Oy) >=	distance(Bx, By, Ox, Oy))
	{
		length = distance(Ax, Ay, Ox, Oy);
		lShape.vertex[0].x = Ax;	lShape.vertex[0].y = Ay;
	}
	else
	{
		length = distance(Bx, By, Ox, Oy);
		lShape.vertex[0].x = Bx;	lShape.vertex[0].y = By;
	}

	/*set vertex[2]
	*/
	lShape.vertex[2].x = Ox + (Dx - Ox) / distance(Dx, Dy, Ox, Oy) * length;
	lShape.vertex[2].y = Oy + (Dy - Oy) / distance(Dx, Dy, Ox, Oy) * length;
}
}//namespace lss