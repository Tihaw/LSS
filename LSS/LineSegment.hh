#ifndef __LSS_LineSegment_HH_
#define __LSS_LineSegment_HH_

namespace lss
{

const double SD_PI = 3.1415926535897932384626433832795;

//if angle difference is smaller than this value, we treat as parallel
const double AngleDiffThreshold = 10.0;

//if length difference is smaller than this ratio, we treat as equal
const double LengthDiffRatioThreshold = 0.4;

//if pixel coordinate difference is smaller than this ratio, we treat as same point
const double CoordinateDiffRatioThreshold = 10;

//define a line segment with start XY and end XY
class LineSegment{
public:
	explicit LineSegment(double x1, double y1, double x2, double y2);
	LineSegment(const LineSegment& rhs);
	LineSegment& operator=(const LineSegment& rhs);
	~LineSegment();

	double GetTheta() const{ return theta; };
	double GetLength() const{ return length; };
	double GetRho() const{ return rho; };
	double GetCenterX() const{ return centerX; };
	double GetCenterY() const{ return centerY; };
	double GetSX()const{ return s_X; }
	double GetSY()const{ return s_Y; }
	double GetEX()const{ return e_X; }
	double GetEY()const{ return e_Y; }
	bool IsPicked() const{ return isPicked; };

	inline void SetPickOut() { isPicked = true; };
	double CenterDistance(LineSegment &line) const;

private:

	void swap(LineSegment& rhs);

	//in r-theta coordinate
	double rho;
	double theta;

	//in X-Y coordinate
	double s_X;
	double s_Y;
	double e_X;
	double e_Y;
	double length;
	double centerX;
	double centerY;

	bool isPicked; //whether picked
};

double distance(double x1, double y1, double x2, double y2);
void intersectPoint(const LineSegment &l1, const LineSegment &l2, double &intersect_x, double &intersect_y);

} // namespace lss

#endif // __LSS_LineSegment_HH_