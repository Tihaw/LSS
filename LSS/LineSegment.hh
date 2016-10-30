#ifndef __LSS_LineSegment_HH_
#define __LSS_LineSegment_HH_

namespace lss
{
const double SD_PI = 3.1415926535897932384626433832795;

const int RightAngle = 90;

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

	inline double GetTheta() const{ return theta; };
	inline double GetLength() const{ return length; };
	inline double GetRho() const{ return rho; };
	inline double GetCenterX() const{ return centerX; };
	inline double GetCenterY() const{ return centerY; };
	inline double GetSX()const{ return s_X; }
	inline double GetSY()const{ return s_Y; }
	inline double GetEX()const{ return e_X; }
	inline double GetEY()const{ return e_Y; }
	inline bool IsPicked() const{ return isPicked; };

	/*NON-CONST set the state isPicked = true
	*/
	inline void SetPickOut() { isPicked = true; };
	
	/*cal the distance of two line segment centers
	*/
	double CenterDistance(const LineSegment &line) const;

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

/*cal the intersect point, OUT: double &intersect_x, double &intersect_y
*/
void intersectPoint(const LineSegment &l1, const LineSegment &l2, double &intersect_x, double &intersect_y);

/*check if the two length are equal
*/
bool equalLength(double len1, double len2);

/*check if the two angle are equal
*/
bool equalAngle(double degree1, double degree2);

/*check if the two point are approximately same
*/
bool identicalPoint(double x1, double y1, double x2, double y2, double objSize /*length1 + length2*/);
} // namespace lss

#endif // __LSS_LineSegment_HH_