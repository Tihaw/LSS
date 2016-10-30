#include "LineSegment.hh"

#include "float.h"
#include "math.h"

#include <utility>

#define LS_NO_NEGATIVE_THETA

namespace lss{

//coordinate origin - x
const int LS_OX = 0;
//coordinate origin - y
const int LS_OY = 0;

LineSegment::LineSegment(const LineSegment& l)
	:rho(l.rho), theta(l.theta), s_X(l.s_X), s_Y(l.s_Y), e_X(l.e_X), e_Y(l.e_Y),
	length(l.length), centerX(l.centerX), centerY(l.centerY), isPicked(l.isPicked)
{	}

LineSegment& LineSegment::operator=(const LineSegment& rhs)
{
	if (this != &rhs)
	{
		LineSegment temp(rhs);
		//resource management? no
		this->swap(temp);
	}
	return *this;
}

void LineSegment::swap(LineSegment& rhs)
{
	std::swap(*this, rhs);
}

LineSegment::~LineSegment()
{	}
	
double LineSegment::CenterDistance(const LineSegment &line) const
{
	return 	sqrt(
		(this->centerX - line.centerX) * (this->centerX - line.centerX)
		+
		(this->centerY - line.centerY) * (this->centerY - line.centerY)
		);
}

LineSegment::LineSegment(double x1, double y1, double x2, double y2)
{
	double k;
	double x, y;

	s_X = x1;
	s_Y = y1;
	e_X = x2;
	e_Y = y2;

	length = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));

	/************************************************************************/
	/* rho and theta calculation.      here C point will be the origin*/
	/************************************************************************/
	/*problematic: what if D :   D------A-------B.
	line AB.   D on AB(A--------D-----------B).  C outside AB. CD °Õ AB. A x1y1 B x2y2 Cx0y0
	k=|AD| / |AB|
	k * AB = AD = AC + CD£¨”÷ AB * CD= 0; so£¨k * AB* AB = AC *AB£¨then k =AC * AB / £®AB * AB£©°£
	k = ( (x0- x1) * (x2 - x1) + (y0 - y1) * (y2 - y1) )  / ( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) ) ;
	we get : x = x1 + k*(x2 - x1); y = y1 + k*(y2 - y1);
	*/
	k = ((LS_OX - s_X) * (e_X - s_X) + (LS_OY - s_Y) * (e_Y - s_Y))
		/ ((e_X - s_X) * (e_X - s_X) + (e_Y - s_Y) * (e_Y - s_Y));
	x = s_X + k*(e_X - s_X);
	y = s_Y + k*(e_Y - s_Y);
	/*
	CA °§ CD / |CD| = rho; CA: (x1 - SD_OX, y1 - SD_OY), CD: (x- SD_OX, y - SD_OY)
	*/
	// on case that line cross origin
	if (fabs(x) < DBL_EPSILON && fabs(y) < DBL_EPSILON)
		rho = 0;
	else
		rho = ((x1 - LS_OX) * (x - LS_OX) + (y1 - LS_OY) * (y - LS_OY))
		/ sqrt((x - LS_OX) * (x - LS_OX) + (y - LS_OY) * (y - LS_OY));
	/*
	cos (theta) =  CX °§ CD / (|CX|*|CD|); CX: (1,0), CD: (x- SD_OX, y - SD_OY)
	in case D(0,0), cal: CX °§ AB / (|CX|*|AB|); AB(x2-x1, y2-y1)
	*/
	// in case that line cross origin, -- D(0,0).
	if (fabs(rho) < DBL_EPSILON)
	{
		theta = (1 * (x2 - x1) + 0 * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
		theta = acos(theta);
		theta += SD_PI / 2;
		//it does not matter, 10 degree or -170 degree
		if (theta > SD_PI)
			theta -= SD_PI;
		else if (theta < 0 - SD_PI)
			theta += SD_PI;
	}
	else
	{
		theta = (1 * (x - LS_OX) + 0 * (y - LS_OY))
			/ sqrt((x - LS_OX) * (x - LS_OX) + (y - LS_OY) * (y - LS_OY));
		//determine theta by the location of D
		if (y >= 0)
			theta = acos(theta);
		else
			theta = 0 - acos(theta);
	}

	//in degree
	theta = theta / SD_PI * 180;

#ifdef LS_NO_NEGATIVE_THETA
	if (theta < 0)
	{
		theta += 180;
		rho = 0 - rho;
	}
#endif

	//set state
	isPicked = false;

	//set center point x-y
	centerX = (e_X + s_X) / 2;
	centerY = (e_Y + s_Y) / 2;
}

void intersectPoint(const LineSegment &l1, const LineSegment &l2, double &intersect_x, double &intersect_y)
{
	if (fabs(l1.GetTheta() - l2.GetTheta()) < AngleDiffThreshold)
	{
		intersect_x = 0;
		intersect_y = 0;
		return;
	}

	/* projection of a point onto a given line	*/
	/*line AB & line CD, O is the intersect point
	we have |AO| = AB°§ AD / |AB|  (AB°§AD = |AB||AD|cos(theta))
	AO = |AO|/|AB| * AB
	so:  AO = AB°§AD / (|AB|^2) * AB.
	we can then calc with the coordinate// AO = ratio * AB;
	AO = (Bx - Ax, By- Ay)°§(Dx - Ax, Dy - Ay) / ((Bx - Ax)^2+(By - Ay)^2)
	*(Bx - Ax, By - Ay)
	O = AO + A.
	*/

	const LineSegment *lLong, *lShort;

	if (l1.GetLength() > l2.GetLength())
	{
		lLong = (&l1);
		lShort = (&l2);
	}
	else
	{
		lShort = (&l1);
		lLong = (&l2);
	}

	/* we choose the further one to lLong as D. lLong s -- A, e -- B
	*/
	double Ax, Ay, Bx, By, Dx, Dy;
	double ratio;

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

	//calc process
	ratio = ((Bx - Ax)*(Dx - Ax) + (By - Ay)*(Dy - Ay)) /
		((Bx - Ax)*(Bx - Ax) + (By - Ay)*(By - Ay));

	intersect_x = ratio * (Bx - Ax) + Ax;	intersect_y = ratio * (By - Ay) + Ay;
}

double distance(double x1, double y1, double x2, double y2)
{
	return 	sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
};

bool equalLength(double len1, double len2)
{
	const double lengthRatio = 0.2;

	if (fabs(len1 - len2) / fabs(len1 + len2) < lengthRatio)
	{
		return true;
	}

	return false;
}

bool equalAngle(double degree1, double degree2)
{
	const double angleAperture = 20;
	if (fabs(degree1 - degree2) < angleAperture)
	{
		return true;
	}

	return false;
}

bool identicalPoint(double x1, double y1, double x2, double y2, double objSize /*length1 + length2*/)
{
	const double ratio = 20;

	if (fabs(x1 - x2) > (objSize / ratio))
	{
		return false;
	}

	if (fabs(y1 - y2) > (objSize / ratio))
	{
		return false;
	}

	return true;
}

} // namespace lss
