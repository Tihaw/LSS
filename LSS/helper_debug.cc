#include "helper_debug.hh"

/*helper function, used in detect for debug
output a LineSegment data to stream
*/
std::ostream& operator<<(std::ostream& os, const LineSegment &i) {
	return os << i.GetSX() << " " << i.GetSY() << " " <<
		i.GetEX() << " " << i.GetEY() << " " << i.GetTheta() << " " <<
		i.GetCenterX() << " " << i.GetCenterY() << " " << i.GetLength() << " " <<
		i.GetRho() << " " << i.IsPicked() << "\n";
}
/*helper function, used in detect for debug
output a vector<LineSegment> data to stream
*/
std::ostream& operator<<(std::ostream& os, const std::vector<LineSegment> &i) {
	for (vector<LineSegment>::const_iterator it = i.begin();
		it != i.end(); ++it)
	{
		os << it->GetSX() << " " << it->GetSY() << " " <<
			it->GetEX() << " " << it->GetEY() << " " << it->GetTheta() << " " <<
			it->GetCenterX() << " " << it->GetCenterY() << " " << it->GetLength() << " " <<
			it->GetRho() << " " << it->IsPicked() << "\n";
	}
	return os;
}
/*helper function, used in detect for debug
output a vector<LineSegment> data to stream
*/
std::ostream& operator<<(std::ostream& os, const std::vector<LineSegment *> &i) {
	for (std::vector<LineSegment *>::const_iterator it = i.begin();
		it != i.end(); ++it)
	{
		os << (*it)->GetSX() << " " << (*it)->GetSY() << " " <<
			(*it)->GetEX() << " " << (*it)->GetEY() << " " << (*it)->GetTheta() << " " <<
			(*it)->GetCenterX() << " " << (*it)->GetCenterY() << " " << (*it)->GetLength() << " " <<
			(*it)->GetRho() << " " << (*it)->IsPicked() << "\n";
	}
	return os;
}