#ifndef __LSS_HELPER_DEBUG_HH__
#define __LSS_HELPER_DEBUG_HH__

#define _DEBUG_GETLINES_

#include <fstream>
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <string>

#include "LineSegment.hh"

using lss::LineSegment;
using std::vector;

/*helper function, used in detect for debug
output a LineSegment data to stream
*/
std::ostream& operator<<(std::ostream& os, const LineSegment &i);

/*helper function, used in detect for debug
output a vector<LineSegment> data to stream
*/
std::ostream& operator<<(std::ostream& os, const std::vector<LineSegment> &i);

/*helper function, used in detect for debug
output a vector<LineSegment> data to stream
*/
std::ostream& operator<<(std::ostream& os, const std::vector<LineSegment *> &i);

#endif //define __LSS_HELPER_DEBUG_HH__