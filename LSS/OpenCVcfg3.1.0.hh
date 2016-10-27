#pragma once

#ifndef	__OPENCV_CFG_HH__
#define __OPENCV_CFG_HH__

#include <opencv.hpp>

#ifdef _DEBUG
#define lnkLIB(name) name "d"
#else
#define lnkLIB(name) name
#endif
#define CV_VERSION_ID CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#define cvLIB(name) lnkLIB("opencv_" name CV_VERSION_ID)

/************************************************************************/
/*                    opencv libs for 3.1.0*/
/************************************************************************/

#pragma comment( lib, cvLIB("world"))

using namespace cv;

#endif // __OPENCV_CFG_HH__