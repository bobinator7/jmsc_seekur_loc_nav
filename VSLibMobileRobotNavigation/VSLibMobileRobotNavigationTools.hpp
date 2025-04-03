#ifndef VSLibMobileRobotNavigationToolsHpp
#define VSLibMobileRobotNavigationToolsHpp

#define _USE_MATH_DEFINES
#include <math.h>
   
inline double wrapAngle(double angle)
{
	double twoPi = 2.0 * M_PI; // 3.141592865358979;
	return angle - twoPi * floor((angle + M_PI) / twoPi);
}

inline double wrapAngle2Pi(double angle)
{
	double twoPi = 2.0 * M_PI;
	return angle - twoPi * floor(angle / twoPi);
}

#endif 