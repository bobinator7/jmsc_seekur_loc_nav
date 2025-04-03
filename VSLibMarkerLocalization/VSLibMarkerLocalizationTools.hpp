#ifndef VSLibMarkerLocalizationToolsHpp
#define VSLibMarkerLocalizationToolsHpp

#define _USE_MATH_DEFINES
#include <math.h>

// inline VSM::Vector3 vsmVec3FromCVVec3d(const cv::Vec3d & vec) 
// {
   // return VSM::Vector3(vec.val[0], vec.val[1], vec.val[2]);
// }//end inline

inline double wrapAngle(double angle)
{
	double twoPi = 2.0 * M_PI; // 3.141592865358979;
	return angle - twoPi * floor((angle + M_PI) / twoPi);
}


inline double getProbFromLogOdds(double& logOdds)
{
	return (1 - (1 / (1 + std::exp(logOdds))));
}

inline double getLogOddsFromProb(double& prob)
{
	return std::log(prob / (1 - prob));
}


inline float getProbFromLogOdds(float& logOdds)
{
	return (1 - (1 / (1 + std::exp(logOdds))));
}

inline float getLogOddsFromProb(float& prob)
{
	return std::log(prob / (1 - prob));
}

#endif 