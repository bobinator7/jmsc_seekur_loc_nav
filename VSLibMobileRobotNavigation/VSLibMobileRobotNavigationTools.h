#ifndef VSLibMobileRobotNavigationToolsH
#define VSLibMobileRobotNavigationToolsH

#include "VSLibMobileRobotNavigationExport.h"

namespace VSLibMobileRobotNavigation
{
	class VSLibMobileRobotNavigation_DECLSPEC Tools
	{
	public:
		static unsigned int getPointQuadrant(std::pair<double, double>& point);
		static unsigned int getNextQuadrantClockwise(unsigned int& currentQuadrant);
		static unsigned int getNextQuadrantCounterclockwise(unsigned int& currentQuadrant);
		static unsigned int getQuadrantDistance(bool clockwise, unsigned int& refQuadrant, unsigned int& checkQuadrant);

		static double getEuclideanDistance(std::pair<double, double>& p1, std::pair<double, double>& p2);
		static double getArcLength(std::pair<double, double>& pStart, std::pair<double, double>& pEnd, const double& xPivot);
	};

}//end namespace

#endif 
