#include "../VSLibMobileRobotNavigationTools.h"
#include "../VSLibMobileRobotNavigationTools.hpp"

// 3rd party
#define _USE_MATH_DEFINES
#include <math.h>

unsigned int VSLibMobileRobotNavigation::Tools::getPointQuadrant(std::pair<double, double>& point)
{
	if (point.first >= 0)
		if (point.second >= 0)
			return 1;
		else
			return 4;
	else
		if (point.second >= 0)
			return 2;
		else
			return 3;
}

unsigned int VSLibMobileRobotNavigation::Tools::getNextQuadrantClockwise(unsigned int& currentQuadrant)
{
	unsigned int nextQuadrant = currentQuadrant - 1;
	if (nextQuadrant == 0 || nextQuadrant > 4)
		nextQuadrant = 4;

	return nextQuadrant;
}

unsigned int VSLibMobileRobotNavigation::Tools::getNextQuadrantCounterclockwise(unsigned int& currentQuadrant)
{
	unsigned int nextQuadrant = currentQuadrant + 1;
	if (nextQuadrant == 0 || nextQuadrant > 4)
		nextQuadrant = 1;

	return nextQuadrant;
}

unsigned int VSLibMobileRobotNavigation::Tools::getQuadrantDistance(bool clockwise, unsigned int& refQuadrant, unsigned int& checkQuadrant)
{
	if (refQuadrant == checkQuadrant)
		return 0;

	if (!clockwise)
	{
		unsigned int nextQuadrant = getNextQuadrantCounterclockwise(refQuadrant);
		if (checkQuadrant == nextQuadrant)
			return 1;
		nextQuadrant = getNextQuadrantCounterclockwise(nextQuadrant);
		if (checkQuadrant == nextQuadrant)
			return 2;
		nextQuadrant = getNextQuadrantCounterclockwise(nextQuadrant);
		if (checkQuadrant == nextQuadrant)
			return 3;

		return 4;
	}
	else
	{
		unsigned int nextQuadrant = getNextQuadrantClockwise(refQuadrant);
		if (checkQuadrant == nextQuadrant)
			return 1;
		nextQuadrant = getNextQuadrantClockwise(nextQuadrant);
		if (checkQuadrant == nextQuadrant)
			return 2;
		nextQuadrant = getNextQuadrantClockwise(nextQuadrant);
		if (checkQuadrant == nextQuadrant)
			return 3;

		return 4;
	}
}

double VSLibMobileRobotNavigation::Tools::getEuclideanDistance(std::pair<double, double>& p1, std::pair<double, double>& p2)
{
	return std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
}

// only for pivot points on the x axis at xPivot
double VSLibMobileRobotNavigation::Tools::getArcLength(std::pair<double, double>& pStart, std::pair<double, double>& pEnd, const double& xPivot)
{
	double angleStart = std::atan2(pStart.second, pStart.first - xPivot);
	double angleEnd = std::atan2(pEnd.second, pEnd.first - xPivot);

	double angleDiff;
	if (!std::signbit(xPivot))
		angleDiff = angleEnd - angleStart;
	else
		angleDiff = angleStart - angleEnd;


	angleDiff = wrapAngle2Pi(angleDiff);

	// radius from collision point to robot footprint
	double radius = std::sqrt(std::pow(pStart.first - xPivot, 2) + std::pow(pStart.second, 2));

	// factor to transform arc from collision point to arc of drivable path
	double factor = std::fabs(xPivot) / std::fabs(radius);

	double out = radius * std::fabs(angleDiff) * factor;
	return out;
}