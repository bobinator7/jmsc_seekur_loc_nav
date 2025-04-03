#include "../VSLibMobileRobotNavigationCollavDWA.h"

// Tools
#include "../VSLibMobileRobotNavigationTools.h"
#include "../VSLibMobileRobotNavigationTools.hpp"

// 3rd party

namespace VSLibMobileRobotNavigation
{
	CollavDWA::CollavDWA()
		: numRadialPaths_(40)
		, numSamplesPerRadius_(5)
		, sigma_(1.0)
		, alpha_(0.2)
		, beta_(0.2)
		, gamma_(0.2)
		, useMapPoints_(false)
		, ptrMap_(nullptr)
	{
		//calcWeighting = &CollavDWA::calcWeightingBasic;
		calcWeighting = &CollavDWA::calcWeightingObstacleCostmap;
	}

	CollavDWA::~CollavDWA()
	{
	}

	void CollavDWA::setUseMapPointsFlag(bool flag)
	{
		useMapPoints_ = flag;
	}

	VSM::Vector3 CollavDWA::getRobotPose()
	{
		return poseRobot_;
	}

	std::vector<std::pair<double, double>> CollavDWA::getCollisionPointCloud()
	{
		return collisionPointCloud_;
	}

	std::vector<std::pair<double, double>> CollavDWA::getCollisionPoints()
	{
		return collPoints_;
	}

   std::vector<std::pair<double, double>> CollavDWA::getIntersectionPoints()
   {
      return intersectPoints_;
   }

   std::vector<double> CollavDWA::getVecRadius()
   {
      return vecRadius_;
   }

	void CollavDWA::setObjectiveFunctionParam(const double& sigma, const double& alpha, const double& beta, const double& gamma)
	{
		sigma_ = sigma;
		alpha_ = alpha;
		beta_ = beta;
		gamma_ = gamma;
	}

	void CollavDWA::setRobotProperties(const double& robotWidth, const double& robotLength)
	{
		robotWidth_ = robotWidth;
		robotLength_ = robotLength;
	}

	void CollavDWA::setRobotVelocityBounds(const double& transLower, const double& transUpper, const double& rotLower, const double& rotUpper)
	{
		transVelLowBound_ = transLower;
		transVelUpBound_ = transUpper;
		rotVelLowBound_ = rotLower;
		rotVelUpBound_ = rotUpper;
	}

	void CollavDWA::setRobotAccelerationBounds(const double& transLower, const double& transUpper, const double& rotLower, const double& rotUpper)
	{
		transAccLowBound_ = transLower;
		transAccUpBound_ = transUpper;
		rotAccLowBound_ = rotLower;
		rotAccUpBound_ = rotUpper;
	}

	std::vector<double> CollavDWA::sampleRadialPaths()
	{
		std::vector<double> retVec;
		double minAngle = 0;
		double maxAngle = M_PI;

		// quadrant 1
		if (vrMax_ > 0 && vrMin_ > 0)
		{
			maxAngle = atan2(vtMax_, vrMin_);
			minAngle = atan2(vtMin_, vrMax_);
		}
		// quadrant 2
		else if (vrMax_ < 0 && vrMin_ < 0)
		{
			maxAngle = atan2(vtMin_, vrMax_);
			minAngle = atan2(vtMax_, vrMin_);
		}
		// quandrant 1 + 2
		else
		{
			maxAngle = atan2(vtMin_, vrMin_);
			minAngle = atan2(vtMin_, vrMax_);
		}

		///debug
		if (vtMin_ <= DBL_EPSILON)
		{
			retVec.push_back(0.0);
			retVec.push_back(-0.0);
		}


		// radius is gradient of v / w
		double stepAngle = (maxAngle - minAngle) / (numRadialPaths_ + 1);
		for (int ii = 0; ii < numRadialPaths_; ii++)
		{
			double radius = 0;
			double currentAngle = minAngle + stepAngle * (ii + 1);
			if (currentAngle < M_PI_2)
			{
				radius = -tan(currentAngle);
				if (std::isnan(radius))
					radius = std::numeric_limits<double>::infinity();
			}
			else
			{
				currentAngle = M_PI_2 - (currentAngle - M_PI_2);
				radius = tan(currentAngle);
				if (std::isnan(radius))
					radius = std::numeric_limits<double>::infinity();
			}

			retVec.push_back(radius);
		}

		// sort vector by "straigtness of path"
		std::sort(retVec.begin(), retVec.end(), [](double a, double b) {
			return std::fabs(a) > std::fabs(b);
		});

		return retVec;

	}

	bool CollavDWA::calculateCollisionWithPointCloud(
		double radius, 
		std::vector<std::pair<double, double>>& laserscanPointCloud,
		double& collDist,
		std::pair<double, double>& admSpeed)
	{
		bool retVal = false;
		double minCollDist = std::fabs(radius * 2 * M_PI);
		std::pair<double, double> maxAdmSpeed = std::make_pair(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()); 
		std::pair<double, double> collPoint;
      std::pair<double, double> intersectPoint;

		// straight motion
		if (std::isinf(radius))
		{
			for (auto& itPoint : laserscanPointCloud)
			{
				if (std::fabs(itPoint.first) < robotWidth_)
				{
					if (retVal == false)
						retVal = true;

					// calc collision distance
					double newCollDist = itPoint.second - robotWidth_ - (velTrans_ * deltaT_);
					if (newCollDist < minCollDist)
					{
						minCollDist = newCollDist;
						double v = std::sqrt(2 * minCollDist * std::fabs(transAccLowBound_));
						double w = 0.0;
						maxAdmSpeed = std::make_pair(v, w);

						/// debug
						collPoint = itPoint;
					}

				}
			}
		}
		// arc
		else
		{
			double maxDistToCollision, minDistToCollision;

			// pivot point in robot footprint
			if (std::fabs(radius) < robotWidth_)
			{
				maxDistToCollision = std::sqrt(std::pow(fabs(radius) + robotWidth_, 2) + std::pow(robotLength_, 2));
				minDistToCollision = 0;

			}
			// pivot point outside robot footprint
			else
			{
				maxDistToCollision = std::sqrt(std::pow(fabs(radius) + robotWidth_, 2) + std::pow(robotLength_, 2));
				minDistToCollision = std::fabs(std::fabs(radius) - robotWidth_);
			}

			for (auto& itPoint : laserscanPointCloud)
			{
				double distToPivotPoint = std::sqrt(std::pow(itPoint.first - radius, 2) + std::pow(itPoint.second, 2));
				if (distToPivotPoint < maxDistToCollision && distToPivotPoint >= minDistToCollision)
				{
					if (retVal == false)
						retVal = true;

					// calc collision distance
					double newCollDist = calcRadialCollisionDistance(radius, distToPivotPoint, itPoint);
					if (newCollDist < minCollDist)
					{
						minCollDist = newCollDist;

						double v, w;
						v = std::sqrt(2 * minCollDist * std::fabs(transAccLowBound_));
						w = v / -radius;

						
						maxAdmSpeed = std::make_pair(v, w);

						/// debug
						collPoint = itPoint;
                  intersectPoint = tmpIntersectPoint_;
					}
				}
			}
		}

		/// debug
		collPoints_.push_back(collPoint);
      intersectPoints_.push_back(intersectPoint);

		collDist = minCollDist;
		admSpeed = maxAdmSpeed;

		return retVal;
	}

	double CollavDWA::calcRadialCollisionDistance(double radiusPivotpoint, double distPointToPivotpoint, std::pair<double, double>& scanPoint)
	{
		// how?
		// calculate every solution of intersection between robot footprint and circular path
		// take intersection point in the quadrant nearest to scanpoint and with closest euclidean distance
		// (counterclockwise for positive r and clockwise for negative r)
		// atm: code is hardcoded for rectangular footprints 
		// -> use geometry library like boost geometry or cgal for intersection point generation and/or
		// collision distance generation

		// get current quadrant of laserpoint
		unsigned int scanQuadrant = Tools::getPointQuadrant(scanPoint);

		std::vector<std::pair<unsigned int, std::pair<double, double>>> intersectionPoints;
		// get intersections with left and right side of the rectangle
		double tmp1 = std::pow(robotWidth_ - radiusPivotpoint, 2);
		double tmp2 = std::pow(-robotWidth_ - radiusPivotpoint, 2);
		double tmp3 = std::pow(robotLength_, 2);
		double tmp4 = std::pow(-robotLength_, 2);

		// no intersection with x-axis
		double tmp = std::pow(distPointToPivotpoint, 2);
		if (tmp < tmp1 && tmp < tmp2)
			return std::numeric_limits<double>::max();

		double f1, f2;
		// calculate intersection with right side of robot
		if (tmp >= tmp1)
		{
			f1 = robotWidth_;
			f2 = std::sqrt(tmp - tmp1);
			if (f2 < robotLength_)
			{
				std::pair<double, double> point = std::make_pair(f1, f2);
				unsigned int quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
				point = std::make_pair(f1, -f2);
				quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
			}
		}
		
		// calculate intersection with left side of robot
		if (tmp >= tmp2)
		{
			f1 = -robotWidth_;
			f2 = std::sqrt(tmp - tmp2);
			if (f2 < robotLength_)
			{
				std::pair<double, double> point = std::make_pair(f1, f2);
				unsigned int quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
				point = std::make_pair(f1, -f2);
				quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
			}
		}

		// calculate intersection with top side of robot
		if (tmp >= tmp3)
		{
			f2 = robotLength_;
			f1 = radiusPivotpoint + std::sqrt(tmp - tmp3);
			if (std::fabs(f1) < robotWidth_)
			{
				std::pair<double, double> point = std::make_pair(f1, f2);
				unsigned int quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
			}
			f1 = radiusPivotpoint - std::sqrt(tmp - tmp3);
			if (std::fabs(f1) < robotWidth_)
			{
				std::pair<double, double> point = std::make_pair(f1, f2);
				unsigned int quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
			}
		}

		// calculate intersection with bottom side of robot
		if (tmp >= tmp4)
		{
			f2 = -robotLength_;
			f1 = radiusPivotpoint + std::sqrt(tmp - tmp4);
			if (std::fabs(f1) < robotWidth_)
			{
				std::pair<double, double> point = std::make_pair(f1, f2);
				unsigned int quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
			}
			f1 = radiusPivotpoint - std::sqrt(tmp - tmp4);
			if (std::fabs(f1) < robotWidth_)
			{
				std::pair<double, double> point = std::make_pair(f1, f2);
				unsigned int quadrant = Tools::getPointQuadrant(point);
				intersectionPoints.push_back(std::make_pair(quadrant, point));
			}
		}

		// this should not happen!
		if (intersectionPoints.empty())
			return std::numeric_limits<double>::max();

		// get closest intersection point
		std::pair<double, double> closestIntersectionPoint;
		unsigned int minDistQuad = 4;
		double minDist = std::numeric_limits<double>::max();
		for (auto& pointIt : intersectionPoints)
		{

			double dist = Tools::getArcLength(scanPoint, pointIt.second, radiusPivotpoint);

			//minDist = std::min(minDist, dist);
         if (dist < minDist)
         {
            minDist = dist;
            closestIntersectionPoint = pointIt.second;
         }
		}

		// calculate distance from scan point to intersection point
		// TODO: check if virtual position of robot in next timestep is needed instead of current
		//double out = Tools::getArcLength(scanPoint, closestIntersectionPoint, radiusPivotpoint);
      tmpIntersectPoint_ = closestIntersectionPoint;
		return minDist;
	}

	double CollavDWA::getMaxClearanceToPointCloud(std::vector<std::pair<double, double>>& pointCloudInRobotFrame)
	{
		double minDist = std::numeric_limits<double>::max();
		for (auto& it : pointCloudInRobotFrame)
		{
			if (std::fabs(it.first) < robotWidth_)
			{
				if (std::fabs(it.second) < (robotLength_ - robotWidth_))
				{
					return 0.0;
				}
				else
				{
					double xDiff = std::fabs(it.first);
					double yDiff = std::fabs(it.second) - std::fabs(robotLength_);
					double tmpDist = std::hypot(xDiff, yDiff);
					if (tmpDist < robotWidth_)
						return 0.0;
					else
						minDist = std::min(minDist, (tmpDist - robotWidth_));
				}
			}
			else
			{
				if (std::fabs(it.second) < (robotLength_- robotWidth_))
				{
					double dist = std::fabs(it.first) - robotWidth_;
					minDist = std::min(minDist, dist);
				}
				else
				{
					double xDiff = std::fabs(it.first);
					double yDiff = std::fabs(it.second) - std::fabs(robotLength_);
					double tmpDist = std::hypot(xDiff, yDiff);
					if (tmpDist < robotWidth_)
						return 0.0;
					else
						minDist = std::min(minDist, (tmpDist - robotWidth_));
				}
			}
		}

		return minDist;
	}

	std::pair<double, double> CollavDWA::getMaxVelocityInDW(double radiusPivotpoint)
	{
		if (std::fabs(radiusPivotpoint) < DBL_EPSILON)
			if (std::signbit(radiusPivotpoint))
				return std::make_pair(0.0, vrMax_);
			else
				return std::make_pair(0.0, vrMin_);

		// upper edge of dw in v/w diagram
		double vrTop = vtMax_ / (-radiusPivotpoint);
		if (vrTop < vrMax_ && vrTop > vrMin_)
			return std::make_pair(vtMax_, vrTop);

		// right edge of dw in v/w diagram
		double vtTop = (-radiusPivotpoint) * vrMax_;
		if (vtTop < vtMax_ && vtTop > vtMin_)
			return std::make_pair(vtTop, vrMax_);

		// left edge of dw in v/w diagram
		vtTop = (-radiusPivotpoint) * vrMin_;
		if (vtTop < vtMax_ && vtTop > vtMin_)
			return std::make_pair(vtTop, vrMin_);

		// no solution for given radius!
		return std::make_pair(0.0, 0.0);
	}

	std::pair<double, double> CollavDWA::getMinVelocityInDW(double radiusPivotpoint)
	{
		// lower edge of dw in v/w diagram
		double vrTop = vtMin_ / (-radiusPivotpoint);
		if (vrTop < vrMax_ && vrTop > vrMin_)
			return std::make_pair(vtMin_, vrTop);

		// no solution for given radius!
		return std::make_pair(0.0, 0.0);
	}

	bool CollavDWA::isInDW(std::pair<double, double> vPt)
	{
		if (vPt.first <= vtMax_ && vPt.first >= vtMin_ && vPt.second <= vrMax_ && vPt.second >= vrMin_)
			return true;
		else
			return false;
	}

	double CollavDWA::calcWeightingBasic(double collDist, std::pair<double, double> vSample)
	{
		VSM::Vector3 poseRobotProjected = calcFuturePose(vSample);

		if (!ptrMap_)
			return 0.;

		double yDiff = poseGoal_.getY() - poseRobotProjected.getY();
		double xDiff = poseGoal_.getX() - poseRobotProjected.getX();
		double angleDiff2 = std::atan2(yDiff, xDiff);
		double angleDiff = angleDiff2 - (poseRobotProjected.getZ() + M_PI_2);		// robot is headed in its y axis
		angleDiff = wrapAngle(angleDiff);
		double weightHeading = (M_PI - std::fabs(angleDiff)) / M_PI;

		double weightDist;
		double maxTravelRange = 2;
		if (collDist > maxTravelRange)
			weightDist = 1;
		else
			weightDist = collDist / maxTravelRange;

		// TODO: remove hardcoded goal tolerance (replace with goal distance cost function?)
		double weightVelocity;
		double goalTolerance = 0.1;
		if (std::hypot(xDiff, yDiff) < goalTolerance)
			weightVelocity = 1 - vSample.first / transVelUpBound_;
		else
			weightVelocity = vSample.first / transVelUpBound_;

		return sigma_ * (alpha_ * weightHeading + gamma_ * weightVelocity + beta_ * weightDist);


	}

	double CollavDWA::calcWeightingObstacleCostmap(std::pair<double, double> vSample)
	{
		VSM::Vector3 poseRobotProjected = calcFuturePose(vSample);

		if (!ptrMap_)
			return 0.;

		double weightHeading = ptrMap_->getGradNF1(poseRobotProjected);

		double weightDist = ptrMap_->getCostmapWeighting(poseRobotProjected);

		// TODO: remove hardcoded goal tolerance (replace with goal distance cost function?)
		double weightVelocity;
		double goalTolerance = 0.1;
		double yDiff = poseGoal_.getY() - poseRobotProjected.getY();
		double xDiff = poseGoal_.getX() - poseRobotProjected.getX();
		if (std::hypot(xDiff, yDiff) < goalTolerance)
			weightVelocity = 1 - vSample.first / transVelUpBound_;
		else
			weightVelocity = vSample.first / transVelUpBound_;

		return sigma_ * (alpha_ * weightHeading + gamma_ * weightVelocity + beta_ * weightDist);
	}

	std::vector<double> CollavDWA::calcWeightingNormed(std::vector<std::pair<double, double>>& vSamples, std::vector<double>& vecCollDists, std::vector<std::pair<double, double>>& laserscanPointCloud)
	{
		if (!ptrMap_)
			return std::vector<double>();

		std::vector<double> headingWeights, distWeights, clearWeights, velWeights, outWeights;

		for (size_t ii = 0; ii < vSamples.size(); ii++)
		{
			VSM::Vector3 poseRobotProjected = calcFuturePose(vSamples.at(ii));

			double yDiff = poseGoal_.getY() - poseRobotProjected.getY();
			double xDiff = poseGoal_.getX() - poseRobotProjected.getX();
			
			double goalTolerance1 = 0.2;

			double targetAngle;
			if (std::hypot(xDiff, yDiff) < goalTolerance1)
				targetAngle = poseGoal_.getZ();
			else
				targetAngle = std::atan2(yDiff, xDiff) - M_PI_2;	// robot is headed in its y axis

			double angleDiff = targetAngle - poseRobotProjected.getZ();	
			angleDiff = wrapAngle(angleDiff);
			double weightHeading = (M_PI - std::fabs(angleDiff)) / M_PI;
			headingWeights.push_back(weightHeading);

			double weightDist = 0.0;
			double maxTravelRange = 2;
			if (vecCollDists.at(ii) > maxTravelRange)
				weightDist = 1;
			else
				weightDist = vecCollDists.at(ii) / maxTravelRange;
			distWeights.push_back(weightDist);

			double weightSideClearance = 1.0;
			double safetyMaxArea = 0.5;
			double safetyMinArea = 0.0;
			std::vector<std::pair<double, double>> tmp = calcFuturePointCloud(vSamples.at(ii), laserscanPointCloud);
			double robotClearance = getMaxClearanceToPointCloud(tmp);
			if (std::hypot(xDiff, yDiff) >= goalTolerance1)
			{
				if (robotClearance < safetyMaxArea)
				{
					if (robotClearance < safetyMinArea)
					{
						weightSideClearance = 0.0;
					}
					else
					{
						double tmp1 = (robotClearance - safetyMinArea) / (safetyMaxArea - safetyMinArea);
						weightSideClearance = tmp1;//(1.0 - std::cos(tmp1 * M_PI)) * 0.5;
					}
				}
			}
			clearWeights.push_back(weightSideClearance);


			//double weightVelocity = 1 / std::hypot(xDiff, yDiff);
			double weightVelocity;
			double goalTolerance2 = 0.2;
			if (std::hypot(xDiff, yDiff) < goalTolerance2)
			{
				weightVelocity = 1 - vSamples.at(ii).first / transVelUpBound_;
			}
			else
			{
				weightVelocity = vSamples.at(ii).first / transVelUpBound_;
			}

			velWeights.push_back(weightVelocity);
		}

		// normalize within one sampling step	
		double headMin = 0;// *std::min_element(std::begin(headingWeights), std::end(headingWeights));
		double headMax = 1;// *std::max_element(std::begin(headingWeights), std::end(headingWeights));
		double distMin = 0;// *std::min_element(std::begin(distWeights), std::end(distWeights));
		double distMax = 1;// *std::max_element(std::begin(distWeights), std::end(distWeights));
		double clearMin = 0;// *std::min_element(std::begin(clearWeights), std::end(clearWeights));
		double clearMax = 1;// *std::max_element(std::begin(clearWeights), std::end(clearWeights));
		double velMin = 0;// *std::min_element(std::begin(velWeights), std::end(velWeights));
		double velMax = 1;// *std::max_element(std::begin(velWeights), std::end(velWeights));
		for (size_t ii = 0; ii < distWeights.size(); ii++)
		{
			headingWeights.at(ii) = (headingWeights.at(ii) - headMin) / (headMax - headMin);
			distWeights.at(ii) = (distWeights.at(ii) - distMin) / (distMax - distMin);
			clearWeights.at(ii) = (clearWeights.at(ii) - clearMin) / (clearMax - clearMin);
			velWeights.at(ii) = (velWeights.at(ii) - velMin) / (velMax - velMin);

			double outWeight = sigma_ * (alpha_ * headingWeights.at(ii) + gamma_ * velWeights.at(ii) + beta_ * clearWeights.at(ii)); // + beta_ * distWeights.at(ii)
			outWeights.push_back(outWeight);
		}

		return outWeights;
	}

	VSM::Vector3 CollavDWA::calcFuturePose(std::pair<double, double> vSample)
	{
		double transDiffOfProjection = vSample.first * deltaT_;
		double rotDiffOfProjection = vSample.second * deltaT_;

		double resAngle = poseRobot_.z + rotDiffOfProjection;
		resAngle = wrapAngle(resAngle);

		double x = transDiffOfProjection * (-std::sin(resAngle)) + poseRobot_.x;
		double y = transDiffOfProjection * std::cos(resAngle) + poseRobot_.y;

		return VSM::Vector3(x, y, resAngle);
	}

	std::vector<std::pair<double, double>> CollavDWA::calcFuturePointCloud(std::pair<double, double> vSample, std::vector<std::pair<double, double>>& laserscanPointCloud)
	{
		std::vector<std::pair<double, double>> outVec;

		double transDiffOfProjection = vSample.first * deltaT_;
		double rotDiffOfProjection = vSample.second * deltaT_;

		rotDiffOfProjection = wrapAngle(rotDiffOfProjection);

		double x = transDiffOfProjection * (-std::sin(rotDiffOfProjection));
		double y = transDiffOfProjection * std::cos(rotDiffOfProjection);

		VSM::Matrix3x3 orientation = VSM::Matrix3x3(true);
		orientation.setRotZ(rotDiffOfProjection, true);
		VSM::Frame fr(orientation, VSM::Vector3(x, y, 0));

		for (auto& it : laserscanPointCloud)
		{
			VSM::Vector3 transformedPoint = fr.invTrafo(VSM::Vector3(it.first, it.second, 0));
			outVec.push_back(std::make_pair(transformedPoint.x, transformedPoint.y));
		}
		
		return outVec;
	}

	std::pair<double, double> CollavDWA::calculateMovementControl(
		VSM::Vector3 poseRobot,
		VSM::Vector3 poseGoal,
		double velTrans,
		double velRot,
		double dT,
		std::vector<std::pair<double, double>>& laserscanPointCloud)
	{
		deltaT_ = dT;
		velTrans_ = velTrans;
		velRot_ = velRot;

		poseRobot_ = poseRobot;
		poseGoal_ = poseGoal;

		/// (0) add map points to laserscanPointCloud
		collisionPointCloud_.clear();
		collisionPointCloud_ = laserscanPointCloud;
		if (ptrMap_)
		{
			if (useMapPoints_)
			{
				collisionPointCloud_.clear();
				std::vector<std::pair<double, double>> mapPointCloud = ptrMap_->getOccupiedCellPointCloudInRobotCoords(poseRobot_, true);
				collisionPointCloud_.reserve(mapPointCloud.size() + laserscanPointCloud.size());
				collisionPointCloud_.insert(collisionPointCloud_.end(), laserscanPointCloud.begin(), laserscanPointCloud.end());
				collisionPointCloud_.insert(collisionPointCloud_.end(), mapPointCloud.begin(), mapPointCloud.end());
			}	
		}

		// abort if points in robot footprint
		for (auto& it : collisionPointCloud_)
		{
			if (std::fabs(it.first) <= robotWidth_ && std::fabs(it.second) <= robotLength_)
				return std::make_pair(0.0, 0.0);
		}


		/// (1) get bounds of dynamic window
		vtMax_ = velTrans_ + transAccUpBound_ * deltaT_;
		vtMin_ = velTrans_ + transAccLowBound_ * deltaT_;
		vrMax_ = velRot_ + rotAccUpBound_ * deltaT_;
		vrMin_ = velRot_ + rotAccLowBound_ * deltaT_;


		double minVelFactor = 0.2;
		double yDiff = poseGoal_.getY() - poseRobot_.getY();
		double xDiff = poseGoal_.getX() - poseRobot_.getX();

		// robot near goal
		double distToGoal = std::hypot(xDiff, yDiff);
		if (distToGoal < 2.0)
		{
			vtMax_ = (distToGoal - 0.2) / 1.8 * (transVelUpBound_ * (1 - minVelFactor)) + transVelUpBound_ * minVelFactor;

			if (distToGoal < 0.2)
				vtMax_ = transVelUpBound_ * minVelFactor;
		}

		double safetyMaxArea = 1.0;
		double safetyMinArea = 0.1;

		// clearance dep max speed
		double robotClearance = getMaxClearanceToPointCloud(collisionPointCloud_);
		if (robotClearance < safetyMaxArea)
		{
			if (robotClearance < safetyMinArea)
			{
				vtMax_ = std::min(vtMax_, minVelFactor * transVelUpBound_);
			}
			else
			{
				double newVtMaxFactor = robotClearance / (safetyMaxArea - safetyMinArea);
				double newVtMax = newVtMaxFactor * (1 - minVelFactor) + minVelFactor;
				vtMax_ = std::min(vtMax_, newVtMax);
			}
		}

		double frontDist = std::numeric_limits<double>::max();
		for (auto& it : laserscanPointCloud)
		{
			if (std::fabs(it.first) > robotWidth_)
				continue;

			if (it.second < robotLength_)
				continue;

			frontDist = std::min(frontDist, it.second - robotLength_);
		}

		// handle overflow
		if (vtMax_ > transVelUpBound_)
			vtMax_ = transVelUpBound_;
		if (vtMin_ < transVelLowBound_)
			vtMin_ = transVelLowBound_;
		if (vrMax_ > rotVelUpBound_)
			vrMax_ = rotVelUpBound_;
		if (vrMin_ < rotVelLowBound_)
			vrMin_ = rotVelLowBound_;

		/// (2) sample numRadialPaths_ radies inside dynamic window
      vecRadius_.clear();
		if (distToGoal < 0.2)
			vecRadius_ = { 0.0, -0.0 };
		else
			vecRadius_ = sampleRadialPaths();

		collPoints_.clear();
      intersectPoints_.clear();
		std::vector<double> vecCollisionDist;
		std::vector<std::pair<double, double>> vecAdmissibleSpeed;
		/// (3) for each radius r calculate if collision may happen on the arc
		for (int jj = 0; jj < vecRadius_.size(); jj++)
		{
			double itRadius = vecRadius_.at(jj);
			double collDist = std::numeric_limits<double>::max();
			std::pair<double, double> maxAdmSpeed = std::make_pair(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
			std::pair<double, double> minAdmSpeed = getMinVelocityInDW(itRadius);

			/// (4) calculate collision distance and maximal admissable speed
			/// max admissable speed is determined by dynamic window if no collision or outside dw
			if (!calculateCollisionWithPointCloud(itRadius, collisionPointCloud_, collDist, maxAdmSpeed))
			{
				maxAdmSpeed = getMaxVelocityInDW(itRadius);
			}

			// no admissible speed in dw
			if (maxAdmSpeed.first < minAdmSpeed.first || frontDist < 0.1)
				continue;

			if (!isInDW(maxAdmSpeed))
			{
				maxAdmSpeed = getMaxVelocityInDW(itRadius);
			}

			/// (5) sample down from max adm speed (special cases!!)
			double vtStep = (maxAdmSpeed.first - minAdmSpeed.first) / numSamplesPerRadius_;
			double vrStep = (maxAdmSpeed.second - minAdmSpeed.second) / numSamplesPerRadius_;
			for (int kk = 0; kk < numSamplesPerRadius_; kk++)
			{
				vecCollisionDist.push_back(collDist);
				vecAdmissibleSpeed.push_back(
					std::make_pair(minAdmSpeed.first + kk * vtStep, minAdmSpeed.second + kk * vrStep));
			}
			
		}

		if (vecAdmissibleSpeed.empty())
		{
			return std::make_pair(0.0, 0.0);
		}

		/// (6) calculate objective function for each velocity sample (v,w)
		std::vector<double> vecWeights = calcWeightingNormed(vecAdmissibleSpeed, vecCollisionDist, collisionPointCloud_);

		/// (7) choose sample with maximum value
		std::vector<double>::iterator bestWeightIt = std::max_element(std::begin(vecWeights), std::end(vecWeights));
		std::pair<double, double> bestAdmSpeed = vecAdmissibleSpeed.at(std::distance(std::begin(vecWeights), bestWeightIt));



		return bestAdmSpeed;
	}

	void CollavDWA::setGridmap(VSLibMarkerLocalization::Gridmap& map)
	{
		ptrMap_ = &map;
	}

}//end namespace