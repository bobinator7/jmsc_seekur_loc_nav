#ifndef VSLibMobileRobotNavigationCollavDWAH
#define VSLibMobileRobotNavigationCollavDWAH

#include "VSLibMobileRobotNavigationExport.h"

#include "../VSLibMarkerLocalization/VSLibMarkerLocalizationGridmap.h"

#include "Lib/VSM/VSMVector3.h"


namespace VSLibMobileRobotNavigation
{
	/*	Current implementation of collision avoidance algorithm based on
	 *	"The Dynamic Window Approach to Collision Avoidance", Fox 1997 (base paper)
	 *	"Real-Time Obstacle Avoidance For Polygonal Robots With A Reduced Dynamic Window", Arras 2002 (robot frame transform)
	 * "Investigation of Dynamic Window Based Navigation Algorithms on a Real Robot", Maroti 2013 (radial sampling)
	 *
	 * Base assumption made:
	 * - navigation space is a 2D plane
	 * - robot motion model: unicycle model
	 * - robot footprint is a rectangle with its pose in the centre 
	 *   (enhancable to arbitrary robot footprints with offset)
	 */
	class VSLibMobileRobotNavigation_DECLSPEC CollavDWA
	{
	public:
		CollavDWA();
		//CollavDWA(const double& robotWidth, const double& robotLength);
		~CollavDWA();

		// get algorithm parameters
		VSM::Vector3 getRobotPose();
		std::vector<std::pair<double, double>> getCollisionPointCloud();
		std::vector<std::pair<double, double>> getCollisionPoints();
      std::vector<std::pair<double, double>> getIntersectionPoints();
      std::vector<double> getVecRadius();


		// set algorithm parameters
		void setRobotProperties(const double& robotWidth, const double& robotLength);
		void setRobotVelocityBounds(const double& transLower, const double& transUpper, const double& rotLower, const double& rotUpper);
		void setRobotAccelerationBounds(const double& transLower, const double& transUpper, const double& rotLower, const double& rotUpper);
		void setObjectiveFunctionParam(const double& sigma, const double& alpha, const double& beta, const double& gamma);

		// set gridmap ptr
		void setUseMapPointsFlag(bool flag);
		void setGridmap(VSLibMarkerLocalization::Gridmap& map);

		// actual calculation of control vars
		std::pair<double, double> calculateMovementControl(
			VSM::Vector3 poseRobot,
			VSM::Vector3 poseGoal,
			double velTrans,
			double velRot,
			double dT,
			std::vector<std::pair<double, double>>& laserscanPointCloud);

		VSM::Vector3 calcFuturePose(std::pair<double, double> vSample);
		std::vector<std::pair<double, double>> calcFuturePointCloud(std::pair<double, double> vSample, std::vector<std::pair<double, double>>& laserscanPointCloud);

	private:
		// robot paramters
		double robotWidth_, robotLength_;
		double transVelLowBound_, transVelUpBound_, rotVelLowBound_, rotVelUpBound_;
		double transAccLowBound_, transAccUpBound_, rotAccLowBound_, rotAccUpBound_;

		// tmp vars
		double deltaT_;
		double velTrans_, velRot_;
		double vtMin_, vtMax_, vrMin_, vrMax_;
		VSM::Vector3 poseRobot_;
		VSM::Vector3 poseGoal_;
		std::vector<std::pair<double, double>> collisionPointCloud_;
		std::vector<std::pair<double, double>> collPoints_;
      std::vector<std::pair<double, double>> intersectPoints_;
      std::pair<double, double> tmpIntersectPoint_;
      std::vector<double> vecRadius_;

		// general algorithm parameters
		unsigned int numRadialPaths_;
		int numSamplesPerRadius_;
		//unsigned int numSamplePoints_;

		// objective function parameters
		double sigma_, alpha_, beta_, gamma_;

		// algorithm subfunctions
		std::vector<double> sampleRadialPaths();
		bool calculateCollisionWithPointCloud(
			double radius, 
			std::vector<std::pair<double, double>>& laserscanPointCloud,
			double& collDist,
			std::pair<double, double>& admSpeed);
		double calcRadialCollisionDistance(double radiusPivotpoint, double distPointToPivotpoint, std::pair<double, double>& scanPoint);
		double getMaxClearanceToPointCloud(std::vector<std::pair<double, double>>& pointCloudInRobotFrame);
		
		//double (CollavDWA::*calcWeighting)(double collDist, std::pair<double, double> vSample);
		double (CollavDWA::*calcWeighting)(std::pair<double, double> vSample);
		double calcWeightingBasic(double collDist, std::pair<double, double> vSample);
		double calcWeightingObstacleCostmap(std::pair<double, double> vSample);
		std::vector<double> calcWeightingNormed(std::vector<std::pair<double, double>>& vSamples, std::vector<double>& vecCollDists, std::vector<std::pair<double, double>>& laserscanPointCloud);

		//VSM::Vector3 calcFuturePose(std::pair<double, double> vSample);

		bool isInDW(std::pair<double, double> vPt);
		std::pair<double, double> getMaxVelocityInDW(double radiusPivotpoint);		// based on assumption only pos vt
		std::pair<double, double> getMinVelocityInDW(double radiusPivotpoint);

		// gridmap
		bool useMapPoints_;
		VSLibMarkerLocalization::Gridmap* ptrMap_;
		

	};
}//end namespace

#endif 
