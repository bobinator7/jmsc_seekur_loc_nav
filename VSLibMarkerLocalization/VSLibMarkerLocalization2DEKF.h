#ifndef VSLibMarkerLocalization2DEKFH
#define VSLibMarkerLocalization2DEKFH

#include "VSLibMarkerLocalizationExport.h"

// specific
#include <array>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>

#include <Lib/VSM/VSMVector3.h>
#include <Lib/VSM/VSMMatrix3x3.h>
#include <Lib/VSM/VSMMatrixNxM.h>

namespace VSLibMarkerLocalization
{
	class VSLibMarkerLocalization_DECLSPEC EKF2DInstance
	{
	
	public:
		EKF2DInstance();
		~EKF2DInstance();

		// reset filter
		void reset();

		// get robot vars
		VSM::Vector3 getRobotPose();
		VSM::Matrix3x3 getRobotCovariance();

		// get marker vars
		bool checkMarkerIDExists(const int& markerID);
		
		//int getNumMarkersInMap();

		std::unordered_map<int, VSM::Vector3> getMarkerPoseMap();
		std::unordered_map<int, VSM::Matrix3x3> getMarkerCovarianceMap();

		// set filter vars
		void setSystemNoiseQ(const double& varX, const double& varY, const double& varTheta);
		void setSystemNoiseQ(const VSM::Matrix3x3& matIn);
		void setMeasurementNoiseR(const double& varX, const double& varY, const double& varTheta);
		void setMeasurementNoiseR(const VSM::Matrix3x3& matIn);

		// initialize filter (resets)
		void setRobotPose(const VSM::Vector3& robotInitialPose, const VSM::Matrix3x3 robotInitialCovariance);

		// execute filter steps
		void predictStep(const double& deltaDistance, const double& deltaThetaIn);
		void predictStep(const double& deltaVelocity, const double& deltaRotVelIn, const double& deltaTIn);
		void updateStep(const VSM::MatrixNxM& measurementIn);
		void addMarker(const int& markerID, const double& markerX, const double& markerY, const double& markerTheta);

		// set static marker map
		void setStaticMap(std::unordered_map<int, VSM::Vector3> inStaticMap);
		void resetStaticMap();

	private:
		// noise covariances
		Eigen::Matrix3d matR_, matQ_;

		// state
		Eigen::VectorXd vecMu_;
		Eigen::MatrixXd matSigma_;

		// EKF Matrices (Jx Ju as system Jacobian, G as measurement Jacobian, K as Kalman Gain)
		Eigen::MatrixXd matJx_, matJu_, matG_, matK_;

		// marker properties
		std::vector<int> markerIndices_;
		int numMarkerInDynamEKFMap_;
		std::unordered_map<int, VSM::Vector3> staticMarkerMap_;

		VSM::Vector3 getMarkerPose(const int& markerID);
		VSM::Matrix3x3 getMarkerCovariance(const int& markerID);

	};
} //end namespace

#endif 
