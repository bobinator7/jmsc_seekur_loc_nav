#include "../VSLibMarkerLocalization2DEKF.h"

// tools
#include "../VSLibMarkerLocalizationTools.h"
#include "../VSLibMarkerLocalizationTools.hpp"

// 3rd party

namespace VSLibMarkerLocalization
{

	EKF2DInstance::EKF2DInstance()
		: numMarkerInDynamEKFMap_(0)
	{
		matR_ = Eigen::MatrixXd::Identity(3, 3);
		matQ_ = Eigen::MatrixXd::Identity(3, 3);

		vecMu_ = Eigen::VectorXd::Zero(3);
		matSigma_ = Eigen::MatrixXd::Identity(3, 3);
	}

	EKF2DInstance::~EKF2DInstance()
	{
	}

	void EKF2DInstance::reset()
	{
		//matR_ = Eigen::MatrixXd::Identity(3, 3);
		//matQ_ = Eigen::MatrixXd::Identity(3, 3);

		vecMu_ = Eigen::VectorXd::Zero(3);
		matSigma_ = Eigen::MatrixXd::Identity(3, 3);

		numMarkerInDynamEKFMap_ = 0;
		markerIndices_.clear();
	}

	void EKF2DInstance::setRobotPose(const VSM::Vector3& robotInitialPose, const VSM::Matrix3x3 robotInitialCovariance = VSM::Matrix3x3(1.,0.,0.,0.,1.,0.,0.,0.,1.))
	{
		// third component is NOT z, but the angle in the 2D plane
		vecMu_ = Eigen::Vector3d(robotInitialPose[0], robotInitialPose[1], robotInitialPose[2]);

		matSigma_ = Eigen::MatrixXd(3, 3);
		matSigma_ << robotInitialCovariance.getElement(0, 0), robotInitialCovariance.getElement(0, 1), robotInitialCovariance.getElement(0, 2),
						 robotInitialCovariance.getElement(1, 0), robotInitialCovariance.getElement(1, 1), robotInitialCovariance.getElement(1, 2),
						 robotInitialCovariance.getElement(2, 0), robotInitialCovariance.getElement(2, 1), robotInitialCovariance.getElement(2, 2);
	}

	VSM::Vector3 EKF2DInstance::getRobotPose()
	{
      //VSM::Vector3 tmp(vecMu_(0), vecMu_(1), vecMu_(2));
		return VSM::Vector3(vecMu_(0), vecMu_(1), vecMu_(2));
	}

	VSM::Matrix3x3 EKF2DInstance::getRobotCovariance()
	{
      VSM::Matrix3x3 tmp(matSigma_(0, 0), matSigma_(0, 1), matSigma_(0, 2),
         matSigma_(1, 0), matSigma_(1, 1), matSigma_(1, 2),
         matSigma_(2, 0), matSigma_(2, 1), matSigma_(2, 2));
      return tmp;
	}

	bool EKF2DInstance::checkMarkerIDExists(const int& markerID)
	{
		for (int ii = 0; ii < numMarkerInDynamEKFMap_; ii++)
		{
			if (markerIndices_.at(ii) == markerID)
			{
				return true;
			}
		}
		return false;
	}

	VSM::Vector3 EKF2DInstance::getMarkerPose(const int& markerID)
	{
      
		for (int ii = 0; ii < numMarkerInDynamEKFMap_; ii++)
		{
			if (markerIndices_[ii] == markerID)
			{
				int stateVectorCursor = 3 + ii * 3;
				assert(stateVectorCursor < vecMu_.size());

            VSM::Vector3 tmp(vecMu_(stateVectorCursor), vecMu_(stateVectorCursor + 1), vecMu_(stateVectorCursor + 2));
            return tmp;
			}
		}

		// no marker with markerID found!
      return VSM::Vector3(0., 0., 0.);
	}

	VSM::Matrix3x3 EKF2DInstance::getMarkerCovariance(const int& markerID)
	{
		for (int ii = 0; ii < numMarkerInDynamEKFMap_; ii++)
		{
			if (markerIndices_[ii] == markerID)
			{
				int stateVectorCursor = 3 + ii * 3;
				assert(stateVectorCursor < matSigma_.rows() || stateVectorCursor < matSigma_.cols());
            VSM::Matrix3x3 tmp(matSigma_(stateVectorCursor, stateVectorCursor), matSigma_(stateVectorCursor, stateVectorCursor + 1), matSigma_(stateVectorCursor, stateVectorCursor + 2),
               matSigma_(stateVectorCursor + 1, stateVectorCursor), matSigma_(stateVectorCursor + 1, stateVectorCursor + 1), matSigma_(stateVectorCursor + 1, stateVectorCursor + 2),
               matSigma_(stateVectorCursor + 2, stateVectorCursor), matSigma_(stateVectorCursor + 2, stateVectorCursor + 1), matSigma_(stateVectorCursor + 2, stateVectorCursor + 2));
            return tmp;
			}
		}

		// no marker with markerID found!
      return VSM::Matrix3x3();
	}

	std::unordered_map<int, VSM::Vector3> EKF2DInstance::getMarkerPoseMap()
	{
		std::unordered_map<int, VSM::Vector3> outMap = staticMarkerMap_;

		for (int ii = 0; ii < markerIndices_.size(); ii++)
		{
			outMap[markerIndices_[ii]] = VSM::Vector3(vecMu_(3 * ii + 3), vecMu_(3 * ii + 4), vecMu_(3 * ii + 5));
		}

		return outMap;
	}

	std::unordered_map<int, VSM::Matrix3x3> EKF2DInstance::getMarkerCovarianceMap()
	{
		std::unordered_map<int, VSM::Matrix3x3> outMap;

		for (int ii = 0; ii < markerIndices_.size(); ii++)
		{
			int stateVectorCursor = 3 + ii * 3;
			assert(stateVectorCursor < matSigma_.rows() || stateVectorCursor < matSigma_.cols());
			VSM::Matrix3x3 tmp(matSigma_(stateVectorCursor, stateVectorCursor), matSigma_(stateVectorCursor, stateVectorCursor + 1), matSigma_(stateVectorCursor, stateVectorCursor + 2),
				matSigma_(stateVectorCursor + 1, stateVectorCursor), matSigma_(stateVectorCursor + 1, stateVectorCursor + 1), matSigma_(stateVectorCursor + 1, stateVectorCursor + 2),
				matSigma_(stateVectorCursor + 2, stateVectorCursor), matSigma_(stateVectorCursor + 2, stateVectorCursor + 1), matSigma_(stateVectorCursor + 2, stateVectorCursor + 2));
			outMap[markerIndices_[ii]] = tmp;
		}

		for (auto it : staticMarkerMap_)
		{
			outMap[it.first] = VSM::Matrix3x3();
		}

		return outMap;
	}

	//int EKF2DInstance::getNumMarkersInMap()
	//{
	//	return numMarkerInDynamEKFMap_;
	//}

	void EKF2DInstance::setSystemNoiseQ(const double& varX, const double& varY, const double& varTheta)
	{
		matQ_ << varX, 0., 0.,
					0., varY, 0.,
					0., 0., varTheta;
	}

	void EKF2DInstance::setSystemNoiseQ(const VSM::Matrix3x3& matIn)
	{
		matQ_ << matIn.getElement(0, 0), matIn.getElement(0, 1), matIn.getElement(0, 2),
					matIn.getElement(1, 0), matIn.getElement(1, 1), matIn.getElement(1, 2),
					matIn.getElement(2, 0), matIn.getElement(2, 1), matIn.getElement(2, 2);
	}

	void EKF2DInstance::setMeasurementNoiseR(const double& varX, const double& varY, const double& varTheta)
	{
		matR_ << varX, 0., 0.,
					0., varY, 0.,
					0., 0., varTheta;
	}

	void EKF2DInstance::setMeasurementNoiseR(const VSM::Matrix3x3& matIn)
	{
		matR_ << matIn.getElement(0, 0), matIn.getElement(0, 1), matIn.getElement(0, 2),
					matIn.getElement(1, 0), matIn.getElement(1, 1), matIn.getElement(1, 2),
					matIn.getElement(2, 0), matIn.getElement(2, 1), matIn.getElement(2, 2);
	}

	void EKF2DInstance::predictStep(const double& deltaVelocity, const double& deltaRotVelIn, const double& deltaTIn)
	{
		double deltaDistance = deltaVelocity * deltaTIn;
		double deltaThetaIn = deltaRotVelIn * deltaTIn;

		predictStep(deltaDistance, deltaThetaIn);
	}

	void EKF2DInstance::predictStep(const double& deltaDistance, const double& deltaThetaIn)
	{
		// TODO: move to motion model

		// no/small movement!
		if ((deltaDistance < FLT_EPSILON && deltaDistance > -FLT_EPSILON) && (deltaThetaIn < FLT_EPSILON && deltaThetaIn > -FLT_EPSILON))
			return;

		const int n = matSigma_.rows();

		vecMu_(0) -= deltaDistance * std::sin(vecMu_(2));
		vecMu_(1) += deltaDistance * std::cos(vecMu_(2));
		vecMu_(2) += deltaThetaIn;
		vecMu_(2) = wrapAngle(vecMu_(2));

		Eigen::Matrix3d matJxr;
		matJxr << 1, 0, -deltaDistance*std::cos(vecMu_(2)),
					 0, 1, -deltaDistance*std::sin(vecMu_(2)),
					 0, 0, 1;
		Eigen::MatrixXd matJx = Eigen::MatrixXd::Identity(n, n);
		matJx.block<3, 3>(0, 0) = matJxr;

		Eigen::Matrix<double, 3, 2> matJur;
		matJur << -std::sin(vecMu_(2)), 0,
					 std::cos(vecMu_(2)), 0,
					 0, 1;
		Eigen::MatrixXd matJu;
		matJu.setZero(n, 3);
		matJu.block<3, 2>(0, 0) = matJur;

		matSigma_ = matJx * matSigma_ * matJx.transpose() + matJu * matQ_ * matJu.transpose();
	}

	void EKF2DInstance::updateStep(const VSM::MatrixNxM& measurementIn)
	{
		const int m = matSigma_.rows();
		const int n = measurementIn.columns();

		// abort if no measurements made
		if (n <= 0)
			return;

		// z = G * x
		// Da Beobachtung z nicht immer gleich sind (unterschiedliche IDs),
		// ist der z Vektor jedes Mal anders und somit auch G.
		// Im Folgenden wird der z Vektor in der Reihenfolge definiert
		// wie sie nacheinander in Spalten im Eingang measurementIn stehen
		
		Eigen::VectorXd vecZ, vecH, vecSensDiff;
		vecZ.setZero(3 * n, 1);
		vecH.setZero(3 * n, 1);
		vecSensDiff.setZero(3 * n, 1);
		matG_.setZero(3 * n, m);
		for (int ii = 0; ii < n; ii++)
		{
			// get z
			vecZ(3 * ii) = measurementIn.getElement(1, ii);
			vecZ(3 * ii + 1) = measurementIn.getElement(2, ii);
			vecZ(3 * ii + 2) = wrapAngle(measurementIn.getElement(3, ii));

			// get index of corresponding marker in state x
			int currentMarkerCursor = -1;
			for (int jj = 0; jj < markerIndices_.size(); jj++)
			{
				if (markerIndices_.at(jj) == measurementIn.getElement(0, ii))
				{
					currentMarkerCursor = 3 * jj + 3;
				}
			}

			//
			auto search = staticMarkerMap_.find(measurementIn.getElement(0, ii));
			if (search != staticMarkerMap_.end())
			{

				Eigen::Matrix3d matGr;
				double gr13 = -(search->second[0] - vecMu_(0)) * std::sin(vecMu_(2))
					+ (search->second[1] - vecMu_(1)) * std::cos(vecMu_(2));
				double gr23 = -(search->second[0] - vecMu_(0)) * std::cos(vecMu_(2))
					- (search->second[1] - vecMu_(1)) * std::sin(vecMu_(2));
				matGr << -std::cos(vecMu_(2)), -std::sin(vecMu_(2)), gr13,
					std::sin(vecMu_(2)), -std::cos(vecMu_(2)), gr23,
					0, 0, -1;

				matG_.block<3, 3>(3 * ii, 0) = matGr;

				// measurement prediction from current state
				vecH(3 * ii) = search->second[0] * std::cos(vecMu_(2))
					+ search->second[1] * std::sin(vecMu_(2))
					- vecMu_(0) * std::cos(vecMu_(2))
					- vecMu_(1) * std::sin(vecMu_(2));
				vecH(3 * ii + 1) = -search->second[0] * std::sin(vecMu_(2))
					+ search->second[1] * std::cos(vecMu_(2))
					+ vecMu_(0) * std::sin(vecMu_(2))
					- vecMu_(1) * std::cos(vecMu_(2));
				vecH(3 * ii + 2) = search->second[2] - vecMu_(2);
				vecH(3 * ii + 2) = wrapAngle(vecH(3 * ii + 2));

				// calculate sensor difference with a wrapped angle in [-pi, pi]
				vecSensDiff(3 * ii) = vecZ(3 * ii) - vecH(3 * ii);
				vecSensDiff(3 * ii + 1) = vecZ(3 * ii + 1) - vecH(3 * ii + 1);
				if ((vecZ(3 * ii + 2) - vecH(3 * ii + 2)) > M_PI)
					vecSensDiff(3 * ii + 2) = vecZ(3 * ii + 2) - vecH(3 * ii + 2) - 2 * M_PI;
				else if ((vecZ(3 * ii + 2) - vecH(3 * ii + 2)) < -M_PI)
					vecSensDiff(3 * ii + 2) = vecZ(3 * ii + 2) - vecH(3 * ii + 2) + 2 * M_PI;
				else
					vecSensDiff(3 * ii + 2) = vecZ(3 * ii + 2) - vecH(3 * ii + 2);

				continue;
			}

			
			if (currentMarkerCursor == -1)
			{
				addMarker(measurementIn.getElement(0, ii)
					, measurementIn.getElement(1, ii)
					, measurementIn.getElement(2, ii)
					, measurementIn.getElement(3, ii));
				return;
			}
			
			/* debug testing */
			//if (abs(abs(vecZ(3 * ii + 2) - vecH(3 * ii + 2)) - (M_PI)) < 0.08)
			//	if (vecZ(3 * ii + 2) > vecH(3 * ii + 2))
			//		vecZ(3 * ii + 2) = vecZ(3 * ii + 2) - M_PI;
			//	else
			//		vecZ(3 * ii + 2) = vecZ(3 * ii + 2) + M_PI;
			/* debug testing */

			// create G matrix
			Eigen::Matrix3d matGr, matGm;
			double gr13 = -(vecMu_(currentMarkerCursor) - vecMu_(0)) * std::sin(vecMu_(2))
				+ (vecMu_(currentMarkerCursor + 1) - vecMu_(1)) * std::cos(vecMu_(2));
			double gr23 = -(vecMu_(currentMarkerCursor) - vecMu_(0)) * std::cos(vecMu_(2))
				- (vecMu_(currentMarkerCursor + 1) - vecMu_(1)) * std::sin(vecMu_(2));
			matGr << -std::cos(vecMu_(2)), -std::sin(vecMu_(2)), gr13,
						std::sin(vecMu_(2)), -std::cos(vecMu_(2)), gr23,
						0, 0, -1;
			matGm << std::cos(vecMu_(2)), std::sin(vecMu_(2)), 0,
						-std::sin(vecMu_(2)), std::cos(vecMu_(2)), 0,
						0, 0, 1;

			matG_.block<3, 3>(3 * ii, 0) = matGr;
			matG_.block<3, 3>(3 * ii, currentMarkerCursor) = matGm;

			// measurement prediction from current state
			vecH(3 * ii) = vecMu_(currentMarkerCursor) * std::cos(vecMu_(2))
				+ vecMu_(currentMarkerCursor + 1) * std::sin(vecMu_(2))
				- vecMu_(0) * std::cos(vecMu_(2))
				- vecMu_(1) * std::sin(vecMu_(2));
			vecH(3 * ii + 1) = -vecMu_(currentMarkerCursor) * std::sin(vecMu_(2))
				+ vecMu_(currentMarkerCursor + 1) * std::cos(vecMu_(2))
				+ vecMu_(0) * std::sin(vecMu_(2))
				- vecMu_(1) * std::cos(vecMu_(2));
			vecH(3 * ii + 2) = vecMu_(currentMarkerCursor + 2) - vecMu_(2);
			vecH(3 * ii + 2) = wrapAngle(vecH(3 * ii + 2));

			// calculate sensor difference with a wrapped angle in [-pi, pi]
			vecSensDiff(3 * ii) = vecZ(3 * ii) - vecH(3 * ii);
			vecSensDiff(3 * ii + 1) = vecZ(3 * ii + 1) - vecH(3 * ii + 1);
			if ((vecZ(3 * ii + 2) - vecH(3 * ii + 2)) > M_PI)
				vecSensDiff(3 * ii + 2) = vecZ(3 * ii + 2) - vecH(3 * ii + 2) - 2 * M_PI;
			else if ((vecZ(3 * ii + 2) - vecH(3 * ii + 2)) < -M_PI)
				vecSensDiff(3 * ii + 2) = vecZ(3 * ii + 2) - vecH(3 * ii + 2) + 2 * M_PI;
			else
				vecSensDiff(3 * ii + 2) = vecZ(3 * ii + 2) - vecH(3 * ii + 2);
		}

		Eigen::MatrixXd matR;
		matR.setZero(3 * n, 3 * n);
		for (int jj = 0; jj < n; jj++)
		{
			matR.block<3, 3>(3 * jj, 3 * jj) = matR_;
		}

		// Kalman Gain
		matK_.setZero(m, 3);
		Eigen::MatrixXd matInnov, matInnovInv;
		matInnov = matG_ * matSigma_ * matG_.transpose() + matR;
		matInnovInv = matInnov.inverse();
		matK_ = matSigma_ * matG_.transpose() * matInnovInv;		// stable?

		
		vecMu_ = vecMu_ + matK_ * vecSensDiff;
		// angle between 0 and 2 * pi
		for (int kk = 0; kk < n; kk++)
		{
			vecMu_(3 * kk + 2) = wrapAngle(vecMu_(3 * kk + 2));
		}

		matSigma_ = (Eigen::MatrixXd::Identity(m, m) - matK_ * matG_) * matSigma_;
	}

	void EKF2DInstance::addMarker(const int& markerID, const double& markerX, const double& markerY, const double& markerTheta)
	{
		auto search = staticMarkerMap_.find(markerID);
		if (search != staticMarkerMap_.end())
			return;

		markerIndices_.push_back(markerID);
		numMarkerInDynamEKFMap_++;

		// 2D Trafo
		// TODO: check coordinate system of robot
		double markerXInMapCoords = vecMu_(0) + markerX * std::cos(vecMu_(2)) - markerY * std::sin(vecMu_(2));
		double markerYInMapCoords = vecMu_(1) + markerX * std::sin(vecMu_(2)) + markerY * std::cos(vecMu_(2));
		double markerThetaInMapCoords = vecMu_(2) + markerTheta;

		// resize state and covariance matrix
		int newMarkerIndexCursor = vecMu_.size();
		vecMu_.conservativeResizeLike(Eigen::VectorXd::Zero(newMarkerIndexCursor + 3));
		matSigma_.conservativeResizeLike(Eigen::MatrixXd::Zero(newMarkerIndexCursor + 3, newMarkerIndexCursor + 3));

		// init marker mean
		vecMu_(newMarkerIndexCursor) = markerXInMapCoords;
		vecMu_(newMarkerIndexCursor + 1) = markerYInMapCoords;
		vecMu_(newMarkerIndexCursor + 2) = markerThetaInMapCoords;

		// init marker covariance
		Eigen::Matrix3d matL;
		matL << std::cos(vecMu_(2)), -std::sin(vecMu_(2)), 0,
				  std::sin(vecMu_(2)), std::cos(vecMu_(2)), 0,
				  0, 0, 1;

		matSigma_.block<3, 3>(newMarkerIndexCursor, newMarkerIndexCursor) = matL * matQ_ * matL.transpose();
	}

	void EKF2DInstance::setStaticMap(std::unordered_map<int, VSM::Vector3> inStaticMap)
	{
		staticMarkerMap_ = inStaticMap;

		for (int ii = markerIndices_.size() - 1; ii >= 0; ii--)
		{
			int stateMarkerID = markerIndices_.at(ii);
			// remove current markers in state and matrices if in static map
			auto search = staticMarkerMap_.find(stateMarkerID);
			if (search != staticMarkerMap_.end())
			{
				markerIndices_.erase(markerIndices_.begin() + ii);
				numMarkerInDynamEKFMap_--;


				int stateMarkerIndex = 3 * ii + 3;

				// crop state covariance matrix
				unsigned int numRows = matSigma_.rows() - 3;
				unsigned int numCols = matSigma_.cols() - 3;
				matSigma_.block(stateMarkerIndex, 0, numRows - stateMarkerIndex, matSigma_.cols()) = matSigma_.block(stateMarkerIndex + 3, 0, numRows - stateMarkerIndex, matSigma_.cols());
				matSigma_.block(0, stateMarkerIndex, numRows, numCols - stateMarkerIndex) = matSigma_.block(0, stateMarkerIndex + 3, numRows, numCols - stateMarkerIndex);
				matSigma_.conservativeResize(numRows, numCols);

				// crop state vector
				vecMu_.segment(stateMarkerIndex, numRows - stateMarkerIndex) = vecMu_.tail(numRows - stateMarkerIndex);
				vecMu_.conservativeResize(numRows);
			}
		}
	}

	void EKF2DInstance::resetStaticMap()
	{
		staticMarkerMap_.clear();
	}
}//end namespace