#include "../VSLibMarkerLocalization2DUKF.h"

// tools
#include "../VSLibMarkerLocalizationTools.h"
#include "../VSLibMarkerLocalizationTools.hpp"

// 3rd party

namespace VSLibMarkerLocalization
{

	UKF2DInstance::UKF2DInstance()
		: numMarkerInUKFMap_(0)
	{
		matR_ = Eigen::MatrixXd::Identity(3, 3);
		matQ_ = Eigen::MatrixXd::Identity(3, 3);

		vecMu_ = Eigen::VectorXd::Zero(3);
		matSigma_ = Eigen::MatrixXd::Identity(3, 3);

		setUKFParam(0.001, 2, 0);
	}

	UKF2DInstance::~UKF2DInstance()
	{
	}

	void UKF2DInstance::reset()
	{
		vecMu_ = Eigen::VectorXd::Zero(3);
		matSigma_ = Eigen::MatrixXd::Identity(3, 3);

		setUKFParam(alpha_, beta_, kappa_);

		generateStateSigmaPoints(3);

		numMarkerInUKFMap_ = 0;
		markerIndices_.clear();
	}

	void UKF2DInstance::setRobotPose(const VSM::Vector3& robotInitialPose, const VSM::Matrix3x3 robotInitialCovariance = VSM::Matrix3x3(1., 0., 0., 0., 1., 0., 0., 0., 1.))
	{
		// third component is NOT z, but the angle in the 2D plane
		vecMu_ = Eigen::Vector3d(robotInitialPose[0], robotInitialPose[1], robotInitialPose[2]);

		matSigma_ = Eigen::MatrixXd(3, 3);
		matSigma_ << robotInitialCovariance.getElement(0, 0), robotInitialCovariance.getElement(0, 1), robotInitialCovariance.getElement(0, 2),
			robotInitialCovariance.getElement(1, 0), robotInitialCovariance.getElement(1, 1), robotInitialCovariance.getElement(1, 2),
			robotInitialCovariance.getElement(2, 0), robotInitialCovariance.getElement(2, 1), robotInitialCovariance.getElement(2, 2);
	}

	VSM::Vector3 UKF2DInstance::getRobotPose()
	{
		return VSM::Vector3(vecMu_(0), vecMu_(1), vecMu_(2));
	}

	VSM::Matrix3x3 UKF2DInstance::getRobotCovariance()
	{
		VSM::Matrix3x3 tmp(matSigma_(0, 0), matSigma_(0, 1), matSigma_(0, 2),
			matSigma_(1, 0), matSigma_(1, 1), matSigma_(1, 2),
			matSigma_(2, 0), matSigma_(2, 1), matSigma_(2, 2));
		return tmp;
	}

	bool UKF2DInstance::checkMarkerIDExists(const int& markerID)
	{
		for (int ii = 0; ii < numMarkerInUKFMap_; ii++)
		{
			if (markerIndices_.at(ii) == markerID)
			{
				return true;
			}
		}
		return false;
	}

	VSM::Vector3 UKF2DInstance::getMarkerPose(const int& markerID)
	{

		for (int ii = 0; ii < numMarkerInUKFMap_; ii++)
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

	VSM::Matrix3x3 UKF2DInstance::getMarkerCovariance(const int& markerID)
	{
		for (int ii = 0; ii < numMarkerInUKFMap_; ii++)
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

	std::unordered_map<int, VSM::Vector3> UKF2DInstance::getMarkerPoseMap()
	{
		std::unordered_map<int, VSM::Vector3> outMap = staticMarkerMap_;

		for (int ii = 0; ii < markerIndices_.size(); ii++)
		{
			outMap[markerIndices_[ii]] = VSM::Vector3(vecMu_(3 * ii + 3), vecMu_(3 * ii + 4), vecMu_(3 * ii + 5));
		}

		return outMap;
	}

	std::unordered_map<int, VSM::Matrix3x3> UKF2DInstance::getMarkerCovarianceMap()
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

	void UKF2DInstance::setSystemNoiseQ(const double& varX, const double& varY, const double& varTheta)
	{
		matQ_ << varX, 0., 0.,
			0., varY, 0.,
			0., 0., varTheta;
	}

	void UKF2DInstance::setSystemNoiseQ(const VSM::Matrix3x3& matIn)
	{
		matQ_ << matIn.getElement(0, 0), matIn.getElement(0, 1), matIn.getElement(0, 2),
			matIn.getElement(1, 0), matIn.getElement(1, 1), matIn.getElement(1, 2),
			matIn.getElement(2, 0), matIn.getElement(2, 1), matIn.getElement(2, 2);
	}

	void UKF2DInstance::setMeasurementNoiseR(const double& varX, const double& varY, const double& varTheta)
	{
		matR_ << varX, 0., 0.,
			0., varY, 0.,
			0., 0., varTheta;
	}

	void UKF2DInstance::setMeasurementNoiseR(const VSM::Matrix3x3& matIn)
	{
		matR_ << matIn.getElement(0, 0), matIn.getElement(0, 1), matIn.getElement(0, 2),
			matIn.getElement(1, 0), matIn.getElement(1, 1), matIn.getElement(1, 2),
			matIn.getElement(2, 0), matIn.getElement(2, 1), matIn.getElement(2, 2);
	}

	Eigen::MatrixXd UKF2DInstance::generateStateSigmaPoints(const int& L)
	{
		VSLibMarkerLocalization::Tools::fixBrokenCorrelationMatrix(matSigma_);
		Eigen::MatrixXd matSQRT = varc_ * matSigma_;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(matSQRT);
		matSQRT = es.operatorSqrt();

		// TODO: better error handling!
		if (matSQRT.hasNaN())
			return Eigen::MatrixXd::Zero(L, 2 * L + 1);

		Eigen::MatrixXd matStateSigmaPoints;
		matStateSigmaPoints.setZero(L, 2 * L + 1);
		matStateSigmaPoints.col(0) = vecMu_;
		for (int ii = 1; ii < L + 1; ii++)
		{
			matStateSigmaPoints.col(ii) = vecMu_ + matSQRT.col(ii - 1);
			matStateSigmaPoints.col(ii + L) = vecMu_ - matSQRT.col(ii - 1);
		}

		return matStateSigmaPoints;
	}

	void UKF2DInstance::predictStep(const double& deltaVelocity, const double& deltaRotVelIn, const double& deltaTIn)
	{
		double deltaDistance = deltaVelocity * deltaTIn;
		double deltaThetaIn = deltaRotVelIn * deltaTIn;

		predictStep(deltaDistance, deltaThetaIn);
	}

	void UKF2DInstance::predictStep(const double& deltaDistance, const double& deltaThetaIn)
	{
		// no/small movement!
		if ((deltaDistance < FLT_EPSILON && deltaDistance > -FLT_EPSILON) && (deltaThetaIn < FLT_EPSILON && deltaThetaIn > -FLT_EPSILON))
			return;

		const int L = matSigma_.rows();

		// calc sigma points
		Eigen::MatrixXd matStateSigmaPoints = generateStateSigmaPoints(L);

		// propagate sigma points through system model
		for (int ii = 0; ii < 2 * L + 1; ii++)
		{
			matStateSigmaPoints(0, ii) -= deltaDistance * std::sin(matStateSigmaPoints(2, ii));
			matStateSigmaPoints(1, ii) += deltaDistance * std::cos(matStateSigmaPoints(2, ii));
			matStateSigmaPoints(2, ii) += deltaThetaIn;
		}

		// calculate mean from propagated sigma points
		vecMu_.setZero(L);
		for (int ii = 0; ii < 2 * L + 1; ii++)
		{
			vecMu_ += sigmaPointsWeightState_[ii] * matStateSigmaPoints.col(ii);
		}
		VSLibMarkerLocalization::Tools::stateWrapAngle(vecMu_);

		// calculate sigma from deviation mean to sigma point
		matSigma_.setZero(L, L);
		for (int ii = 0; ii < 2 * L + 1; ii++)
		{
			Eigen::VectorXd vecDiff = matStateSigmaPoints.col(ii) - vecMu_;
			VSLibMarkerLocalization::Tools::stateWrapAngle(vecDiff);
			matSigma_ += sigmaPointsWeightCovariance_[ii] * (vecDiff * vecDiff.transpose());
		}
		Eigen::MatrixXd matQResized = Eigen::MatrixXd::Zero(matSigma_.rows(), matSigma_.cols());
		matQResized.block<3, 3>(0, 0) = matQ_;
		matSigma_ += matQResized;
	}

	void UKF2DInstance::updateStep(const VSM::MatrixNxM& measurementIn)
	{
		const int L = matSigma_.rows();
		const int n = measurementIn.columns();

		// abort if no measurements made
		if (n <= 0)
			return;

		// define measurement vector in order of input
		std::vector<int> inputIDs;
		Eigen::VectorXd vecZ(3 * n);
		for (int jj = 0; jj < n; jj++)
		{
			inputIDs.push_back(measurementIn.getElement(0, jj));

			vecZ(3 * jj) = measurementIn.getElement(1, jj);
			vecZ(3 * jj + 1) = measurementIn.getElement(2, jj);
			vecZ(3 * jj + 2) = wrapAngle(measurementIn.getElement(3, jj));
		}

		// recalculate sigma points from state distribution
		Eigen::MatrixXd matStateSigmaPoints = generateStateSigmaPoints(L);

		// propagate sigma points through measurement model
		Eigen::MatrixXd matMeasurementSigmaPoints(3 * n, 2 * L + 1);
		// iterate over measurements
		for (int ii = 0; ii < n; ii++)
		{
			int currentMarkerCursor = -1;
			auto search = staticMarkerMap_.find(measurementIn.getElement(0, ii));
			if (search != staticMarkerMap_.end())
			{
				for (int jj = 0; jj < 2 * L + 1; jj++)
				{
					matMeasurementSigmaPoints(3 * ii + 0, jj) = search->second[0] * std::cos(matStateSigmaPoints(2, jj))
						+ search->second[1] * std::sin(matStateSigmaPoints(2, jj))
						- matStateSigmaPoints(0, jj) * std::cos(matStateSigmaPoints(2, jj))
						- matStateSigmaPoints(1, jj) * std::sin(matStateSigmaPoints(2, jj));
					matMeasurementSigmaPoints(3 * ii + 1, jj) = -search->second[0] * std::sin(matStateSigmaPoints(2, jj))
						+ search->second[1] * std::cos(matStateSigmaPoints(2, jj))
						+ matStateSigmaPoints(0, jj) * std::sin(matStateSigmaPoints(2, jj))
						- matStateSigmaPoints(1, jj) * std::cos(matStateSigmaPoints(2, jj));
					matMeasurementSigmaPoints(3 * ii + 2, jj) = search->second[2] - matStateSigmaPoints(2, jj);
					matMeasurementSigmaPoints(3 * ii + 2, jj) = wrapAngle(matMeasurementSigmaPoints(3 * ii + 2, jj));
				}
			}
			else
			{
				for (int jj = 0; jj < markerIndices_.size(); jj++)
				{
					if (markerIndices_.at(jj) == measurementIn.getElement(0, ii))
					{
						currentMarkerCursor = 3 * jj + 3;
					}
				}

				if (currentMarkerCursor == -1)
				{
					addMarker(measurementIn.getElement(0, ii)
						, measurementIn.getElement(1, ii)
						, measurementIn.getElement(2, ii)
						, measurementIn.getElement(3, ii));
					return;
				}

				for (int jj = 0; jj <  2 * L + 1; jj++)
				{
					matMeasurementSigmaPoints(3 * ii + 0, jj) = matStateSigmaPoints(currentMarkerCursor, jj) * std::cos(matStateSigmaPoints(2, jj))
						+ matStateSigmaPoints(currentMarkerCursor + 1, jj) * std::sin(matStateSigmaPoints(2, jj))
						- matStateSigmaPoints(0, jj) * std::cos(matStateSigmaPoints(2, jj))
						- matStateSigmaPoints(1, jj) * std::sin(matStateSigmaPoints(2, jj));
					matMeasurementSigmaPoints(3 * ii + 1, jj) = -matStateSigmaPoints(currentMarkerCursor, jj) * std::sin(matStateSigmaPoints(2, jj))
						+ matStateSigmaPoints(currentMarkerCursor + 1, jj) * std::cos(matStateSigmaPoints(2, jj))
						+ matStateSigmaPoints(0, jj) * std::sin(matStateSigmaPoints(2, jj))
						- matStateSigmaPoints(1, jj) * std::cos(matStateSigmaPoints(2, jj));
					matMeasurementSigmaPoints(3 * ii + 2, jj) = matStateSigmaPoints(currentMarkerCursor + 2, jj) - matStateSigmaPoints(2, jj);
					matMeasurementSigmaPoints(3 * ii + 2, jj) = wrapAngle(matMeasurementSigmaPoints(3 * ii + 2, jj));
				}
			}

			// TODO: replace loops with tmp var and add single loop here
		}

		// calc measurement prediction mean
		Eigen::VectorXd vecH;
		vecH.setZero(3 * n);
		for (int ii = 0; ii < 2 * L + 1; ii++)
		{
			vecH += sigmaPointsWeightState_[ii] * matMeasurementSigmaPoints.col(ii);
		}
		VSLibMarkerLocalization::Tools::stateWrapAngle(vecH);

		// calc autocorrelation and crosscorrelation
		Eigen::MatrixXd matAutoCorr, matCrossCorr;
		matAutoCorr.setZero(vecH.size(), vecH.size());
		matCrossCorr.setZero(vecMu_.size(), vecH.size());

		for (int ii = 0; ii < 2 * L + 1; ii++)
		{
			Eigen::VectorXd vecDiffMeasurement = matMeasurementSigmaPoints.col(ii) - vecH;
			Eigen::VectorXd vecDiffState = matStateSigmaPoints.col(ii) - vecMu_;

			VSLibMarkerLocalization::Tools::stateWrapAngle(vecDiffMeasurement);
			VSLibMarkerLocalization::Tools::stateWrapAngle(vecDiffState);

			matAutoCorr += sigmaPointsWeightCovariance_[ii] * (vecDiffMeasurement * vecDiffMeasurement.transpose());
			matCrossCorr += sigmaPointsWeightCovariance_[ii] * (vecDiffState * vecDiffMeasurement.transpose());
		}
		Eigen::MatrixXd matRResized = Eigen::MatrixXd::Zero(vecH.size(), vecH.size());
		for (int ii = 0; ii < n; ii++)
		{
			matRResized.block<3, 3>(3 * ii, 3 * ii) = matR_;
		}
		matAutoCorr += matRResized;

		// calc kalman gain
		matK_.setZero(vecMu_.size(), vecH.size());
		matK_ = matCrossCorr * matAutoCorr.inverse();

		// calculate state from kalman gain
		Eigen::VectorXd vecSenseDiff = vecZ - vecH;
		VSLibMarkerLocalization::Tools::stateWrapAngle(vecSenseDiff);
		vecMu_ = vecMu_ + matK_ * vecSenseDiff;
		VSLibMarkerLocalization::Tools::stateWrapAngle(vecMu_);

		// calculate sigma from kalman gain
		matSigma_ = matSigma_ - matK_ * matAutoCorr * matK_.transpose();
	}

	void UKF2DInstance::addMarker(const int& markerID, const double& markerX, const double& markerY, const double& markerTheta)
	{
		auto search = staticMarkerMap_.find(markerID);
		if (search != staticMarkerMap_.end())
			return;

		markerIndices_.push_back(markerID);
		numMarkerInUKFMap_++;

		// 2D Trafo
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

		// update weight vectors
		setUKFParam(alpha_,beta_,kappa_);
	}

	void UKF2DInstance::setStaticMap(std::unordered_map<int, VSM::Vector3> inStaticMap)
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
				numMarkerInUKFMap_--;


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

	void UKF2DInstance::resetStaticMap()
	{
		staticMarkerMap_.clear();
	}

	void UKF2DInstance::setUKFParam(const double& alpha, const double& beta, const double& kappa)
	{
		// update ukf parameters
		alpha_ = alpha;
		beta_ = beta;
		kappa_ = kappa;

		const int L = vecMu_.size();

		varc_ = std::pow(alpha_, 2) * (L + kappa);

		// update weight vectors
		sigmaPointsWeightState_.clear();
		sigmaPointsWeightCovariance_.clear();

		sigmaPointsWeightState_.push_back(1 - L / varc_);
		sigmaPointsWeightCovariance_.push_back((2 - std::pow(alpha_, 2) + beta_) - L / varc_);

		double weight = 1 / (2 * varc_);
		for (int ii = 1; ii < 2 * L + 1; ii++)
		{
			sigmaPointsWeightState_.push_back(weight);
			sigmaPointsWeightCovariance_.push_back(weight);
		}
	}
}//end namespace