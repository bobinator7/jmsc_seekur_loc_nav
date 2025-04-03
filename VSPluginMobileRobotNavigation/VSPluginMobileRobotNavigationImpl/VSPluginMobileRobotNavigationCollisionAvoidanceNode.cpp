// header
#include "../VSPluginMobileRobotNavigationCollisionAvoidanceNode.h"

// other
#include "../VSPluginMobileRobotNavigationProject.h"

#include "Lib/VSD/VSDConnection.h"
#include "Lib/VSDIO/VSDIOInputDouble.h"
#include "Lib/VSDIO/VSDIOInputDoubleVector.h"
#include "Lib/VSDIO/VSDIOInputVSMVector3.h"
#include "Lib/VSDIO/VSDIOOutputDouble.h"
#include "Lib/VSDIO/VSDIOOutputQImage.h"
#include "Lib/VSS/VSSScheduler.h"

#include <QtConcurrent/qtconcurrentrun.h>
#include <chrono>

VSPluginMobileRobotNavigation::CollisionAvoidanceNode::CollisionAvoidanceNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataBaseNode(this)
, dataBoolEnableVisualization(this)
, dataBoolUseMapPoints(this)
, dataBoolUseRealValueForControl(this)
, dataBoolUseRealValueForMapping(this)
, dataBoolUseSimTime(this)
, dataDoubleAlpha(this)
, dataDoubleBeta(this)
, dataDoubleGamma(this)
, dataDoubleSigma(this)
, dataLaserExtension(this)
, dataRotAccelerationRange(this)
, dataRotVelocityRange(this)
, dataTransAccelerationRange(this)
, dataTransVelocityRange(this)
, dataUpdateControlRate(this)
, dataUpdateMapRate(this)
, mapThreadBusy_(false)
, mapThreadWatcher(0)
, visu_(0)
{
	timer_ = new QTimer();
}

VSPluginMobileRobotNavigation::CollisionAvoidanceNode::~CollisionAvoidanceNode()
{
	if (visu_)
		delete visu_;
}

VSDIO::ExtensionIOBoard* VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getIOBoard()
{
   if (ioBoard == 0)
   {
      ioBoard = findFirstExtension<VSDIO::ExtensionIOBoard*>();
      if (ioBoard == 0)
      {
         ioBoard = getMySimState()->newSimStateInstance<VSDIO::ExtensionIOBoard>();
         this->getExtensions().append(ioBoard);
      }
   }
   return ioBoard;
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::postHasBeenAddedToElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   ioBoard = getIOBoard();
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;

	// abort if no ioBoard
	if (!ioBoard)
		return;

	// property modified functions
	connect(this
		, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*))
		, this
		, SLOT(slotPropertiesModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));

	// inputs
	VSDIO::Input* input;

	input = ioBoard->findInputByName("execute");
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotEnableModified(VSDIO::Input*)));

   input = ioBoard->findInputByName("robot_pose");
   if (!input)
   {
      input = ioBoard->createInput<VSDIO::InputVSMVector3>("robot_pose");
   }
   connect(input
         , SIGNAL(signalInputValueModified(VSDIO::Input*))
         , this
         , SLOT(slotPoseValuesModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("goal_pose");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputVSMVector3>("goal_pose");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotPoseValuesModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("v_rIn");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputDouble>("v_rIn");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotMovementValueModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("w_rIn");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputDouble>("w_rIn");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotMovementValueModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("laser_depths");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputDoubleVector>("laser_depths");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotLaserscanValueModified(VSDIO::Input*)));

	// outputs
	VSDIO::Output* output;
   output = ioBoard->findOutputByName("v_rOut");
   if (!output)
   {
      output = ioBoard->createOutput<VSDIO::OutputDouble>("v_rOut");
   }

	output = ioBoard->findOutputByName("w_rOut");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputDouble>("w_rOut");
	}

	output = ioBoard->findOutputByName("occmap");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputQImage>("occmap");
	}

	output = ioBoard->findOutputByName("costmap");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputQImage>("costmap");
	}

	// calculate sensor to base transform
	setLaserscanConversionData();

	// set initial map
	imgHeight_ = 400;
	imgWidth_ = 400;
	setStaticGridmap();

	// initialize timer
	VSD::Connection::connect(timer_, SIGNAL(timeout()), this, SLOT(slotUpdateLoop()));
	timer_->start(100);

	// set initial properties
	if (getRotAccelerationRange() == VSM::Vector2())
	{
		setRotAccelerationRange(VSM::Vector2(-10, 10));
	}
	if (getRotVelocityRange() == VSM::Vector2())
	{
		setRotVelocityRange(VSM::Vector2(-0.25, 0.25));
	}
	if (getTransAccelerationRange() == VSM::Vector2())
	{
		setTransAccelerationRange(VSM::Vector2(-0.75, 0.75));
	}
	if (getTransVelocityRange() == VSM::Vector2())
	{
		setTransVelocityRange(VSM::Vector2(0, 1.5));
	}
	
	// set default time
	lastUpdateControlTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
	lastUpdateMapTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());

	// initialize CA class
	collavInstance_ = VSLibMobileRobotNavigation::CollavDWA();
	collavInstance_.setRobotProperties(0.42,0.6);
	collavInstance_.setRobotVelocityBounds(
		getTransVelocityRange().getX(), 
		getTransVelocityRange().getY(), 
		getRotVelocityRange().getX(), 
		getRotVelocityRange().getY());
	collavInstance_.setRobotAccelerationBounds(
		getTransAccelerationRange().getX(), 
		getTransAccelerationRange().getY(), 
		getRotAccelerationRange().getX(), 
		getRotAccelerationRange().getY());
	collavInstance_.setObjectiveFunctionParam(
		getDoubleSigma(),
		getDoubleAlpha(),
		getDoubleBeta(),
		getDoubleGamma());

	collavInstance_.setGridmap(occmap_);

	// init visu
	setBoolEnableVisualization(false);
	if (getBoolEnableVisualization())
	{
		visu_ = new CollavVisu(getMySimState()->getMyEnvironment()->getMyProject(), this);
		visu_->initialize();
	}

	// initialize watcher
	mapThreadWatcher = new QFutureWatcher<void>();
	VSD::Connection::connect(mapThreadWatcher, SIGNAL(finished()), this, SLOT(slotMapThreadFinished()));

}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

	timer_->stop();
	VSD::Connection::disconnect(timer_, SIGNAL(timeout()), this, SLOT(slotUpdateLoop()));

	// destruct watcher
	if (mapThreadWatcher->isRunning())
		mapThreadWatcher->waitForFinished();
	VSD::Connection::disconnect(mapThreadWatcher, SIGNAL(finished()), this, SLOT(slotMapThreadFinished()));
	delete mapThreadWatcher;
	mapThreadWatcher = 0;
}

// note: edit here to process data on input change
void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
	if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
		return;

	VSPluginMobileRobotNavigation::CollisionAvoidanceNode* thisInstance = simStateInstance->instanceCast<VSPluginMobileRobotNavigation::CollisionAvoidanceNode*>();
	if (thisInstance)
	{
		const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
		VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
		if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("baseNode"))
		{
			setLaserscanConversionData();
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("boolEnableVisualization"))
		{
			if (getBoolEnableVisualization())
			{
				visu_ = new CollavVisu(getMySimState()->getMyEnvironment()->getMyProject(), this);
				visu_->initialize();
			}
			else
			{
				delete visu_;
				visu_ = 0;
			}
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("laserExtension"))
		{
			setLaserscanConversionData();
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("transVelocityRange"))
		{
			collavInstance_.setRobotVelocityBounds(
				getTransVelocityRange().getX(),
				getTransVelocityRange().getY(),
				getRotVelocityRange().getX(),
				getRotVelocityRange().getY());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("rotVelocityRange"))
		{
			collavInstance_.setRobotVelocityBounds(
				getTransVelocityRange().getX(),
				getTransVelocityRange().getY(),
				getRotVelocityRange().getX(),
				getRotVelocityRange().getY());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("transAccelerationRange"))
		{
			collavInstance_.setRobotAccelerationBounds(
				getTransAccelerationRange().getX(),
				getTransAccelerationRange().getY(),
				getRotAccelerationRange().getX(),
				getRotAccelerationRange().getY());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("rotAccelerationRange"))
		{
			collavInstance_.setRobotAccelerationBounds(
				getTransAccelerationRange().getX(),
				getTransAccelerationRange().getY(),
				getRotAccelerationRange().getX(),
				getRotAccelerationRange().getY());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("doubleAlpha"))
		{
			collavInstance_.setObjectiveFunctionParam(
				getDoubleSigma(),
				getDoubleAlpha(),
				getDoubleBeta(),
				getDoubleGamma());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("doubleBeta"))
		{
			collavInstance_.setObjectiveFunctionParam(
				getDoubleSigma(),
				getDoubleAlpha(),
				getDoubleBeta(),
				getDoubleGamma());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("doubleGamma"))
		{
			collavInstance_.setObjectiveFunctionParam(
				getDoubleSigma(),
				getDoubleAlpha(),
				getDoubleBeta(),
				getDoubleGamma());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("doubleSigma"))
		{
			collavInstance_.setObjectiveFunctionParam(
				getDoubleSigma(),
				getDoubleAlpha(),
				getDoubleBeta(),
				getDoubleGamma());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("boolUseSimTime"))
		{
			if (getBoolUseSimTime())
			{
				lastUpdateControlTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
				lastUpdateMapTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
			}
			else
			{
				lastUpdateControlTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
				lastUpdateMapTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
			}
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("boolUseMapPoints"))
		{
			collavInstance_.setUseMapPointsFlag(getBoolUseMapPoints());
		}
	}
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotEnableModified(VSDIO::Input* input)
{
	if (!getInputValueExecute())
	{
		// reset map
		if (mapThreadWatcher)
			mapThreadWatcher->waitForFinished();
		occmap_.resetEmptyMap();
		mapThreadBusy_ = false;


		// reset movement calculation
		std::pair<double, double> outVar = std::make_pair(0.0, 0.0);

		// set output
		if (ioBoard) {
			VSDIO::Output* output = ioBoard->findOutputByName("v_rOut");
			if (output)
			{
				QVariant outputVariant = QVariant::fromValue(outVar.first);
				output->setLocalVariantAndForceNotify(outputVariant);
			}

			output = ioBoard->findOutputByName("w_rOut");
			if (output)
			{
				QVariant outputVariant = QVariant::fromValue(outVar.second);
				output->setLocalVariantAndForceNotify(outputVariant);
			}
		}
	}
	else
	{
		// time reset
		// TODO: use wall time if not in sim
		if (getBoolUseSimTime())
		{
			lastUpdateControlTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
			lastUpdateMapTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
		}
		else
		{
			lastUpdateControlTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
			lastUpdateMapTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		}

		/// debug
		setLaserscanConversionData();

	}
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotUpdateLoop()
{
	if (!getInputValueExecute())
		return;

	// get time period since last update
	bool updateControl = false;
	bool updateMap = false;
	double timePeriodControl, timePeriodMap;
	if (getBoolUseSimTime())
	{
		timePeriodControl = VSS::Scheduler::getCurrentTaskTime(getMySimState()) - lastUpdateControlTime_;
		timePeriodMap = VSS::Scheduler::getCurrentTaskTime(getMySimState()) - lastUpdateMapTime_;

		if (timePeriodControl >= 1 / getUpdateControlRate())
		{
			updateControl = true;
			lastUpdateControlTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
		}

		if (timePeriodMap >= 1 / getUpdateMapRate())
		{
			updateMap = true;
			lastUpdateMapTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
		}
	}
	else
	{
		// TODO use wall time
		double now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		timePeriodControl = now - lastUpdateControlTime_;
		timePeriodMap = now - lastUpdateMapTime_;

		if (timePeriodControl >= 1 / getUpdateControlRate())
		{
			updateControl = true;
			lastUpdateControlTime_ = now;
		}

		if (timePeriodMap >= 1 / getUpdateMapRate())
		{
			updateMap = true;
			lastUpdateMapTime_ = now;
		}
	}

	// trigger movement calculation
	if (updateControl)
	{
		/// debug
		VSM::Vector3 robotPose = getBaseNode()->getWorldFrame().getPosition(); 
		robotPose.setZ(getBaseNode()->getWorldFrame().getOrientation().getRoll());

		std::pair<double, double> outVar;
		if (getBoolUseRealValueForControl())
			outVar = collavInstance_.calculateMovementControl(robotPose, goalPose_, velTrans_, velRot_, 1 / getUpdateControlRate(), laserscanInRobotFrame_);
		else
			outVar = collavInstance_.calculateMovementControl(robotPose_, goalPose_, velTrans_, velRot_, 1 / getUpdateControlRate(), laserscanInRobotFrame_);

		lastControl_ = outVar;
		expectedPose_ = collavInstance_.calcFuturePose(lastControl_);
		// set output
		if (ioBoard) {
			VSDIO::Output* output = ioBoard->findOutputByName("v_rOut");
			if (output)
			{
				QVariant outputVariant = QVariant::fromValue(outVar.first);
				output->setLocalVariantAndForceNotify(outputVariant);
			}

			output = ioBoard->findOutputByName("w_rOut");
			if (output)
			{
				QVariant outputVariant = QVariant::fromValue(outVar.second);
				output->setLocalVariantAndForceNotify(outputVariant);
			}
		}
	}

	if (updateMap)
	{
		if (!mapThreadBusy_)
		{
			if (mapThreadWatcher)
			{
				mapThreadBusy_ = true;
				mapThreadWatcher->setFuture(QtConcurrent::run(this, &VSPluginMobileRobotNavigation::CollisionAvoidanceNode::calcMapInThread));
			}
		}
	}
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotPoseValuesModified(VSDIO::Input* input)
{
	// check if node is active
	if (!getInputValueExecute())
		return;

	// test if property values are active
	//if (!propertyCheck())
	//	return;

	// get pose values
	VSM::Vector3 robotPose, goalPose;
	VSDIO::Input* inputVel = ioBoard->findInputByName("robot_pose");
	if (!inputVel->inherits<VSDIO::InputVSMVector3*>())
		return;
	QVariant variantData = inputVel->instanceCast<VSDIO::InputVSMVector3*>()->getVariant();
	robotPose = variantData.value<VSM::Vector3>();

	VSDIO::Input* inputRot = ioBoard->findInputByName("goal_pose");
	if (!inputRot->inherits<VSDIO::InputVSMVector3*>())
		return;
	variantData = inputRot->instanceCast<VSDIO::InputVSMVector3*>()->getVariant();
	goalPose = variantData.value<VSM::Vector3>();

	// update current poses
	lastRobotPose_ = robotPose_;
	robotPose_ = robotPose;
	goalPose_ = goalPose;

	return;
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotMovementValueModified(VSDIO::Input* input)
{
	// check if node is active
	if (!getInputValueExecute())
		return;

	// test if property values are active
	//if (!propertyCheck())
	//	return;

	// get movement values
	double robotVel, robotRot;
	VSDIO::Input* inputVel = ioBoard->findInputByName("v_rIn");
	if (!inputVel->inherits<VSDIO::InputDouble*>())
		return;
	QVariant variantData = inputVel->instanceCast<VSDIO::InputDouble*>()->getVariant();
	robotVel = variantData.value<double>();

	VSDIO::Input* inputRot = ioBoard->findInputByName("w_rIn");
	if (!inputRot->inherits<VSDIO::InputDouble*>())
		return;
	variantData = inputRot->instanceCast<VSDIO::InputDouble*>()->getVariant();
	robotRot = variantData.value<double>();
	
	// update current velocities
	velTrans_ = robotVel;
	velRot_ = robotRot;
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotLaserscanValueModified(VSDIO::Input* input)
{
	// check if node is active
	if (!getInputValueExecute())
		return;

	// test if property values are active
	//if (!propertyCheck())
	//	return;

	if (!input->inherits<VSDIO::InputDoubleVector*>())
		return;
	QVariant variantData = input->instanceCast<VSDIO::InputDoubleVector*>()->getVariant();
	QVector<double> laserscanVector = variantData.value<QVector<double>>();

	// convert into points in robot frame
	if (!mapThreadBusy_)
	{
		laserscanRaw_ = laserscanVector.toStdVector();
	}
	laserscanInRobotFrame_ = laserscanInRobotFrameFromRange(laserscanVector.toStdVector());
}

bool VSPluginMobileRobotNavigation::CollisionAvoidanceNode::propertyCheck()
{
	return true;
}

std::vector<std::pair<double, double>> VSPluginMobileRobotNavigation::CollisionAvoidanceNode::laserscanInRobotFrameFromRange(std::vector<double> rangeData)
{
	// TODO: move to tools
	if (rangeData.size() > indexRange_)
		return std::vector<std::pair<double, double>>();

	std::vector<std::pair<double, double>> laserPointCloud, tmp;
	for (int ii = 0; ii < rangeData.size(); ii++)
	{
		if (rangeData[ii] < 0.005)
			continue;

		// range to point
		double x = rangeData[ii] * std::sin(indexAngle_[ii]);
		double y = rangeData[ii] * std::cos(indexAngle_[ii]);

		// sensor frame to base frame
		VSM::Vector3 pointInSensorFrame(x, y, 0);
		VSM::Vector3 pointInBaseFrame = sensorToBaseTransform_.trafo(pointInSensorFrame);

		tmp.push_back(std::make_pair(pointInSensorFrame.getX(), pointInSensorFrame.getY()));
		laserPointCloud.push_back(std::make_pair(pointInBaseFrame.getX(), pointInBaseFrame.getY()));
	}

	return laserPointCloud;
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::setLaserscanConversionData()
{
	if (!getBaseNode() || !getLaserExtension())
		return;

	// calculate transform from laserscanner to base 
	VSM::Frame f1 = getBaseNode()->getWorldFrame();
	VSM::Frame f2 = getLaserExtension()->getParentVSD3DNode()->getWorldFrame();
	sensorToBaseTransform_ = getBaseNode()->getWorldFrame().getInverse() * getLaserExtension()->getParentVSD3DNode()->getWorldFrame();

	// calculate range of indices from laser range data
	indexRange_ = std::ceil(getLaserExtension()->getAngle() / getLaserExtension()->getResolution()) + 20;

	// precalculate angle of each index (for performance)
	indexAngle_.clear();
	for (int ii = 0; ii < indexRange_; ii++)
	{
		indexAngle_.push_back(getLaserExtension()->getAngleOffset() + ii * getLaserExtension()->getResolution());
	}

	// set map parameters
	occmap_.setRangesConversionParam(getLaserExtension()->getAngleOffset(), getLaserExtension()->getResolution(), sensorToBaseTransform_);

}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::setStaticGridmap()
{
	// TODO: remove hardcoded resolution and offset
	std::pair<double, double> offset;
	offset.first = -20.0;
	offset.second = -20.0;
	occmap_.importMap(imgWidth_, imgHeight_, 0.1, offset, std::vector<unsigned char>(imgHeight_ * imgWidth_, 0));
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::calcMapInThread()
{
	/// debug
	VSM::Vector3 robotPose = getBaseNode()->getWorldFrame().getPosition();
	robotPose.setZ(getBaseNode()->getWorldFrame().getOrientation().getRoll());

	if (getBoolUseRealValueForMapping())
		occmap_.updateMapFromScan(laserscanRaw_, robotPose);
	else
		occmap_.updateMapFromScan(laserscanRaw_, robotPose_);
}

void VSPluginMobileRobotNavigation::CollisionAvoidanceNode::slotMapThreadFinished()
{
	mapThreadBusy_ = false;

	QImage outMap = occmap_.exportMap();
	QImage outCostmap = occmap_.exportCostmap();
	if (ioBoard)
	{
		VSDIO::Output* output = ioBoard->findOutputByName("occmap");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(outMap);
			output->setLocalVariantAndForceNotify(outputVariant);
		}

		output = ioBoard->findOutputByName("costmap");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(outCostmap);
			output->setLocalVariantAndForceNotify(outputVariant);
		}
	}
}

VSM::Vector3 VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getRobotPose()
{
	return collavInstance_.getRobotPose();
}

std::vector<std::pair<double, double>> VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getCollavPointCloud()
{
	return collavInstance_.getCollisionPointCloud();
}

std::vector<std::pair<double, double>> VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getCollisionPoints()
{
	return collavInstance_.getCollisionPoints();
}

std::vector<std::pair<double, double>> VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getIntersectionPoints()
{
   return collavInstance_.getIntersectionPoints();
}

std::vector<double> VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getVecRadius()
{
   return collavInstance_.getVecRadius();
}

std::vector<std::pair<double, double>> VSPluginMobileRobotNavigation::CollisionAvoidanceNode::getLaserPointCloud()
{
	return laserscanInRobotFrame_;
}