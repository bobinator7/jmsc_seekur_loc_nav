// header
#include "../VSPluginMobileRobotNavigationSecurityFilterNode.h"

// other
#include "../VSPluginMobileRobotNavigationProject.h"

#include "Lib/VSD/VSDConnection.h"
#include "Lib/VSDIO/VSDIOInputDouble.h"
#include "Lib/VSDIO/VSDIOInputDoubleVector.h"
#include "Lib/VSDIO/VSDIOOutputDouble.h"

#include <QtConcurrent/qtconcurrentrun.h>

VSPluginMobileRobotNavigation::SecurityFilterNode::SecurityFilterNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, busy_(false)
, dataBaseNode(this)
, dataLaserExtension(this)
, dataSafetyLength(this)
, dataSafetyWidth(this)
, resultWatcher(0)
, valid_(false)
{
}

VSPluginMobileRobotNavigation::SecurityFilterNode::~SecurityFilterNode()
{
}

VSDIO::ExtensionIOBoard* VSPluginMobileRobotNavigation::SecurityFilterNode::getIOBoard()
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

void VSPluginMobileRobotNavigation::SecurityFilterNode::postHasBeenAddedToElementContainer()
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

	// calculate sensor to base transform
	setLaserscanConversionData();

	// set initial rectengular safety bound
	dataSafetyWidth.set(0.62);
	dataSafetyLength.set(1.13);

	resultWatcher = new QFutureWatcher<void>();
	VSD::Connection::connect(resultWatcher, SIGNAL(finished()), this, SLOT(slotCheckValidityFinished()));
}

void VSPluginMobileRobotNavigation::SecurityFilterNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

	VSD::Connection::disconnect(resultWatcher, SIGNAL(finished()), this, SLOT(slotCheckValidityFinished()));
	if (resultWatcher->isRunning())
		resultWatcher->waitForFinished();
	delete resultWatcher;
	resultWatcher = 0;
}

// note: edit here to process data on input change
void VSPluginMobileRobotNavigation::SecurityFilterNode::slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
	if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
		return;

	VSPluginMobileRobotNavigation::SecurityFilterNode* thisInstance = simStateInstance->instanceCast<VSPluginMobileRobotNavigation::SecurityFilterNode*>();
	if (thisInstance)
	{
		const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
		VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
		if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("baseNode"))
		{
			setLaserscanConversionData();
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("laserExtension"))
		{
			setLaserscanConversionData();
		}
	}
}

void VSPluginMobileRobotNavigation::SecurityFilterNode::slotEnableModified(VSDIO::Input* input)
{
	if (!getInputValueExecute())
	{
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
		setLaserscanConversionData();
	}
}

void VSPluginMobileRobotNavigation::SecurityFilterNode::slotLaserscanValueModified(VSDIO::Input* input)
{
	// check if node is active
	if (!getInputValueExecute())
		return;

	if (busy_)
		return;

	// test if property values are active
	//if (!propertyCheck())
	//	return;

	if (!input->inherits<VSDIO::InputDoubleVector*>())
		return;
	QVariant variantData = input->instanceCast<VSDIO::InputDoubleVector*>()->getVariant();
	QVector<double> laserscanVector = variantData.value<QVector<double>>();

	if (!resultWatcher)
		return;
	busy_ = true;

	resultWatcher->setFuture(QtConcurrent::run(this, &VSPluginMobileRobotNavigation::SecurityFilterNode::checkValidInThread, laserscanVector.toStdVector()));

}

void VSPluginMobileRobotNavigation::SecurityFilterNode::slotCheckValidityFinished()
{
	busy_ = false;

	// set output if 
	std::pair<double, double> outVar = std::make_pair(0.0, 0.0);
	if (valid_)
	{
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

		outVar.first = robotVel;
		outVar.second = robotRot;
	}

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

void VSPluginMobileRobotNavigation::SecurityFilterNode::checkValidInThread(std::vector<double> rangeVector)
{
	valid_ = validLaserscanInRobotFrameFromRange(rangeVector);
}

bool VSPluginMobileRobotNavigation::SecurityFilterNode::propertyCheck()
{
	return true;
}

bool VSPluginMobileRobotNavigation::SecurityFilterNode::validLaserscanInRobotFrameFromRange(std::vector<double> rangeData)
{
	// TODO: move to tools
	if (rangeData.size() != indexRange_)
		return false;

	double halfWidth = getSafetyWidth() / 2;
	double halfLength = getSafetyLength() / 2;

	for (int ii = 0; ii < indexRange_; ii++)
	{
		// range to point
		double x = rangeData[ii] * std::sin(indexAngle_[ii]);
		double y = rangeData[ii] * std::cos(indexAngle_[ii]);

		// sensor frame to base frame
		VSM::Vector3 pointInSensorFrame(x, y, 0);
		VSM::Vector3 pointInBaseFrame = sensorToBaseTransform_.trafo(pointInSensorFrame);

		if (std::fabs(pointInBaseFrame.x) < halfWidth && std::fabs(pointInBaseFrame.y) < halfLength)
			return false;

	}

	return true;
}

void VSPluginMobileRobotNavigation::SecurityFilterNode::setLaserscanConversionData()
{
	if (!getBaseNode() || !getLaserExtension())
		return;

	// calculate transform from laserscanner to base 
	sensorToBaseTransform_ = getBaseNode()->getWorldFrame().getInverse() * getLaserExtension()->getParentVSD3DNode()->getWorldFrame();

	// calculate range of indices from laser range data
	indexRange_ = std::floor(getLaserExtension()->getAngle() / getLaserExtension()->getResolution()) - 1;

	// precalculate angle of each index (for performance)
	indexAngle_.clear();
	for (int ii = 0; ii < indexRange_; ii++)
	{
		indexAngle_.push_back(getLaserExtension()->getAngleOffset() + ii * getLaserExtension()->getResolution());
	}
}