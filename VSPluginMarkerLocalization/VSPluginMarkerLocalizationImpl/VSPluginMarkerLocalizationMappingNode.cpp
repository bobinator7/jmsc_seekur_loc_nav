// header
#include "../VSPluginMarkerLocalizationMappingNode.h"

// other
#include "../VSPluginMarkerLocalizationProject.h"

#include "Lib/VSDIO/VSDIOInputDoubleVector.h"
#include "Lib/VSDIO/VSDIOInputVSMVector3.h"
#include "Lib/VSDIO/VSDIOOutputQImage.h"
#include "Lib/VSDTools/VSDTools.h"
#include "Lib/VSS/VSSScheduler.h"


#include <qfile.h>

VSPluginMarkerLocalization::MappingNode::MappingNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataBaseNode(this)
, dataBoolEnableVisualization(this)
, dataBoolUseSimTime(this)
, dataLaserExtension(this)
, dataRobotPose(this)
, dataStaticMarkerMapFile(this)
, dataUpdateRate(this)
{

}

VSPluginMarkerLocalization::MappingNode::~MappingNode()
{

}

VSDIO::ExtensionIOBoard* VSPluginMarkerLocalization::MappingNode::getIOBoard()
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

// note: edit here to modify board on construction
void VSPluginMarkerLocalization::MappingNode::postHasBeenAddedToElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   ioBoard = getIOBoard();
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;

	// property modified functions
	connect(this
		, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*))
		, this
		, SLOT(slotPropertiesModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));

	if (!ioBoard)
		return;

	// input
	VSDIO::Input* input;

	input = ioBoard->findInputByName("execute");
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotEnableModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("robot pose");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputVSMVector3>("robot pose");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotInputModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("laser_depths");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputDoubleVector>("laser_depths");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotLaserscanValueModified(VSDIO::Input*)));

	// output
	VSDIO::Output* output = ioBoard->findOutputByName("occupancy gridmap");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputQImage>("occupancy gridmap");
	}

	output = ioBoard->findOutputByName("costmap");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputQImage>("costmap");
	}

	// set extension param
	setLaserscanConversionData();

	// set initial map
	if (!loadImageFromFile())
	{
		imgHeight_ = 400;
		imgWidth_ = 400;
		imgRawData_ = std::vector<unsigned char>(imgHeight_ * imgWidth_, 0);
	}
	setStaticGridmap();

	// set time param
	dataBoolUseSimTime.set(true);
	lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
	dataUpdateRate.set(2.0);
}

void VSPluginMarkerLocalization::MappingNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();
}

// note: edit here to process data on input change
void VSPluginMarkerLocalization::MappingNode::slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
	if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
		return;

	VSPluginMarkerLocalization::MappingNode* thisInstance = simStateInstance->instanceCast<VSPluginMarkerLocalization::MappingNode*>();
	if (thisInstance)
	{
		const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
		VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
		if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("staticMarkerMapFile"))
		{
			if (!loadImageFromFile())
			{
				imgHeight_ = 400;
				imgWidth_ = 400;
				imgRawData_ = std::vector<unsigned char>(imgHeight_ * imgWidth_, 0);
			}
			setStaticGridmap();
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("laserExtension"))
		{
			setLaserscanConversionData();
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("baseNode"))
		{
			setLaserscanConversionData();
		}

	}
}

void VSPluginMarkerLocalization::MappingNode::slotInputModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
		return;

}

void VSPluginMarkerLocalization::MappingNode::slotLaserscanValueModified(VSDIO::Input* input)
{
   // get value of the bool input "execute" defined by base class
   if (!getInputValueExecute())
      return;

	// get time period since last update
	double timePeriod;
	if (getDataBoolUseSimTime())
	{
		timePeriod = VSS::Scheduler::getCurrentTaskTime(getMySimState()) - lastUpdateTime_;
		if (timePeriod < 1 / getUpdateRate())
			return;
		lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
	}
	else
	{
		// TODO use wall time
		timePeriod = 1 / getUpdateRate();
	}


	// get input
	if (!input->inherits<VSDIO::InputDoubleVector*>())
		return;
	QVariant variantData = input->instanceCast<VSDIO::InputDoubleVector*>()->getVariant();
	QVector<double> laserscanVector = variantData.value<QVector<double>>();

	VSM::Vector3 robotPose;
	VSDIO::Input* inputVel = ioBoard->findInputByName("robot pose");
	if (!inputVel->inherits<VSDIO::InputVSMVector3*>())
		return;
	variantData = inputVel->instanceCast<VSDIO::InputVSMVector3*>()->getVariant();
	robotPose = variantData.value<VSM::Vector3>();

	// update map
	staticGridmap_.updateMapFromScan(laserscanVector.toStdVector(), robotPose);
	staticGridmap_.generateCostMap(robotPose, VSM::Vector3(0.0, 10.0, 0.0));

	// set output
	QImage outMap = staticGridmap_.exportMap();
	QImage outCostmap = staticGridmap_.exportCostmap();
	if (ioBoard)
	{
		VSDIO::Output* output = ioBoard->findOutputByName("occupancy gridmap");
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

void VSPluginMarkerLocalization::MappingNode::slotEnableModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
	{
		if (imgRawData_.empty())
		{
			staticGridmap_.resetEmptyMap();
		}
		else
		{
			setStaticGridmap();
		}
		
	}
	else
	{
		// time reset
		// TODO: use wall time if not in sim
		if (getBoolUseSimTime())
			lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
		//else
		//	lastUpdateTime_ = 
	}
}

// note: sanity check
bool VSPluginMarkerLocalization::MappingNode::propertyCheck()
{
   if(getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return false;

   return true;
}

bool VSPluginMarkerLocalization::MappingNode::loadImageFromFile()
{
	// open file and create a stream
	if (getStaticMarkerMapFile().isEmpty())
		return false;
	QString filename = VSDTools::compileAbsoluteUrl(getMyModel(), getStaticMarkerMapFile()).toLocalFile();
	QFile configFile(filename);
	if (!configFile.exists())
		return false;
	configFile.open(QFile::ReadOnly);
	QTextStream stream(&configFile);

	// check if file is valid pgm image
	QString magicNumber = stream.readLine();
	if (magicNumber != "P5")
		return false;

	QString widthAndHeight = stream.readLine();
	imgWidth_ = widthAndHeight.split(" ")[0].toInt();
	imgHeight_ = widthAndHeight.split(" ")[1].toInt();

	imgMaxGreyValue_ = stream.readLine().toInt();

	// clear stream and data vector
	stream.flush();
	imgRawData_.clear();

	// read image byte stream
	unsigned char ch;
	size_t numData = imgWidth_ * imgHeight_;
	for (size_t ii = 0; ii < numData; ii++)
	{
		ch = static_cast<unsigned char>(stream.read(1).at(0).toLatin1());
		imgRawData_.push_back(255 - ch);
	}

	return true;
}

void VSPluginMarkerLocalization::MappingNode::setStaticGridmap()
{
	// TODO: remove hardcoded resolution and offset
	std::pair<double, double> offset;
	offset.first = -20.0;
	offset.second = -20.0;
	staticGridmap_.importMap(imgWidth_, imgHeight_, 0.1, offset, imgRawData_);
}

void VSPluginMarkerLocalization::MappingNode::setLaserscanConversionData()
{
	if (!getBaseNode() || !getLaserExtension())
		return;

	// calculate transform from laserscanner to base 
	VSM::Frame sensorToBaseTransform = getBaseNode()->getWorldFrame().getInverse() * getLaserExtension()->getParentVSD3DNode()->getWorldFrame();

	staticGridmap_.setRangesConversionParam(getLaserExtension()->getAngleOffset(), getLaserExtension()->getResolution(), sensorToBaseTransform);

}
