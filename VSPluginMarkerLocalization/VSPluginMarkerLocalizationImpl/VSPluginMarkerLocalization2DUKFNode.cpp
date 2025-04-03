// header
#include "../VSPluginMarkerLocalization2DUKFNode.h"

// other
#include "../VSPluginMarkerLocalizationProject.h"

#include "Plugin/VSLibMarkerLocalization/VSLibMarkerLocalizationTools.h"

#include "Lib/VSD/VSDConnection.h"
#include "Lib/VSDTools/VSDTools.h"

#include "Lib/VSDIO/VSDIOInputBool.h"
#include "Lib/VSDIO/VSDIOInputDouble.h"
#include "Lib/VSDIO/VSDIOInputVSMMatrixNxM.h"
#include "Lib/VSDIO/VSDIOOutputVSMFrame.h"
#include "Lib/VSDIO/VSDIOOutputVSMMatrixNxM.h"
#include "Lib/VSDIO/VSDIOOutputVSMVector3.h"

#include "Lib/VSS/VSSScheduler.h"

#include <array>
#include <chrono>


VSPluginMarkerLocalization::Marker2DUKFNode::Marker2DUKFNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataBoolEnableVisualization(this)
, dataBoolUseSimTime(this)
, dataIntNumberOfCameras(this)
, dataIntNumMarkerSeen(this)
, dataInitialRobotPose(this)
, dataInitialRobotCovariance(this)
, dataSystemNoiseCovarianceQ(this)
, dataMeasurementNoiseCovarianceR(this)
, dataStaticMarkerMapFile(this)
, dataUpdateRate(this)
, num_of_input_slots_(4)
, visu_(0)
{
	timer = new QTimer();
}

VSPluginMarkerLocalization::Marker2DUKFNode::~Marker2DUKFNode()
{
	if (visu_)
		delete visu_;
}

VSDIO::ExtensionIOBoard* VSPluginMarkerLocalization::Marker2DUKFNode::getIOBoard()
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
void VSPluginMarkerLocalization::Marker2DUKFNode::postHasBeenAddedToElementContainer()
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

   if (ioBoard)
   {
		// inputs
		VSDIO::Input* input;
      input = ioBoard->findInputByName("robot velocity");
      if (!input)
      {
         input = ioBoard->createInput<VSDIO::InputDouble>("robot velocity");
      }
      connect(input
            , SIGNAL(signalInputValueModified(VSDIO::Input*))
            , this
            , SLOT(slotVelocityModified(VSDIO::Input*)));

		input = ioBoard->findInputByName("robot rotation");
		if (!input)
		{
			input = ioBoard->createInput<VSDIO::InputDouble>("robot rotation");
		}
		connect(input
			, SIGNAL(signalInputValueModified(VSDIO::Input*))
			, this
			, SLOT(slotRotationModified(VSDIO::Input*)));

		//input = ioBoard->findInputByName("camera marker measurements");
		//if (!input)
		//{
		//	input = ioBoard->createInput<VSDIO::InputVSMMatrixNxM>("camera marker measurements");
		//}
		//connect(input
		//	, SIGNAL(signalInputValueModified(VSDIO::Input*))
		//	, this
		//	, SLOT(slotMeasurementModified(VSDIO::Input*)));

		input = ioBoard->findInputByName("execute");
		connect(input
			, SIGNAL(signalInputValueModified(VSDIO::Input*))
			, this
			, SLOT(slotEnableModified(VSDIO::Input*)));

		num_of_input_slots_ = getIntNumberOfCameras();
		for (int ii = 0; ii < num_of_input_slots_; ii++)
		{
			std::string str = "camera_marker_measurements_" + std::to_string(ii);

			updatedSincePrediction_[str] = false;

			QString input_name = QString(str.c_str());
			VSDIO::Input* input = ioBoard->findInputByName(input_name);
			if (!input)
			{
				input = ioBoard->createInput<VSDIO::InputVSMMatrixNxM>(input_name);
			}
			connect(input
				, SIGNAL(signalInputValueModified(VSDIO::Input*))
				, this
				, SLOT(slotMeasurementModified(VSDIO::Input*)));
		}


		// outputs
		VSDIO::Output* output;
      output = ioBoard->findOutputByName("robot pose");
      if (!output)
      {
         output = ioBoard->createOutput<VSDIO::OutputVSMVector3>("robot pose");
      }

		output = ioBoard->findOutputByName("robot frame");
		if (!output)
		{
			output = ioBoard->createOutput<VSDIO::OutputVSMFrame>("robot frame");
		}

		output = ioBoard->findOutputByName("marker map");
		if (!output)
		{
			output = ioBoard->createOutput<VSDIO::OutputVSMMatrixNxM>("marker map");
		}

		// initialize timer
		VSD::Connection::connect(timer, SIGNAL(timeout()), this, SLOT(slotUpdatePoseWithOdom()));
		timer->start(1000/getUpdateRate());

		// initialize time
		lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());

		// initialize properties
		if (getInitialRobotPose() == VSM::Vector3())
			setInitialRobotPose(VSM::Vector3(0,0,0));

		if (getInitialRobotCovariance() == VSM::Matrix3x3())
			setInitialRobotCovariance(VSM::Matrix3x3(0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.25));

		if (getSystemNoiseCovarianceQ() == VSM::Matrix3x3())
		{
			VSM::Matrix3x3 initQ(0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.004);
			setSystemNoiseCovarianceQ(initQ);
		}

		if (getMeasurementNoiseCovarianceR() == VSM::Matrix3x3())
		{
			VSM::Matrix3x3 initR(0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1);
			setMeasurementNoiseCovarianceR(initR);
		}

		robotPose_ = getInitialRobotPose();
		robotCovariance_ = getInitialRobotCovariance();
		filter_.setRobotPose(robotPose_, robotCovariance_);
		filter_.setSystemNoiseQ(getSystemNoiseCovarianceQ());
		filter_.setMeasurementNoiseR(getMeasurementNoiseCovarianceR());

		inVelocity_ = 0.;
		inRotation_ = 0.;

		

		// init visu
		if (getBoolEnableVisualization())
		{
			visu_ = new EKFVisu(getMySimState()->getMyEnvironment()->getMyProject(), this);
			visu_->initialize();
		}

		// init static map
	   //dataStaticMarkerMapFile.set("file:///E:/VEROSIM/002/Modelle/John_MA/Simulation_Flur/map/static_marker_map.csv");
		if (loadStaticMapFromFile())
		{
			filter_.setStaticMap(staticMarkerMap_);
		}
		else
		{
			filter_.resetStaticMap();
		}
   }
   else  //if (!ioBoard)
   {
      return;
   }
}

void VSPluginMarkerLocalization::Marker2DUKFNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

	timer->stop();
	VSD::Connection::disconnect(timer, SIGNAL(timeout()), this, SLOT(slotUpdatePoseWithOdom()));
}

// note: edit here to process data on input change
void VSPluginMarkerLocalization::Marker2DUKFNode::slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
	if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
		return;

	VSPluginMarkerLocalization::Marker2DUKFNode* thisInstance = simStateInstance->instanceCast<VSPluginMarkerLocalization::Marker2DUKFNode*>();
	if (thisInstance)
	{
		const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
		VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
		if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("systemNoiseCovarianceQ"))
		{
			// TODO: check if Q is valid covariance matrix first!
			filter_.setSystemNoiseQ(getSystemNoiseCovarianceQ());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("measurementNoiseCovarianceR"))
		{
			// TODO: check if R is valid covariance matrix first!
			filter_.setMeasurementNoiseR(getMeasurementNoiseCovarianceR());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("initialRobotPose"))
		{
			// TODO: replace initial pose with actual robot pose ?
			robotPose_ = getInitialRobotPose();
			filter_.setRobotPose(robotPose_, robotCovariance_);
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("initialRobotCovariance"))
		{
			// TODO: replace initial covariance with actual robot covariance ?
			robotCovariance_ = getInitialRobotCovariance();
			filter_.setRobotPose(robotPose_, robotCovariance_);
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("updateRate"))
		{
			timer->setInterval(1000/getUpdateRate());
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("intNumberOfCameras"))
		{
			// adjust input slots based on available cameras
			int new_num_of_input_slots = getIntNumberOfCameras();

			if (num_of_input_slots_ == new_num_of_input_slots)
				return;
			else if (num_of_input_slots_ > new_num_of_input_slots)
			{
				while (num_of_input_slots_ > new_num_of_input_slots)
				{
					--num_of_input_slots_;
					std::string str = "camera_marker_measurements_" + std::to_string(num_of_input_slots_);
					
					// remove entry from updated measrument input table
					auto search = updatedSincePrediction_.find(str);
					if (search != updatedSincePrediction_.end())
						updatedSincePrediction_.erase(search);

					// remove ioBoard input
					QString input_name = QString(str.c_str());
					VSDIO::Input* input = ioBoard->findInputByName(input_name);
					ioBoard->removeInput(input);
				}
			}
			else
			{
				while (num_of_input_slots_ < new_num_of_input_slots)
				{
					std::string str = "camera_marker_measurements_" + std::to_string(num_of_input_slots_);

					// add entry to updated measurement input table
					updatedSincePrediction_[str] = false;

					// add ioBoard input
					QString input_name = QString(str.c_str());
					VSDIO::Input* input = ioBoard->findInputByName(input_name);
					if (!input)
					{
						input = ioBoard->createInput<VSDIO::InputVSMMatrixNxM>(input_name);
					}
					connect(input
						, SIGNAL(signalInputValueModified(VSDIO::Input*))
						, this
						, SLOT(slotMeasurementModified(VSDIO::Input*)));

					++num_of_input_slots_;
				}
			}
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("boolEnableVisualization"))
		{
			if (getBoolEnableVisualization())
			{
				visu_ = new EKFVisu(getMySimState()->getMyEnvironment()->getMyProject(), this);
				visu_->initialize();
			}
			else
			{
				delete visu_;
				visu_ = 0;
			}
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("staticMarkerMapFile"))
		{
			if (loadStaticMapFromFile())
			{
				filter_.setStaticMap(staticMarkerMap_);
			}
			else
			{
				filter_.resetStaticMap();
			}
		}
		else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("boolUseSimTime"))
		{
			if (getBoolUseSimTime())
				lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
			else
				lastUpdateTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		}
	}
}

void VSPluginMarkerLocalization::Marker2DUKFNode::slotVelocityModified(VSDIO::Input* input)
{
   // get value of the bool input "execute" defined by base class
   if (!getInputValueExecute())
      return;

   // test if property values are valid
   //if (!propertyCheck())
   //   return;

	QVariant inputVariant = input->getVariant();
	inVelocity_ = inputVariant.value<double>();
}

void VSPluginMarkerLocalization::Marker2DUKFNode::slotEnableModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
	{
		// filter reset
		filter_.reset();
		robotPose_ = getInitialRobotPose();
		robotCovariance_ = getInitialRobotCovariance();
		filter_.setRobotPose(robotPose_, robotCovariance_);
		filter_.setSystemNoiseQ(getSystemNoiseCovarianceQ());
		filter_.setMeasurementNoiseR(getMeasurementNoiseCovarianceR());

		markerPoses_.clear();
		markerCovariances_.clear();
	}
	else
	{
		// time reset
		// TODO: use wall time if not in sim
		if (getBoolUseSimTime())
			lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
		else
			lastUpdateTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	}
}

void VSPluginMarkerLocalization::Marker2DUKFNode::slotRotationModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
		return;

	// test if property values are valid
	//if (!propertyCheck())
	//   return;

	QVariant inputVariant = input->getVariant();
	inRotation_ = inputVariant.value<double>();
}

void VSPluginMarkerLocalization::Marker2DUKFNode::slotMeasurementModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
		return;

	QVariant inputVariant = input->getVariant();
	inMarkerMeasurement_ = inputVariant.value<VSM::MatrixNxM>();

	// check if updated already since prediction
	std::string str = input->getName().toUtf8().constData();
	auto search = updatedSincePrediction_.find(str);
	if (search != updatedSincePrediction_.end())
	{
		if (search->second)
			return;

		search->second = true;
	}

	// abort if empty
	if (inMarkerMeasurement_.columns() < 1)
		return;

	// get seen marker
	for (size_t ii = 0; ii < inMarkerMeasurement_.columns(); ii++)
	{
		markerSeenUntilPrediction_.insert(inMarkerMeasurement_.getElement(0, ii));
	}

	// update step!
	filter_.updateStep(inMarkerMeasurement_);

	robotPose_ = filter_.getRobotPose();
	robotCovariance_ = filter_.getRobotCovariance();

	// get changed markers
	markerPoses_ = filter_.getMarkerPoseMap();
	markerCovariances_ = filter_.getMarkerCovarianceMap();

	// set output
	VSM::MatrixNxM matMarkerPoses = VSLibMarkerLocalization::Tools::vsm2DMarkerMatFromStdUnorderedMap2D(markerPoses_);
	if (ioBoard)
	{
		VSDIO::Output* output = ioBoard->findOutputByName("marker map");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(matMarkerPoses);
			output->setLocalVariantAndForceNotify(outputVariant);
		}

		output = ioBoard->findOutputByName("robot pose");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(robotPose_);
			output->setLocalVariantAndForceNotify(outputVariant);
		}

		output = ioBoard->findOutputByName("robot frame");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(vec2frame(robotPose_));
			output->setLocalVariantAndForceNotify(outputVariant);
		}
	}
}

void VSPluginMarkerLocalization::Marker2DUKFNode::slotUpdatePoseWithOdom()
{
	if (!getInputValueExecute())
		return;

   // get time period since last update
	double timePeriod;
	if (getDataBoolUseSimTime())
	{
		timePeriod = VSS::Scheduler::getCurrentTaskTime(getMySimState()) - lastUpdateTime_;
	}
	else 
	{
		double now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		timePeriod = now - lastUpdateTime_;
	}

	// enable all measurement slots to update
	for (auto& it : updatedSincePrediction_)
	{
		it.second = false;
	}

	// set seen marker count and reset seen marker set
	dataIntNumMarkerSeen.set(markerSeenUntilPrediction_.size());
	markerSeenUntilPrediction_.clear();

	// predict step!
	if (!(timePeriod < 0.00001))
		filter_.predictStep(inVelocity_, inRotation_, timePeriod);

	robotPose_ = filter_.getRobotPose();
	robotCovariance_ = filter_.getRobotCovariance();
	
	if (getDataBoolUseSimTime())
	{
		lastUpdateTime_ = VSS::Scheduler::getCurrentTaskTime(getMySimState());
	}
	else
	{
		lastUpdateTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	}

	// set output
	if (ioBoard) {
		VSDIO::Output* output = ioBoard->findOutputByName("robot pose");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(robotPose_);
			output->setLocalVariantAndForceNotify(outputVariant);
		}

		output = ioBoard->findOutputByName("robot frame");
		if (output)
		{
			QVariant outputVariant = QVariant::fromValue(vec2frame(robotPose_));
			output->setLocalVariantAndForceNotify(outputVariant);
		}
	}
}

// public get fcns
VSM::Vector3 VSPluginMarkerLocalization::Marker2DUKFNode::getRobotPose()
{
	return robotPose_;
}

VSM::Matrix3x3 VSPluginMarkerLocalization::Marker2DUKFNode::getRobotCovariance()
{
	return robotCovariance_;
}

std::unordered_map<int, VSM::Vector3> VSPluginMarkerLocalization::Marker2DUKFNode::getMarkerPoses()
{
	return markerPoses_;
}

std::unordered_map<int, VSM::Matrix3x3> VSPluginMarkerLocalization::Marker2DUKFNode::getMarkerCovariances()
{
	return markerCovariances_;
}

// note: sanity check
bool VSPluginMarkerLocalization::Marker2DUKFNode::propertyCheck()
{
   if(getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return false;

   return true;
}

bool VSPluginMarkerLocalization::Marker2DUKFNode::loadStaticMapFromFile()
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

	staticMarkerMap_.clear();

	char ch;
	while (!stream.atEnd())
	{
		int id;
		double x, y, theta;
		stream >> id;
		stream >> x;
		stream >> y;
		stream >> theta;
		
		stream >> ch;

		VSM::Vector3 markerPose(x, y, theta);
		staticMarkerMap_[id] = markerPose;
	}

	return true;
}


VSM::Frame VSPluginMarkerLocalization::Marker2DUKFNode::vec2frame(VSM::Vector3 vecIn)
{
	double height = 1;

	VSM::Matrix3x3 orientation = VSM::Matrix3x3(true);
	orientation.setRotZ(vecIn[2], true);

	return VSM::Frame(orientation, VSM::Vector3(vecIn[0], vecIn[1], height));
}

