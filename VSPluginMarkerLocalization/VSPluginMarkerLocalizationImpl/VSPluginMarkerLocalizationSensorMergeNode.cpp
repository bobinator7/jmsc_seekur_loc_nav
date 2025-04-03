// header
#include "../VSPluginMarkerLocalizationSensorMergeNode.h"

// other
#include "../VSPluginMarkerLocalizationProject.h"

#include "Lib/VSDIO/VSDIOInputVSMMatrixNxM.h"
#include "Lib/VSDIO/VSDIOOutputVSMMatrixNxM.h"
#include "Lib/VSD/VSDConnection.h"
#include "Plugin/VSLibMarkerLocalization/VSLibMarkerLocalizationTools.h"

#include <unordered_map>

VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::MarkerEKFSensorMergeNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataBool2DMarkerOutput(this)
, dataIntNumberOfCameras(this)
, dataIntPropertyOutputFrequency(this)
, num_of_input_slots_(4)
{
   timer_ = new QTimer();
}

VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::~MarkerEKFSensorMergeNode()
{
}

/* management (start) */
VSDIO::ExtensionIOBoard* VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::getIOBoard()
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

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::postHasBeenAddedToElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   // abort if no IO-Board or GUI
   ioBoard = getIOBoard();
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;
   if (!ioBoard)
      return;

   // property modified functions
   connect(this
      , SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*))
      , this
      , SLOT(slotPropertiesModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
   
   // initialize properties
	dataBool2DMarkerOutput.set(true);
   dataIntNumberOfCameras.set(4);
   dataIntPropertyOutputFrequency.set(2);

   // initialize inputs
   int num_of_inputs = getIntNumberOfCameras();
   for (int ii = 0; ii < num_of_inputs; ii++)
   {
      std::string str = "detectedMarkerFramesIn_" + std::to_string(ii);
      QString input_name = QString(str.c_str());
      VSDIO::Input* input = ioBoard->findInputByName(input_name);
      if (!input)
      {
         input = ioBoard->createInput<VSDIO::InputVSMMatrixNxM>(input_name);
      }
      connect(input
            , SIGNAL(signalInputValueModified(VSDIO::Input*))
            , this
            , SLOT(slotMyInputValueModified(VSDIO::Input*)));
   }

   // initialize outputs
   VSDIO::Output* output = ioBoard->findOutputByName("detectedMarkerFramesOut");
   if (!output)
   {
      output = ioBoard->createOutput<VSDIO::OutputVSMMatrixNxM>("detectedMarkerFramesOut");
   }

   // intialize timer
   dataIntPropertyOutputFrequency.set(2);
   VSD::Connection::connect(timer_, SIGNAL(timeout()), this, SLOT(slotTimedOutput()));
   timer_->start(1 / getIntPropertyOutputFrequency());
}

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;

   VSPluginMarkerLocalization::MarkerEKFSensorMergeNode* thisInstance = simStateInstance->instanceCast<VSPluginMarkerLocalization::MarkerEKFSensorMergeNode*>();
   if (thisInstance)
   {
      const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
      VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
      if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("intNumberOfCameras"))
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
               std::string str = "detectedMarkerFramesIn_" + std::to_string(num_of_input_slots_);
               QString input_name = QString(str.c_str());
               VSDIO::Input* input = ioBoard->findInputByName(input_name);
               ioBoard->removeInput(input);
            }
         }
         else
         {
            while (num_of_input_slots_ < new_num_of_input_slots)
            {
               std::string str = "detectedMarkerFramesIn_" + std::to_string(num_of_input_slots_);
					QString input_name = QString(str.c_str());
               VSDIO::Input* input = ioBoard->findInputByName(input_name);
               if (!input)
               {
                  input = ioBoard->createInput<VSDIO::InputVSMMatrixNxM>(input_name);
               }
               connect(input
                  , SIGNAL(signalInputValueModified(VSDIO::Input*))
                  , this
                  , SLOT(slotMyInputValueModified(VSDIO::Input*)));

               ++num_of_input_slots_;
            }
         }
      }
      else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("intPropertyOutputFrequency"))
      {
         timer_->setInterval(1 / getIntPropertyOutputFrequency());
      }
   }
}

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

   timer_->stop();
   VSD::Connection::disconnect(timer_, SIGNAL(timeout()), this, SLOT(slotTimedOutput()));
}

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::slotMyInputValueModified(VSDIO::Input* input)
{
   // get value of the bool input "execute" defined by base class
   if (!getInputValueExecute())
      return;

   // test if property values are valid
   if (!propertyCheck())
      return;

   // process the input value
   updateSensorData(input);

}
/* management (end) */

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::slotTimedOutput()
{
   // TODO: call this function with a QTimer
   // priority thread lock
   std::unique_lock<std::mutex> lk_n(m_next_);
   std::unique_lock<std::mutex> lk_d(m_data_);
   lk_n.unlock();
      
   // merge markers from multiple cameras into single map
   std::unordered_map<int, VSM::PoseVector3Quaternion> combined_camera_markers;
   for (int ii = 0; ii < num_of_input_slots_; ii++)
   {
      std::unordered_map<int, VSM::PoseVector3Quaternion> single_camera_markers = marker_maps_[ii];
      for (const auto& cursor : single_camera_markers)
      {
         auto search = combined_camera_markers.find(cursor.first);
         if (search != combined_camera_markers.end())
         {
            // interpolate marker pose if duplicate
            VSM::Frame interpolated_frame = VSM::interpolate(search->second.getFrame(), 0.5, cursor.second.getFrame());

            search->second = VSM::PoseVector3Quaternion(interpolated_frame);
         }
         else
         {
            combined_camera_markers[cursor.first] = cursor.second;
         }
      }
   }

   // std::unordered_map<int, VSM::PoseVector3Quaternion> -> VSM::MatrixNxM
	VSM::MatrixNxM marker_matrix_out;
	if (!getBool2DMarkerOutput())
		marker_matrix_out = VSLibMarkerLocalization::Tools::vsm3DMarkerMatFromStdUnorderedMap3D(combined_camera_markers);
	else
		marker_matrix_out = VSLibMarkerLocalization::Tools::vsm2DMarkerMatFromStdUnorderedMap3D(combined_camera_markers);


   //VSM::MatrixNxM debugOut(4,1);
   //copyDataOnOutput(debugOut);
   copyDataOnOutput(marker_matrix_out);

   lk_d.unlock();
}

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::updateSensorData(VSDIO::Input* input)
{
   // priority thread lock
   std::unique_lock<std::mutex> lk_l(m_lowprio_);
   std::unique_lock<std::mutex> lk_n(m_next_);
   std::unique_lock<std::mutex> lk_d(m_data_);
   lk_n.unlock();

   // determine which camera input is from
   VSDIO::InputVSMMatrixNxM* inputVSMMatrixNxM = input->instanceCast<VSDIO::InputVSMMatrixNxM*>();
   if (!inputVSMMatrixNxM)
      return;
   
   std::string input_no = inputVSMMatrixNxM->getName().toUtf8().constData();
   char& c_num = input_no.back();
   int num = c_num - '0';

   // update internal map of markers for that specific camera
   QVariant variant_data = input->instanceCast<VSDIO::InputVSMMatrixNxM*>()->getVariant();
   VSM::MatrixNxM marker_matrix = variant_data.value<VSM::MatrixNxM>();

   // VSM::MatrixNxM -> std::unordered_map<int, VSM::PoseVector3Quaternion>
   std::unordered_map<int, VSM::PoseVector3Quaternion> marker_map = VSLibMarkerLocalization::Tools::stdUnorderedMap3DFromVsm3DMarkerMat(marker_matrix);

   marker_maps_[num] = marker_map;

   lk_d.unlock();
   lk_l.unlock();
}

bool VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::propertyCheck()
{
   if(getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return false;

   if (getIntNumberOfCameras() <= 0)
   {
      getMyEnvironment()->showInfo("VSPluginMarkerLocalization::MarkerEKFSensorMergeNode: Please set IntPropertyPixel!");
      return false;
   }

   if (getIntPropertyOutputFrequency() <= 0)
   {
      getMyEnvironment()->showInfo("VSPluginMarkerLocalization::MarkerEKFSensorMergeNode: Please set IntPropertyPixel!");
      return false;
   }

   return true;
}

void VSPluginMarkerLocalization::MarkerEKFSensorMergeNode::copyDataOnOutput(const VSM::MatrixNxM& marker_matrix_out)
{
   VSDIO::Output* output = getIOBoard()->findOutputByName("detectedMarkerFramesOut");

   if (output)
   {
      QVariant outputValue = QVariant::fromValue(marker_matrix_out);
      output->setLocalVariant(outputValue);
   }
}
