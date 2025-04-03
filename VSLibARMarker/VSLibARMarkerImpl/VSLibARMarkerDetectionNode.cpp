// header
#include "../VSLibARMarkerDetectionNode.h"

// specific
#include "../VSLibARMarkerTools.h"
#include "../VSLibARMarkerLandmark.h"
#include "../VSLibARMarkerLandmarkExtension.h"

#include "Plugin/VSLibVisualGPS/VSLibVisualGPSNavMap.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVInputCVMat.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVOutputCVMat.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVCommon.h"
#include "Lib/VSDIO/VSDIOInputBool.h"
#include "Lib/VSDIO/VSDIOExtensionIOBoard.h"

#include "Lib/VSD/VSDConnection.h"
#include <QtConcurrent/qtconcurrentrun.h>

#include "Main/VEROSIM/VEROSIMProject.h"

VSLibARMarker::DetectionNode::DetectionNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, mySimState(mySimState)
, dataCurrentDetectionsCount(this)
, working(false)
, resultWatcher(0)
, ioBoard(VSD::Ref<VSDIO::ExtensionIOBoard>())
{
   cv::redirectError(VSLibOpenCV::Common::OpenCVErrorCallback);
}

VSLibARMarker::DetectionNode::~DetectionNode()
{
}

VSDIO::ExtensionIOBoard* VSLibARMarker::DetectionNode::getIOBoard()
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

void VSLibARMarker::DetectionNode::postHasBeenAddedToElementContainer()
{
   //call base class method
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   connect(this
         , SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*))
         , this
         , SLOT(slotMyPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));

   ioBoard = getIOBoard();

   VSDIO::Input* inputExecute = ioBoard->findInputByName("enable");
   connect(inputExecute
         , SIGNAL(signalInputValueModified(VSDIO::Input*))
         , this
         , SLOT(slotExecuteValueModified(VSDIO::Input*)));
   
   VSDIO::Input* imageInput = ioBoard->findInputByName("image");
   if (!imageInput)
   {
      imageInput = getIOBoard()->createInput<VSLibOpenCV::InputCVMat>("image");
   }
   connect(imageInput
         , SIGNAL(signalInputValueModified(VSDIO::Input*))
         , this
         , SLOT(slotImageInputValueModified(VSDIO::Input*)));

   VSDIO::Output* resultImageOutput = ioBoard->findOutputByName("resultImage");
   if (!resultImageOutput)
   {
      resultImageOutput = getIOBoard()->createOutput<VSLibOpenCV::OutputCVMat>("resultImage");
   }

   VSDIO::Output* detectedMarkersOutput = ioBoard->findOutputByName("detectedMarkers");
   if (!detectedMarkersOutput)
   {
      detectedMarkersOutput = getIOBoard()->createOutput<VSLibOpenCV::OutputCVMat>("detectedMarkers");
   }

   resultWatcher = new QFutureWatcher<void>();
   VSD::Connection::connect(resultWatcher, SIGNAL(finished()), this, SLOT(slotDetectionFinished()));
}

void VSLibARMarker::DetectionNode::preWillBeRemovedFromElementContainer()
{
   //call base class method
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

   VSD::Connection::disconnect(resultWatcher, SIGNAL(finished()), this, SLOT(slotDetectionFinished()));
   if (resultWatcher->isRunning())
      resultWatcher->waitForFinished();
   delete resultWatcher;
   resultWatcher = 0;
}

void VSLibARMarker::DetectionNode::slotMyPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   // No property changes have to be supervised!
}

void VSLibARMarker::DetectionNode::slotInputValueModified(VSDIO::Input* input)
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::slotInputValueModified(input);
   if (!input)
   {
      return;
   }

   VSDIO::InputBool* inputBool = input->instanceCast<VSDIO::InputBool*>();
   if (!inputBool)
   {
      return;
   }
   if (inputBool->getName() != "execute")
   {
      return;
   }
   if (!getInputValueExecute())
   {
      VSDIO::Output* output = getIOBoard()->findOutputByName("resultImage");
      if (output)
      {
         QVariant result = QVariant::fromValue(VSLibOpenCV::Mat());
         output->setLocalVariant(result);
         setCurrentDetectionsCount(0);
      }
   }
}

void VSLibARMarker::DetectionNode::slotImageInputValueModified(VSDIO::Input* input)
{
   if (!getInputValueExecute())
   {
      return;
   }
   if (working)
      return;
   QVariant myValue = input->getVariant();
   currentImage = myValue.value<VSLibOpenCV::Mat>();
   if (currentImage.empty())
      return;
   if (!resultWatcher)
      return;
   working = true;
   cv::_OutputArray corners(currentMarkers.corners);
   cv::_OutputArray ids(currentMarkers.ids);
   resultWatcher->setFuture(QtConcurrent::run(this, &VSLibARMarker::DetectionNode::detectInThread, currentImage, corners, ids));
}

void VSLibARMarker::DetectionNode::detectInThread(const cv::Mat& image, cv::_OutputArray& corners, cv::_OutputArray& ids)
{
   Tools::detectMarkers(image,corners, ids);
}

void VSLibARMarker::DetectionNode::slotDetectionFinished()
{
   VSLibOpenCV::Mat resultImage = currentImage;
   working = false;
   if (!getInputValueExecute())
      return;

   setCurrentDetectionsCount(currentMarkers.size());
  
   if (!currentMarkers.empty())
   {
      cv::aruco::drawDetectedMarkers(resultImage, currentMarkers.corners, currentMarkers.ids);
   }

   VSDIO::Output* output = getIOBoard()->findOutputByName("resultImage");
   if (output)
   {
      QVariant result = QVariant::fromValue(VSLibOpenCV::Mat(resultImage));
      output->setLocalVariant(result);
   }
   output = getIOBoard()->findOutputByName("detectedMarkers");
   if (output)
   {
      VSLibOpenCV::Mat resultMarkers = currentMarkers.asMatrix();
      QVariant result = QVariant::fromValue(resultMarkers);
      output->setLocalVariant(result);
   }
}

bool VSLibARMarker::DetectionNode::isActive()
{
   return getInputValueExecute();
}