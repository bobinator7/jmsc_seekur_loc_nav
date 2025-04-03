// header
#include "../VSLibARMarkerLocalizationNode.h"

// specific
#include "../VSLibARMarkerTools.h"
#include "../VSLibARMarkerLandmark.h"
#include "../VSLibARMarkerLandmarkExtension.h"
#include "../VSLibARMarkerNode.h"

#include "Lib/VSLibImageProcessing/VSLibImageProcessingCalibrationTools.h"
#include "Plugin/VSLibVisualGPS/VSLibVisualGPSNavMap.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVInputCVMat.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVOutputCVMat.h"
#include "Lib/VSDIO/VSDIOInputBool.h"
#include "Lib/VSDIO/VSDIOOutputDouble.h"
#include "Lib/VSDIO/VSDIOOutputVSMFrame.h"
#include "Lib/VSDIO/VSDIOExtensionIOBoard.h"

#include "Lib/VSD/VSDConnection.h"
#include "Lib/VSDTools/VSDTools.h"
#include <QtConcurrent/qtconcurrentrun.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "Plugin/VSLibOpenCV/VSLibOpenCVCommon.h"

#include "Main/VEROSIM/VEROSIMProject.h"

VSLibARMarker::LocalizationNode::LocalizationNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, mySimState(mySimState)
, dataMarkerEdgeLength(this)
, dataFindAveragePose(this)
, dataAppendUnknownMarkersToMap(this)
, dataMaxLocalizationNodeDistance(this)
, dataMinMarkerViewingAngle(this)
, dataNumberOfVisibleMarkers(this)
, dataShowBoxVisu(this)
, dataShowAxisVisu(this)
, dataCameraCalibrationFile(this)
, dataCameraExtension(this)
, dataNode(this)
, dataBaseNode(this)
, dataMap(this)
, cameraExtension(0)
, visu(0)
, working(false)
, resultWatcher(0)
, ioBoard(VSD::Ref<VSDIO::ExtensionIOBoard>())
{
   cv::redirectError(VSLibOpenCV::Common::OpenCVErrorCallback);
}

VSLibARMarker::LocalizationNode::~LocalizationNode()
{
   if (visu)
      delete visu;
}

VSDIO::ExtensionIOBoard* VSLibARMarker::LocalizationNode::getIOBoard()
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

void VSLibARMarker::LocalizationNode::postHasBeenAddedToElementContainer()
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

   VSDIO::Output* estimatedFrameOutput = ioBoard->findOutputByName("estimatedFrame");
   if (!estimatedFrameOutput)
   {
      estimatedFrameOutput = getIOBoard()->createOutput<VSDIO::OutputVSMFrame>("estimatedFrame");
   }

   VSDIO::Output* validityOutput = ioBoard->findOutputByName("validity");
   if (!validityOutput)
   {
      validityOutput = getIOBoard()->createOutput<VSDIO::OutputDouble>("validity");
   }

   if (getCameraExtension())
   {
      cameraExtension = getCameraExtension();
      connect(cameraExtension, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
              this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
   }

   if (!visu)
   {
      visu = new BoxVisu(mySimState->getMyEnvironment()->getMyProject(), this);
      visu->initialize();
   }

   if (getCameraCalibrationFile().isEmpty())
   {
      buildCameraMatrix();
   }
   else
   {
      if (!loadCameraConfigFromFile())
      {
         buildCameraMatrix();
      }
   }
   
   resultWatcher = new QFutureWatcher<void>();
   VSD::Connection::connect(resultWatcher, SIGNAL(finished()), this, SLOT(slotDetectionFinished()));
}

void VSLibARMarker::LocalizationNode::preWillBeRemovedFromElementContainer()
{
   //call base class method
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

   VSD::Connection::disconnect(resultWatcher, SIGNAL(finished()), this, SLOT(slotDetectionFinished()));
   if (resultWatcher->isRunning())
      resultWatcher->waitForFinished();
   delete resultWatcher;
   resultWatcher = 0;
}

void VSLibARMarker::LocalizationNode::slotMyPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   if(getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;

   VSLibARMarker::LocalizationNode* thisInstance = simStateInstance->instanceCast<VSLibARMarker::LocalizationNode*>();
   if (thisInstance)
   {
      const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
      VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
      if (  metaProperty == thisMetaInstance->findMyOrBaseClassProperty("cameraExtension") )
      {
         if (getCameraExtension() != cameraExtension)
         {
            if (cameraExtension)
            {
               disconnect(cameraExtension, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
                          this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
               cameraExtension = 0;
            }
            if (getCameraExtension())
            {
               cameraExtension = getCameraExtension();
               connect(cameraExtension, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
                       this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
            }
            if (!loadCameraConfigFromFile())
            {
               buildCameraMatrix();
            }
         }
        
      }
      else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("cameraCalibrationFile"))
      {
         if (!loadCameraConfigFromFile())
         {
            buildCameraMatrix();
         }
      }
      else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("markerEdgeLength"))
      {
         if (getMarkerEdgeLength() <= 0)
            setMarkerEdgeLength(0.1);
      }
   }
}

void VSLibARMarker::LocalizationNode::slotInputValueModified(VSDIO::Input* input)
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
   if (inputBool->getValue())
   {
      if (getCameraExtension() && getBaseNode())
      {
         sensorFrameRelToBaseNode = (getBaseNode()->getWorldFrame().getInverse() * getCameraExtension()->getParentVSD3DNode()->getWorldFrame()).getInverse();
      }
      else
      {
         sensorFrameRelToBaseNode.setIdentity();
      }
      return;
   }

   VSDIO::Output* validityOutput = getIOBoard()->findOutputByName("validity");
   if (validityOutput)
   {
      QVariant validity = QVariant::fromValue(0.0);
      validityOutput->setLocalVariant(validity);
   }
}


void VSLibARMarker::LocalizationNode::slotImageInputValueModified(VSDIO::Input* input)
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
   resultWatcher->setFuture(QtConcurrent::run(this, &VSLibARMarker::LocalizationNode::detectInThread, currentImage, getMarkerEdgeLength(), corners, ids));    
}

void VSLibARMarker::LocalizationNode::detectInThread(const cv::Mat& image, const double& markerEdgeLength, cv::_OutputArray& corners, cv::_OutputArray& ids)
{
   Tools::detectMarkers(image, corners, ids);
}

void VSLibARMarker::LocalizationNode::slotDetectionFinished()
{
   VSLibOpenCV::Mat resultImage(currentImage);

   //if (!currentMarkers.empty())
   //{
   //   cv::aruco::drawDetectedMarkers(resultImage, currentMarkers.corners, currentMarkers.ids);
   //}

   MarkerDetections visibleMarkersInRange;
   markersUnknown.clear();
   
   currentMarkers.estimateMarkerPoses(getMarkerEdgeLength(), cameraMatrix, distCoeffs);
   for (int i = 0; i < currentMarkers.size(); ++i)
   {
      bool knownInMap = false;
      if (getMap())
      {
         foreach(VSLibVisualGPS::Landmark* instance, getMap()->getModel()->getLandmarks())
         {
            Landmark* lm = (Landmark*)instance;
            if (lm == 0)
               continue;
            if (((LandmarkExtension*)lm->getExtension())->getId() == currentMarkers.ids.at(i))
            {
               knownInMap = true;
               break;
            }
         }
      }
      if (getMaxLocalizationNodeDistance() <= 0 && getMinMarkerViewingAngle() <= 0)
      {
         if (!knownInMap)
         {
            markersUnknown.append(currentMarkers.at(i));
            continue;
         }
         else
         {
            visibleMarkersInRange.append(currentMarkers.at(i));
            continue;
         }
      }
      bool reject = false;
      if (getMaxLocalizationNodeDistance() > 0)
      {
         if (cv::norm(currentMarkers.tvecs.at(i)) > getMaxLocalizationNodeDistance())
         {
            reject = true;
         }
      }
      if (getMinMarkerViewingAngle() > 0)
      {
         cv::Mat mat;
         cv::Rodrigues(currentMarkers.rvecs.at(i), mat);
         VSM::Matrix3x3 orientation = VSLibOpenCV::Common::vsmMatrix3x3FromCVMat(mat);
         double roll = VSM::radToDeg(orientation.getRoll());
         double pitch = VSM::radToDeg(orientation.getPitch());
         double yaw = VSM::radToDeg(orientation.getYaw());
         double roll_rad = orientation.getRoll();
         double pitch_rad = orientation.getPitch();
         double yaw_rad = orientation.getYaw();

         // check pitch and yaw instead of roll and yaw, because angles are still in openCV coordinate system
         if (abs(orientation.getPitch()) < getMinMarkerViewingAngle() && VSM::pi-abs(orientation.getYaw()) < getMinMarkerViewingAngle())
         {
            reject = true;
            //qDebug() << "rejected!";
         }
      }
      if (!reject)
      {
         if (!knownInMap)
         {
            markersUnknown.append(currentMarkers.at(i));
         }
         else
         {
            visibleMarkersInRange.append(currentMarkers.at(i));
         }
      }
   }
   
   setNumberOfVisibleMarkers(visibleMarkersInRange.size());
   cv::aruco::drawDetectedMarkers(resultImage, visibleMarkersInRange.corners, visibleMarkersInRange.ids);
   visibleMarkers = visibleMarkersInRange;

   VSDIO::Output* resultOutput = getIOBoard()->findOutputByName("resultImage");
   if (resultOutput)
   {
      QVariant result = QVariant::fromValue(resultImage);
      resultOutput->setLocalVariant(result);
      emit signalTriggerPaintAllWindows();
   }

   VSDIO::Output* estimatedFrameOutput = getIOBoard()->findOutputByName("estimatedFrame");
   VSDIO::Output* validityOutput = getIOBoard()->findOutputByName("validity");
   if (estimatedFrameOutput && estimatedFrameOutput->getIsConnected() && !visibleMarkersInRange.empty())
   {
      VSM::Frame frame = estimateCameraPose();
      frame = frame*sensorFrameRelToBaseNode;
      if (frame != VSM::Frame())
      {
         QVariant estimatedFrame = QVariant::fromValue(frame);
         estimatedFrameOutput->setLocalVariant(estimatedFrame);

         if (validityOutput)
         {
            QVariant validity = QVariant::fromValue(1.0);
            validityOutput->setLocalVariant(validity);
         }
      }
   }
   else
   {
      if (validityOutput)
      {
         QVariant validity = QVariant::fromValue(0.0);
         validityOutput->setLocalVariant(validity);
      }
   }
   if (!getInputValueExecute())
   {
      if (validityOutput)
      {
         QVariant validity = QVariant::fromValue(0.0);
         validityOutput->setLocalVariant(validity);
      }
   }
   working = false;
}

void VSLibARMarker::LocalizationNode::slotCameraPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   Q_UNUSED(simStateInstance);
   Q_UNUSED(metaProperty);
   buildCameraMatrix();
}

void VSLibARMarker::LocalizationNode::buildCameraMatrix()
{
   if (getCameraExtension() && getMarkerEdgeLength() > 0)
   {
      double fov = getCameraExtension()->getXAngle();
      int width = getCameraExtension()->getImageWidth();
      int height = getCameraExtension()->getImageHeight();
      double f = VSLibImageProcessing::CalibrationTools::fovToFocalLength(fov)*width;
      double principleX = width/2;
      double principleY = height/2;
      cameraMatrix = (cv::Mat_<float>(3,3) << f, 0, principleX, 0, f, principleY, 0, 0, 1);
      distCoeffs = cv::Mat_<float>::zeros(1, 5);
   }
   else
   {
      cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
      distCoeffs = cv::Mat_<float>::zeros(1, 5);
   }
}

bool VSLibARMarker::LocalizationNode::loadCameraConfigFromFile()
{
   if (getCameraCalibrationFile().isEmpty())
      return false;
   QString filename = VSDTools::compileAbsoluteUrl(getMyModel(), getCameraCalibrationFile()).toLocalFile();
   QFile configFile(filename);
   if (!configFile.exists())
      return false;
   configFile.open(QFile::ReadOnly);
   QTextStream stream(&configFile);
   cameraMatrix = cv::Mat_<float>::zeros(3, 3);
   distCoeffs = cv::Mat_<float>::zeros(1, 5);
   char ch = '\0';
   for (int i = 0; i<13; ++i) // filter out string: "cameraMatrix("
   {
      stream >> ch;
   }
   for (int r = 0; r<3; ++r)
   {
      for (int c = 0; c<3; ++c)
      {
         stream >> cameraMatrix.ptr<float>(r)[c] >> ch;
      }
   }
   stream >> ch; // filter out char: '\n'
   for (int i = 0; i<11; ++i) // filter out string: "distCoeffs("
   {
      stream >> ch;
   }
   for (int c = 0; c<5; ++c)
   {
      stream >> distCoeffs.ptr<float>(0)[c] >> ch;
   }
   return true;
}

//double VSLibARMarker::LocalizationNode::focalLengthFromFieldOfView(double fov)
//{
//   return 1.0 / (2.0*tan(fov/2.0));
//
//}

VSLibARMarker::MarkerDetections& VSLibARMarker::LocalizationNode::getVisibleMarkers()
{
   if (!visibleMarkers.empty() && visibleMarkers.tvecs.empty())
      visibleMarkers.estimateMarkerPoses(getMarkerEdgeLength(), cameraMatrix, distCoeffs);
   return visibleMarkers;
}

bool VSLibARMarker::LocalizationNode::isActive()
{
   return getInputValueExecute();
}

VSM::Frame VSLibARMarker::LocalizationNode::estimateCameraPose()
{
   //sanity checks:
   VSM::Frame result;
   if (visibleMarkers.empty())
      return result;
   if (getMap() == 0)
      return result;
   if (getMap()->getLandmarks().isEmpty())
      return result;

   visibleMarkers.estimateMarkerPoses(getMarkerEdgeLength(), cameraMatrix, distCoeffs);

   /* Vorgehen:
   1. Wähle den nächsten (euklidischer Abstand) Marker.
      optional: wähle keinen Marker, der frontal vor der Kamera steht (wegen Singularitäten)
   2. Suche den Marker per Id in der Navigationskarte
   3. Berechne den Kamera-Frame und gib ihn zurück
   */

   if (getFindAveragePose())
   {
      VSM::Vector3 averagePosition(0,0,0);
      VSM::Quaternion averageOrientation(0,0,0,0);

      int counter = 0;
      for (int i = 0; i < visibleMarkers.size(); ++i)
      {
         VSM::Frame landmarkFrame;
         bool success = false;
         foreach(VSLibVisualGPS::Landmark* instance, getMap()->getModel()->getLandmarks())
         {
            Landmark* lm = (Landmark*)instance;
            if (lm == 0)
               continue;
            if (((LandmarkExtension*)lm->getExtension())->getId() == visibleMarkers.ids.at(i))
            {
               landmarkFrame = lm->getWorldFrame();
               success = true;
            }
         }
         if (!success)
            continue;
     
         VSM::Frame frame = Tools::getCameraWorldFrame(visibleMarkers.tvecs.at(i), visibleMarkers.rvecs.at(i), landmarkFrame);
         averagePosition += frame.getPosition();
         averageOrientation += frame.getOrientation().getQuaternion();
         counter++;
      }
      averagePosition = averagePosition / visibleMarkers.size();
      averageOrientation = averageOrientation * (1.0/ counter);
      result = VSM::Frame(averageOrientation.getMatrix3x3(), averagePosition);
   }
   else
   {
      //Schritt 1
      double d = VSM::maxDouble;
      int index = -1;
      for (int i = 0; i < visibleMarkers.size(); ++i)
      {
         cv::Vec3d t = visibleMarkers.tvecs.at(i);
         double di = cv::norm(t);
         if (di < d)
         {
            d = di;
            index = i;
         }
      }
      if (index<0)
         return result;

      //Schritt 2
      VSM::Frame landmarkFrame;
      bool success = false;
      foreach(VSLibVisualGPS::Landmark* instance, getMap()->getModel()->getLandmarks())
      {
         Landmark* lm = (Landmark*)instance;
         if (lm == 0)
            continue;
         if (((LandmarkExtension*)lm->getExtension())->getId() == visibleMarkers.ids.at(index))
         {
            landmarkFrame = lm->getWorldFrame();
            success = true;
         }
      }
      if (!success)
         return result;

      //Schritt 3
      result = Tools::getCameraWorldFrame(visibleMarkers.tvecs.at(index), visibleMarkers.rvecs.at(index), landmarkFrame);
   }
   
   if (getAppendUnknownMarkersToMap())
   {
      if (markersUnknown.size() != 0)
      {
         appendNewMarkersToMap(markersUnknown, result);
         markersUnknown.clear();
      }
   }

   return result;
}

void VSLibARMarker::LocalizationNode::appendNewMarkersToMap(QList<Marker> newMarkers, VSM::Frame cameraFrame)
{
   VSD::ModelInstance* parentInstance = getMap()->getParentForNewForNewLandmarks();
   if (!parentInstance)
   {
      showWarning("Cannot add Landmarks! Set PropertyRef:'ParentForNewForNewLandmarks' in NavigationMap!");
      return;
   }
   if (!parentInstance->inherits<VSD::Node*>())
      return;
   VSD::Node* parentNode = parentInstance->instanceCast<VSD::Node*>();
   VSD::SimState* simState = parentNode->getMySimState();
   foreach(Marker marker, newMarkers)
   {
      VSLibARMarker::Node* markerNodeInstance = simState->newSimStateInstance<VSLibARMarker::Node>();
      markerNodeInstance->setId(marker.getId());
      markerNodeInstance->setMarkerEdgeLength(getMarkerEdgeLength());
      markerNodeInstance->setShow(false);
      parentNode->getChildNodes().append(markerNodeInstance);
      LandmarkExtension* lmExtension = markerNodeInstance->findFirstExtensionInherits<LandmarkExtension*>();
      if (!lmExtension)
      {
         showWarning("Strange things happen. The newly created MarkerNode should have a LandmarkExtension but does not! 8-|");
         continue;
      }
      VSM::Frame markerWorldFrame = Tools::getMarkerWorldFrame(marker.getTvec(), marker.getRvec(), cameraFrame);
      markerNodeInstance->setWorldFrame(markerWorldFrame);

      getMap()->getLandmarks().append(lmExtension);
      getMap()->getModel()->addLandmark(lmExtension->createModel());
   }
}