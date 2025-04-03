// header
#include "../VSLibARMarkerExtensionMarkerVisu.h"

// specific
#include "../VSLibARMarkerTools.h"

#include "Lib/VSLibImageProcessing/VSLibImageProcessingCalibrationTools.h"
//#include "../VSLibARMarkerInputMarkerVector.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVInputCVMat.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVCommon.h"
#include "Lib/VSDIO/VSDIOInputBool.h"

#include "Lib/VSD/VSDConnection.h"
#include "Lib/VSDTools/VSDTools.h"
#include "../VSLibARMarkerTools.h"

#include "Main/VEROSIM/VEROSIMProject.h"

//OpenCV-Debugging!
int __cdecl myCVErrorCallback( int status, const char* func_name, const char* err_msg, const char* file_name, int line, void* userdata )
{
   Q_UNUSED(userdata);
   QString msg;
   msg.sprintf("cv::Exception in %s @ %d - %s: %s",file_name,line,func_name,err_msg);
   qDebug() << msg;
   return status;
}

VSLibARMarker::ExtensionMarkerVisu::ExtensionMarkerVisu(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, mySimState(mySimState)
, dataMarkerEdgeLength(this)
, dataShowBoxVisu(this)
, dataShowAxisVisu(this)
, dataMarkerId(this)
, dataMoveCamera(this)
, dataCameraExtension(this)
, dataNode(this)
, dataCameraCalibrationFile(this)
, cameraExtension(0)
, visu(0)
//, working(false)
, ioBoard(VSD::Ref<VSDIO::ExtensionIOBoard>())
{
   cv::redirectError(myCVErrorCallback);
}

VSLibARMarker::ExtensionMarkerVisu::~ExtensionMarkerVisu()
{
   if (visu)
      delete visu;
}

VSDIO::ExtensionIOBoard* VSLibARMarker::ExtensionMarkerVisu::getIOBoard()
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

void VSLibARMarker::ExtensionMarkerVisu::postHasBeenAddedToElementContainer()
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
   
   VSDIO::Input* markerInput = ioBoard->findInputByName("detected markers");
   if (!markerInput)
   {
      markerInput = getIOBoard()->createInput<VSLibOpenCV::InputCVMat>("detected markers");
   }
   connect(markerInput
         , SIGNAL(signalInputValueModified(VSDIO::Input*))
         , this
         , SLOT(slotMarkerInputValueModified(VSDIO::Input*)));

   if (getCameraExtension())
   {
      cameraExtension = getCameraExtension();
      connect(cameraExtension, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
         this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
   }

   // initialize
   if (!loadCameraConfigFromFile())
      buildCameraMatrix();

   dataMarkerEdgeLength.set(0.5);
   dataShowAxisVisu.set(true);
   dataShowBoxVisu.set(true);

   if (!visu)
   {
      visu = new BoxVisu(mySimState->getMyEnvironment()->getMyProject(), this);
      visu->initialize();
   }

   
   

}

void VSLibARMarker::ExtensionMarkerVisu::preWillBeRemovedFromElementContainer()
{
   //call base class method
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();
}

void VSLibARMarker::ExtensionMarkerVisu::slotMyPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   if(getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;

   VSLibARMarker::ExtensionMarkerVisu* thisInstance = simStateInstance->instanceCast<VSLibARMarker::ExtensionMarkerVisu*>();
   if (thisInstance)
   {
      const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
      VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
      if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("cameraExtension"))
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

void VSLibARMarker::ExtensionMarkerVisu::slotInputValueModified(VSDIO::Input* input)
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
      //do something!
      return;
   }
}


void VSLibARMarker::ExtensionMarkerVisu::slotMarkerInputValueModified(VSDIO::Input* input)
{
   if (!getInputValueExecute())
   {
      return;
   }
   //if (working)
   //   return;
   QVariant myValue = input->getVariant();
   cv::Mat markerMatrix = myValue.value<VSLibOpenCV::Mat>();
   MarkerDetections markers;
   markers.fromMatrix(markerMatrix);
   if (markers.empty())
   {
      visibleMarkers = MarkerDetections();
      return;
   }

   if (getMarkerId() >= 0)
   {

      MarkerDetections myMarker;
      for (int i = 0; i < markers.size(); ++i)
      {
         if (markers.ids.at(i) == getMarkerId())
         {
            myMarker.append(markers.at(i));
            break;
         }
         markers = myMarker;
      }
      if (markers.empty())
      {
         visibleMarkers = MarkerDetections();
         return;
      }
   }
   markers.estimateMarkerPoses(getMarkerEdgeLength(), cameraMatrix, distCoeffs);

   visibleMarkers = markers;
      
   if (!getCameraExtension())
   {
      visibleMarkers = MarkerDetections();
      return;
   }
   if (!getNode())
   {
      visibleMarkers = MarkerDetections();
      return;
   }
   VSD3D::Node* node = getNode();
   VSD3D::Node* cameraNode = getCameraExtension()->getParentVSD3DNode();
   if (!cameraNode)
   {
      visibleMarkers = MarkerDetections();
      return;
   }
         
   if (getMoveCamera())
   {
      VSM::Frame mwf = node->compileUpdatedWorldFrame();
      VSM::Frame cwf = Tools::getCameraWorldFrame(visibleMarkers.tvecs.front(), visibleMarkers.rvecs.front(), mwf);
      cameraNode->setWorldFrame(cwf);
   }
   else
   {
      VSM::Frame cwf = cameraNode->compileUpdatedWorldFrame();
      VSM::Frame mwf = Tools::getMarkerWorldFrame(visibleMarkers.tvecs.front(), visibleMarkers.rvecs.front(), cwf);
      node->setWorldFrame(mwf);
   }
}

void VSLibARMarker::ExtensionMarkerVisu::slotCameraPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   Q_UNUSED(simStateInstance);
   Q_UNUSED(metaProperty);
   if (!loadCameraConfigFromFile())
      buildCameraMatrix();
}

void VSLibARMarker::ExtensionMarkerVisu::buildCameraMatrix()
{
   if (getCameraExtension() && getMarkerEdgeLength() > 0)
   {
      double fov = getCameraExtension()->getXAngle();
      int width = getCameraExtension()->getImageWidth();
      int height = getCameraExtension()->getImageHeight();
      double f = VSLibImageProcessing::CalibrationTools::fovToFocalLength(fov)*width;
      double principleX = width / 2;
      double principleY = height / 2;
      cameraMatrix = (cv::Mat_<float>(3, 3) << f, 0, principleX, 0, f, principleY, 0, 0, 1);
      distCoeffs = cv::Mat_<float>::zeros(1, 5);
   }
   else
   {
      cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
      distCoeffs = cv::Mat_<float>::zeros(1, 5);
   }
}

bool VSLibARMarker::ExtensionMarkerVisu::loadCameraConfigFromFile()
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

VSLibARMarker::MarkerDetections& VSLibARMarker::ExtensionMarkerVisu::getVisibleMarkers()
{
   if (!visibleMarkers.empty() && visibleMarkers.tvecs.empty())
      visibleMarkers.estimateMarkerPoses(getMarkerEdgeLength(), cameraMatrix, distCoeffs);
   return visibleMarkers;
}

bool VSLibARMarker::ExtensionMarkerVisu::isActive()
{
   return getInputValueExecute();
}
