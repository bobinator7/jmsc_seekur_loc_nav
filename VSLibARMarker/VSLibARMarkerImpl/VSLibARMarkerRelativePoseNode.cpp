// header
#include "../VSLibARMarkerRelativePoseNode.h"

// specific
#include "Lib/VSDIO/VSDIOExtensionIOBoard.h"
#include "Lib/VSDIO/VSDIOInputBool.h"
#include "Lib/VSDIO/VSDIOOutputVSMMatrixNxM.h"
#include "Lib/VSDTools/VSDTools.h"
#include "Lib/VSLibImageProcessing/VSLibImageProcessingCalibrationTools.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVCommon.h"
#include "Plugin/VSLibOpenCV/VSLibOpenCVInputCVMat.h"

#include <opencv2/calib3d.hpp>
#include <QFile>

VSLibARMarker::RelativePoseNode::RelativePoseNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataBaseNode(this)
, dataCameraCalibrationFile(this)
, dataCameraExtension(this)
, dataEnableToBaseTransform(this)
, dataEdgeLengthThreshold(this)
, dataMarkerEdgeLength(this)
, dataMinEdgeLengthDiff(this)
, cameraExtension_(0)
{
   cv::redirectError(VSLibOpenCV::Common::OpenCVErrorCallback);
}

VSLibARMarker::RelativePoseNode::~RelativePoseNode()
{
}

VSDIO::ExtensionIOBoard* VSLibARMarker::RelativePoseNode::getIOBoard()
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

void VSLibARMarker::RelativePoseNode::postHasBeenAddedToElementContainer()
{
   // call base class method
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   // property modified functions
   connect(this
         , SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*))
         , this
         , SLOT(slotPropertiesModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));

   // reference modified
   if (getCameraExtension())
   {
      cameraExtension_ = getCameraExtension();
      connect(cameraExtension_, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
         this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
   }
   else if (getBaseNode())
   {

   }

   /** IO-Board (start) **/
   ioBoard = getIOBoard();
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
	   return;
   if (!ioBoard)
	   return;
   
   VSDIO::Input* inputEnable = ioBoard->findInputByName("enable");
   connect(inputEnable
	   , SIGNAL(signalInputValueModified(VSDIO::Input*))
	   , this
	   , SLOT(slotEnableValueModified(VSDIO::Input*)));

   VSDIO::Input* detectedMarkers = ioBoard->findInputByName("detectedMarkersIn");
   if (!detectedMarkers)
   {
	   detectedMarkers = getIOBoard()->createInput<VSLibOpenCV::InputCVMat>("detectedMarkersIn");
   }
   connect(detectedMarkers
	   , SIGNAL(signalInputValueModified(VSDIO::Input*))
	   , this
	   , SLOT(slotDetectedMarkersValueModified(VSDIO::Input*)));
   
   VSDIO::Output* detectedMarkersOutput = ioBoard->findOutputByName("relMarkerPosesOut");
   if (!detectedMarkersOutput)
   {
      detectedMarkersOutput = getIOBoard()->createOutput<VSDIO::OutputVSMMatrixNxM>("relMarkerPosesOut");
   }

   /** IO-Board (end) **/

   // initialize parameters
   dataEnableToBaseTransform.set(true);
   buildCameraMatrix();
   sensorFrameRelToBaseFrame_.setIdentity();

   // replace in real world example with static frame
	if (getBaseNode() != nullptr && getCameraExtension() != nullptr)
		sensorFrameRelToBaseFrame_ = getBaseNode()->compileUpdatedWorldFrame().getInverse() * getCameraExtension()->getParentVSD3DNode()->compileUpdatedWorldFrame();
}

void VSLibARMarker::RelativePoseNode::preWillBeRemovedFromElementContainer()
{
   //call base class method
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();

}

void VSLibARMarker::RelativePoseNode::slotCameraPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
	if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
		return;

	VSLibSensor::CameraExtensionBase* thisInstance = simStateInstance->instanceCast<VSLibSensor::CameraExtensionBase*>();
	if (thisInstance)
	{
		buildCameraMatrix();
	}
}


void VSLibARMarker::RelativePoseNode::slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty)
{
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;

   VSLibARMarker::RelativePoseNode* thisInstance = simStateInstance->instanceCast<VSLibARMarker::RelativePoseNode*>();
   if (thisInstance)
   {
      const VSD::MetaInstance* thisMetaInstance = thisInstance->getMetaInstance();
      VSL_ASSERT_ELSE_RETURN(thisMetaInstance);
      if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("cameraExtension"))
      {
         if (getCameraExtension() != cameraExtension_)
         {
            if (cameraExtension_)
            {
               disconnect(cameraExtension_, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
                  this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
               cameraExtension_ = 0;
            }
            if (getCameraExtension())
            {
               cameraExtension_ = getCameraExtension();
               connect(cameraExtension_, SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)),
                  this, SLOT(slotCameraPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
            }
            if (!loadCameraConfigFromFile())
            {
               buildCameraMatrix();
            }
         }

			if (getBaseNode() != nullptr && getCameraExtension() != nullptr)
				sensorFrameRelToBaseFrame_ = getBaseNode()->compileUpdatedWorldFrame().getInverse() * getCameraExtension()->getParentVSD3DNode()->compileUpdatedWorldFrame();
			else
				sensorFrameRelToBaseFrame_.setIdentity();

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
      else if (metaProperty == thisMetaInstance->findMyOrBaseClassProperty("baseNode"))
      {
			if (getBaseNode() != nullptr && getCameraExtension() != nullptr)
				sensorFrameRelToBaseFrame_ = getBaseNode()->compileUpdatedWorldFrame().getInverse() * getCameraExtension()->getParentVSD3DNode()->compileUpdatedWorldFrame();
			else
				sensorFrameRelToBaseFrame_.setIdentity();
      }
   }
}

void VSLibARMarker::RelativePoseNode::slotEnableValueModified(VSDIO::Input* input)
{
	if (!getInputValueExecute())
		return;
}

void VSLibARMarker::RelativePoseNode::slotDetectedMarkersValueModified(VSDIO::Input* input)
{
   // return if node is not enabled
   if (!getInputValueExecute())
      return;

   // get input
   if (!input->inherits<VSLibOpenCV::InputCVMat*>())
      return;
   QVariant variantData = input->getVariant();
   cv::Mat marker_cvmatrix = variantData.value<VSLibOpenCV::Mat>();

   // convert input into MarkerDetections format
   detectedMarkers_.fromMatrix(marker_cvmatrix);
   if (detectedMarkers_.empty())
   {
      detectedMarkers_ = MarkerDetections();
      return;
   }

   // create tvecs and rvecs in MarkerDetections class
   detectedMarkers_.estimateMarkerPoses(getMarkerEdgeLength(), cameraMatrix_, distCoeffs_);
	// unstable marker detections
	detectedMarkers_.filterDetections(getEdgeLengthThreshold(),getMinEdgeLengthDiff());

   // TODO: move to subfunction
   // TODO: check page hits and misses due to 2d indexing
   // OPT: check if num_detected_markers == num_markers

   int num_detected_marker = detectedMarkers_.size();
   std::vector<VSM::Vector4> columns;
   for (int ii = 0; ii < num_detected_marker; ii++)
   {
      int cursor_id = detectedMarkers_.at(ii).getId();
      cv::Vec3d cursor_tvec = detectedMarkers_.at(ii).getTvec();
      cv::Vec3d cursor_rvec = detectedMarkers_.at(ii).getRvec();

      // convert relative poses in frames
      cv::Mat rotation_matrix;
      cv::Rodrigues(cursor_rvec, rotation_matrix);
      VSM::Matrix3x3 orientation = VSLibOpenCV::Common::vsmMatrix3x3FromCVMat(rotation_matrix);
      VSM::Vector3 position = VSLibOpenCV::Common::vsmVec3FromCVVec3d(cursor_tvec);
      VSM::Quaternion quat(orientation);
      VSM::Frame markerFrameRelToSensorFrame(orientation,position);

      // convert opencv coord sys into verosim coord
      markerFrameRelToSensorFrame = VSM::Frame::fromRotationX(-3.141592 / 2) * markerFrameRelToSensorFrame;
      position = markerFrameRelToSensorFrame.getPosition();
      orientation = markerFrameRelToSensorFrame.getOrientation();

		// unstable marker id
      if (cursor_id == 1023)
         continue;
      
      if (getEnableToBaseTransform())
      {
         QString str = getName();
         std::string sstr = str.toStdString();
         
         VSM::Frame markerFrameRelToBaseFrame = sensorFrameRelToBaseFrame_ * markerFrameRelToSensorFrame;

         position = markerFrameRelToBaseFrame.getPosition();
         orientation = markerFrameRelToBaseFrame.getOrientation();
         quat = VSM::Quaternion(orientation);
      }

      // convert relative poses in matrix format containing columns of form (id,x,y,theta) in Robot coords
		VSM::Vector3 vecZ(0, 0, 1);
		VSM::Vector3 vecZProjected = orientation * vecZ;
		double theta = std::atan2(vecZProjected.getY(), vecZProjected.getX());

		VSM::Vector4 cursor_column(cursor_id, position[0], position[1], theta);

      //VSM::Vector4 quat_v4 = quat.getVector4();
      //VSM::VectorN cursor_column = VSM::VectorN(8);
      //cursor_column.setElement(0, cursor_id);
      //cursor_column.setElement(1, position[0]);
      //cursor_column.setElement(2, position[1]);
      //cursor_column.setElement(3, position[2]);
      //cursor_column.setElement(4, quat_v4[0]);
      //cursor_column.setElement(5, quat_v4[1]);
      //cursor_column.setElement(6, quat_v4[2]);
      //cursor_column.setElement(7, quat_v4[3]);

      columns.push_back(cursor_column);
      
   }

   if (columns.size() <= 0)
      return;

   VSM::MatrixNxM markerposes_vsm(4, columns.size());
   for (int jj = 0; jj < columns.size(); jj++)
   {
      markerposes_vsm.setColumn(jj, columns[jj]);
   }

   // set output values
   VSDIO::Output* output = getIOBoard()->findOutputByName("relMarkerPosesOut");
   if (output)
   {
      QVariant outputValue = QVariant::fromValue(markerposes_vsm);
      output->setLocalVariant(outputValue);
   }
}

/* Camera config (begin) */
bool VSLibARMarker::RelativePoseNode::loadCameraConfigFromFile()
{
   if (getCameraCalibrationFile().isEmpty())
      return false;
   QString filename = VSDTools::compileAbsoluteUrl(getMyModel(), getCameraCalibrationFile()).toLocalFile();
   QFile configFile(filename);
   if (!configFile.exists())
      return false;
   configFile.open(QFile::ReadOnly);
   QTextStream stream(&configFile);
   cameraMatrix_ = cv::Mat_<float>::zeros(3, 3);
   distCoeffs_ = cv::Mat_<float>::zeros(1, 5);
   char ch = '\0';
   for (int i = 0; i<13; ++i) // filter out string: "cameraMatrix("
   {
      stream >> ch;
   }
   for (int r = 0; r<3; ++r)
   {
      for (int c = 0; c<3; ++c)
      {
         stream >> cameraMatrix_.ptr<float>(r)[c] >> ch;
      }
   }
   stream >> ch; // filter out char: '\n'
   for (int i = 0; i<11; ++i) // filter out string: "distCoeffs("
   {
      stream >> ch;
   }
   for (int c = 0; c<5; ++c)
   {
      stream >> distCoeffs_.ptr<float>(0)[c] >> ch;
   }
   return true;
}

void VSLibARMarker::RelativePoseNode::buildCameraMatrix()
{
   if (getCameraExtension() && getMarkerEdgeLength() > 0)
   {
      double fov = getCameraExtension()->getXAngle();
      int width = getCameraExtension()->getImageWidth();
      int height = getCameraExtension()->getImageHeight();
      double f = VSLibImageProcessing::CalibrationTools::fovToFocalLength(fov)*width;
      double principleX = width / 2;
      double principleY = height / 2;
      cameraMatrix_ = (cv::Mat_<float>(3, 3) << f, 0, principleX, 0, f, principleY, 0, 0, 1);
      distCoeffs_ = cv::Mat_<float>::zeros(1, 5);
   }
   else
   {
      cameraMatrix_ = cv::Mat::eye(3, 3, CV_32FC1);
      distCoeffs_ = cv::Mat_<float>::zeros(1, 5);
   }
}

/* Camera config (end) */
