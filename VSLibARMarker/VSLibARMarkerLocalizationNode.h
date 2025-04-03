#ifndef VSLibARMarkerLocalizationNodeH
#define VSLibARMarkerLocalizationNodeH

//base class
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

#include "../VSLibSensor/VSLibSensorCameraExtensionBase.h"
#include "../VSLibVisualGPS/VSLibVisualGPSNavMapNode.h"
#include "Lib/VSD/VSDModelInstanceExtension.h"
#include "Lib/VSD/VSDProperty.h"

#include <opencv2/aruco.hpp>

#include "VSLibARMarkerBoxVisu.h"
#include "VSLibARMarkerExport.h"
#include "VSLibARMarkerDatatypes.h"
#include <QFutureWatcher>

namespace VSD
{
   class SimState;
}

namespace VSDIO
{
   class ExtensionIOBoard;
   class Input;
}

namespace VSLibARMarker
{
   class VSLibARMarker_DECLSPEC LocalizationNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("AR marker LocalizationNode node")
         ICON ":/VSLibARMarker/icons/ardetect.png"
      );

   protected:
      LocalizationNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~LocalizationNode();

      VSDIO::ExtensionIOBoard* getIOBoard();
      void postHasBeenAddedToElementContainer();
      void preWillBeRemovedFromElementContainer();

   public:
      MarkerDetections& getVisibleMarkers();
      bool isActive();

   protected slots:
      void slotInputValueModified(VSDIO::Input* input);
      void slotImageInputValueModified(VSDIO::Input* input);
      void slotMyPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);
      void slotCameraPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);
      void slotDetectionFinished();

   protected:
      bool loadCameraConfigFromFile();
      void buildCameraMatrix();
      //double focalLengthFromFieldOfView(double fov);
      void detectInThread(const cv::Mat& image, const double& markerEdgeLength, cv::_OutputArray& corners, cv::_OutputArray& ids);
      VSM::Frame estimateCameraPose();
      void appendNewMarkersToMap(QList<Marker> newMarkers, VSM::Frame cameraFrame);

      //data
      MarkerDetections currentMarkers;
      VSD::Ref<VSDIO::ExtensionIOBoard> ioBoard;
      VSM::Frame sensorFrameRelToBaseNode;
      VSLibSensor::CameraExtensionBase* cameraExtension;
      cv::Mat cameraMatrix;
      cv::Mat distCoeffs;
      BoxVisu* visu;
      VSD::SimState* mySimState;
      MarkerDetections visibleMarkers;
      QList<Marker> markersUnknown;
      bool working;
      VSLibOpenCV::Mat currentImage;
      QFutureWatcher<void> *resultWatcher;

      VSD_PROPERTY_VAL(double markerEdgeLength
         DATA getDataMarkerEdgeLength
         READ getMarkerEdgeLength
         WRITE setMarkerEdgeLength
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Meter);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, MarkerEdgeLength);

      VSD_PROPERTY_VAL(double maxLocalizationNodeDistance
         DATA getDataMaxLocalizationNodeDistance
         READ getMaxLocalizationNodeDistance
         WRITE setMaxLocalizationNodeDistance
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Meter);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, MaxLocalizationNodeDistance);

      VSD_PROPERTY_VAL(double minMarkerViewingAngle
         DATA getDataMinMarkerViewingAngle
         READ getMinMarkerViewingAngle
         WRITE setMinMarkerViewingAngle
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Radian);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, MinMarkerViewingAngle);

      VSD_PROPERTY_VAL(bool findAveragePose
         DATA getDataFindAveragePose
         READ getFindAveragePose
         WRITE setFindAveragePose
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, FindAveragePose);

      VSD_PROPERTY_VAL(bool appendUnknownMarkersToMap
         DATA getDataAppendUnknownMarkersToMap
         READ getAppendUnknownMarkersToMap
         WRITE setAppendUnknownMarkersToMap
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, AppendUnknownMarkersToMap);

      VSD_PROPERTY_VAL(int numberOfVisibleMarkers
         DATA getDataNumberOfVisibleMarkers
         READ getNumberOfVisibleMarkers
         WRITE setNumberOfVisibleMarkers
         SAVE false
         EDIT_VIEW true
         EDIT_MODIFY false
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, NumberOfVisibleMarkers);

      VSD_PROPERTY_VAL(QUrl cameraCalibrationFile
         DATA getDataCameraCalibrationFile
         READ getCameraCalibrationFile
         WRITE setCameraCalibrationFile
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(QUrl, CameraCalibrationFile);

      VSD_PROPERTY_VAL(bool showBoxVisu
         DATA getDataShowBoxVisu
         READ getShowBoxVisu
         WRITE setShowBoxVisu
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, ShowBoxVisu);

      VSD_PROPERTY_VAL(bool showAxisVisu
         DATA getDataShowAxisVisu
         READ getShowAxisVisu
         WRITE setShowAxisVisu
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, ShowAxisVisu);

      VSD_PROPERTY_REF(VSLibVisualGPS::NavMapNode map
         DATA getDataMap
         READ getMap
         WRITE setMap
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_REF_INTERNAL(VSLibVisualGPS::NavMapNode, Map);

      VSD_PROPERTY_REF(VSLibSensor::CameraExtensionBase cameraExtension
         DATA getDataCameraExtension
         READ getCameraExtension
         WRITE setCameraExtension
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_REF_INTERNAL(VSLibSensor::CameraExtensionBase, CameraExtension);

      VSD_PROPERTY_REF(VSD3D::Node node
         DATA getDataNode
         READ getNode
         WRITE setNode
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_REF_INTERNAL(VSD3D::Node, Node);

      VSD_PROPERTY_REF(VSD3D::Node baseNode
         DATA getDataBaseNode
         READ getBaseNode
         WRITE setBaseNode
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_REF_INTERNAL(VSD3D::Node, BaseNode);

   }; // class LocalizationNode
}; // namespace VSLibARMarker

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::LocalizationNode);

#endif //VSLibARMarkerLocalizationNodeH
