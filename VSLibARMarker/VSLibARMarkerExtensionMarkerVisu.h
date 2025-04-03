#ifndef VSLibARMarkerExtensionMarkerVisuH
#define VSLibARMarkerExtensionMarkerVisuH

//base class
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

#include "../VSLibSensor/VSLibSensorCameraExtensionBase.h"

#include <opencv2/aruco.hpp>

#include "VSLibARMarkerBoxVisu.h"
#include "VSLibARMarkerDatatypes.h"
#include "VSLibARMarkerExport.h"

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
   class VSLibARMarker_DECLSPEC ExtensionMarkerVisu : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("AR marker detection visu")
         ICON ":/VSLibARMarker/icons/landmark.png"
      );

   protected:
      ExtensionMarkerVisu(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~ExtensionMarkerVisu();

      VSDIO::ExtensionIOBoard* getIOBoard();
      void postHasBeenAddedToElementContainer();
      void preWillBeRemovedFromElementContainer();

   public:
      bool isActive();
      VSLibARMarker::MarkerDetections& getVisibleMarkers();

   protected slots:
      void slotInputValueModified(VSDIO::Input* input);
      void slotMarkerInputValueModified(VSDIO::Input* input);
      void slotMyPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);
      void slotCameraPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

   protected:
      void buildCameraMatrix();
      bool loadCameraConfigFromFile();
      //void buildCameraMatrix();

      //data
      VSD::Ref<VSDIO::ExtensionIOBoard> ioBoard;
      //VSM::Frame sensorFrameRelToBaseNode;
      VSLibSensor::CameraExtensionBase* cameraExtension;
      //cv::Mat cameraMatrix;
      cv::Mat cameraMatrix;
      cv::Mat distCoeffs;
      BoxVisu* visu;
      VSD::SimState* mySimState;
      VSLibARMarker::MarkerDetections visibleMarkers;
      //bool working;

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

      VSD_PROPERTY_VAL(int markerId
         DATA getDataMarkerId
         READ getMarkerId
         WRITE setMarkerId
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         DEFAULT -1
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, MarkerId);
      
      VSD_PROPERTY_VAL(bool moveCamera
         DATA getDataMoveCamera
         READ getMoveCamera
         WRITE setMoveCamera
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, MoveCamera);

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

   }; // class ExtensionMarkerVisu
}; // namespace VSLibARMarker

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::ExtensionMarkerVisu);

#endif //VSLibARMarkerExtensionMarkerVisuH
