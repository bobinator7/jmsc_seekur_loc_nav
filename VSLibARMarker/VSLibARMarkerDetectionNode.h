#ifndef VSLibARMarkerDetectionNodeH
#define VSLibARMarkerDetectionNodeH

//base class
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

#include "../VSLibSensor/VSLibSensorCameraExtensionBase.h"
#include "../VSLibVisualGPS/VSLibVisualGPSNavMapNode.h"
#include "Lib/VSD/VSDModelInstanceExtension.h"
#include "Lib/VSD/VSDProperty.h"

#include <opencv2/aruco.hpp>

#include "VSLibARMarkerBoxVisu.h"
#include "VSLibARMarkerDatatypes.h"
#include "VSLibARMarkerExport.h"
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
   class VSLibARMarker_DECLSPEC DetectionNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("AR marker detection node")
         ICON ":/VSLibARMarker/icons/ardetect.png"
      );

   protected:
      DetectionNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~DetectionNode();

      VSDIO::ExtensionIOBoard* getIOBoard();
      void postHasBeenAddedToElementContainer();
      void preWillBeRemovedFromElementContainer();

   public:
      bool isActive();

   protected slots:
      void slotInputValueModified(VSDIO::Input* input);
      void slotImageInputValueModified(VSDIO::Input* input);
      void slotMyPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);
      void slotDetectionFinished();

   protected:
      void detectInThread(const cv::Mat& image, cv::_OutputArray& corners, cv::_OutputArray& ids);

      //data
      VSD::Ref<VSDIO::ExtensionIOBoard> ioBoard;
      //VSLibSensor::CameraExtensionBase* cameraExtension;

      VSD::SimState* mySimState;
      bool working;
      cv::Mat currentImage;
      MarkerDetections currentMarkers;
      QFutureWatcher<void> *resultWatcher;

      //VSD_PROPERTY_VAL(double markerEdgeLength
      //   DATA getDataMarkerEdgeLength
      //   READ getMarkerEdgeLength
      //   WRITE setMarkerEdgeLength
      //   SAVE true
      //   EDIT_VIEW true
      //   EDIT_MODIFY true
      //   CLONE true
      //   SYNC_LOCAL_VIA_PROPERTY true
      //   SYNC_DISTRIBUTED_VIA_INSTANCE true
      //   UNIT VSD::Unit::Meter);
      //VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, MarkerEdgeLength);

      //VSD_PROPERTY_REF(VSLibSensor::CameraExtensionBase cameraExtension
      //   DATA getDataCameraExtension
      //   READ getCameraExtension
      //   WRITE setCameraExtension
      //   SAVE true
      //   EDIT_VIEW true
      //   EDIT_MODIFY true
      //   CLONE true
      //   SYNC_LOCAL_VIA_PROPERTY true
      //   SYNC_DISTRIBUTED_VIA_INSTANCE true);
      //VSD_PROPERTY_TEMPLATE_REF_INTERNAL(VSLibSensor::CameraExtensionBase, CameraExtension);

      //VSD_PROPERTY_VAL(bool mirrorInputStream
      //   DATA getDataMirrorInputStream
      //   READ getMirrorInputStream
      //   WRITE setMirrorInputStream
      //   SAVE true
      //   EDIT_VIEW true
      //   EDIT_MODIFY true
      //   CLONE true
      //   SYNC_LOCAL_VIA_PROPERTY true
      //   SYNC_DISTRIBUTED_VIA_INSTANCE true);
      //VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, MirrorInputStream);

      VSD_PROPERTY_VAL(int currentDetectionsCount
         DATA getDataCurrentDetectionsCount
         READ getCurrentDetectionsCount
         WRITE setCurrentDetectionsCount
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY false
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, CurrentDetectionsCount);

   }; // class DetectionNode
}; // namespace VSLibARMarker

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::DetectionNode);

#endif //VSLibARMarkerDetectionNodeH
