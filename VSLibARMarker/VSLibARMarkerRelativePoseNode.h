#ifndef VSLibARMarkerRelativePoseNodeH
#define VSLibARMarkerRelativePoseNodeH

// base class
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibSensor/VSLibSensorCameraExtensionBase.h"
#include "VSLibARMarkerDatatypes.h"

namespace VSLibARMarker
{
   class RelativePoseNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("AR marker relative pose node")
         ICON ":/VSLibARMarker/icons/ardetect.png"
      );

   protected:
      RelativePoseNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~RelativePoseNode();

   // management
   private:
      // plugin base
      VSDIO::ExtensionIOBoard* getIOBoard();
      void postHasBeenAddedToElementContainer();
      void preWillBeRemovedFromElementContainer();

   protected slots:
      // slot functions
      void slotEnableValueModified(VSDIO::Input* input);
	   void slotDetectedMarkersValueModified(VSDIO::Input* input);
      void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);
		void slotCameraPropertyModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

   // data
   private:
      // class vars
      cv::Mat cameraMatrix_;
      MarkerDetections detectedMarkers_;
      VSLibSensor::CameraExtensionBase* cameraExtension_;
      cv::Mat distCoeffs_;
      VSM::Frame sensorFrameRelToBaseFrame_;

      // class functions
      void buildCameraMatrix();
      bool loadCameraConfigFromFile();
      
   protected:
      // properties: camera
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

	   VSD_PROPERTY_VAL(double markerEdgeLength
         DATA getDataMarkerEdgeLength
         READ getMarkerEdgeLength
         WRITE setMarkerEdgeLength
			DEFAULT 0.3
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Meter);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, MarkerEdgeLength);

      // properties: to-base transform
      VSD_PROPERTY_VAL(bool enableToBaseTransform
         DATA getDataEnableToBaseTransform
         READ getEnableToBaseTransform
         WRITE setEnableToBaseTransform
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, EnableToBaseTransform);

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

		// filter properties
		VSD_PROPERTY_VAL(int edgeLengthThreshold
			DATA getDataEdgeLengthThreshold
			READ getEdgeLengthThreshold
			WRITE setEdgeLengthThreshold
			DEFAULT 30
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, EdgeLengthThreshold);

		VSD_PROPERTY_VAL(int minEdgeLengthDiff
			DATA getDataMinEdgeLengthDiff
			READ getMinEdgeLengthDiff
			WRITE setMinEdgeLengthDiff
			DEFAULT 10
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, MinEdgeLengthDiff);

   }; // class RelativePoseNode
}; // namespace VSLibARMarker

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::RelativePoseNode);

#endif //VSLibARMarkerRelativePoseNodeH
