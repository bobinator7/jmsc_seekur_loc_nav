#ifndef VSPluginMarkerLocalizationSensorMergeNodeH
#define VSPluginMarkerLocalizationSensorMergeNodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibSensor/VSLibSensorSensorExtensionBase.h"

#include "Lib/VSM/VSMPoseVector3Quaternion.h"

#include <future>
#include <unordered_map>

namespace VSPluginMarkerLocalization
{
   class MarkerEKFSensorMergeNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("MarkerEKFSensorMergeNode")
      );

   // construction
   protected:
	   MarkerEKFSensorMergeNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~MarkerEKFSensorMergeNode();

   // management
   private:
      // plugin base
      /// \fn virtual VSDIO::ExtensionIOBoard* getIOBoard()
      ///
      /// \brief Allows access to the IO-Board.
      virtual VSDIO::ExtensionIOBoard* getIOBoard();

      /// \fn virtual void postHasBeenAddedToElementContainer()
      ///
      /// \brief Will be called once the SensorDataProcessingNodeBase has been added to the element container. 
      virtual void postHasBeenAddedToElementContainer();

      /// \fn virtual void preWillBeRemovedFromElementContainer()
      ///
      /// \brief Will be called before the SensorDataProcessingNodeBase will be removed from the element container. 
      virtual void preWillBeRemovedFromElementContainer();

      bool propertyCheck();

   protected slots:
      // slot functions
      void slotMyInputValueModified(VSDIO::Input* input);
      void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

      void slotTimedOutput();

      void slotPropagateDirtyFlag() {};
      void slotPropagateChangeSignal() {};

   //data
   private:
      int num_of_input_slots_;
      std::unordered_map<int, std::unordered_map<int, VSM::PoseVector3Quaternion>> marker_maps_;

      QTimer *timer_;

      std::mutex m_lowprio_, m_next_, m_data_;

      

      /// \fn void doSomethingWithSensorData(const QImage& sensorData)
      ///
      /// \brief example function for processing sensordata (\ref sensorData). After data processing the function \ref void copyDataOnOutput(const QImage& processedSensorData) will be called.
      ///
      /// \param  sensorData  The input image to be processed.
      void updateSensorData(VSDIO::Input* input);

      /// \fn void copyDataOnOutput(const QImage& processedSensorData)
      ///
      /// \brief Will copy the processed sensor data (\ref sensorData) to the dedicated output of the IO-Board.
      ///
      /// \param  processedSensorData  The image that will be copied to the output.
      void copyDataOnOutput(const VSM::MatrixNxM& marker_matrix_out);

   protected:

   public:

      /// \property intNumberOfCameras
      ///
      /// \brief Property - number of input cameras
      VSD_PROPERTY_VAL(int intNumberOfCameras
         DATA getDataIntNumberOfCameras
         READ getIntNumberOfCameras
         WRITE setIntNumberOfCameras
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Undefined);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, IntNumberOfCameras);

      /// \property int intPropertyOutputFrequency
      ///
      /// \brief Property - frequency of output thread signaling merged sensor data
      VSD_PROPERTY_VAL(int intPropertyOutputFrequency
         DATA getDataIntPropertyOutputFrequency
         READ getIntPropertyOutputFrequency
         WRITE setIntPropertyOutputFrequency
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Hertz);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, IntPropertyOutputFrequency);

		/// \property int intPropertyOutputFrequency
		///
		/// \brief Property - frequency of output thread signaling merged sensor data
		VSD_PROPERTY_VAL(bool bool2DMarkerOutput
			DATA getDataBool2DMarkerOutput
			READ getBool2DMarkerOutput
			WRITE setBool2DMarkerOutput
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, Bool2DMarkerOutput);
   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMarkerLocalization::MarkerEKFSensorMergeNode);

#endif //VSPluginMarkerLocalizationSensorMergeNodeH
