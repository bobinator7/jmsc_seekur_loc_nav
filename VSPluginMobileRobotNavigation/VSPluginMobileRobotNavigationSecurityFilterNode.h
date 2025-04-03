#ifndef VSPluginMobileRobotNavigationSecurityFilterNodeH
#define VSPluginMobileRobotNavigationSecurityFilterNodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibMarkerLocalization/VSLibMarkerLocalizationGridmap.h"
#include "../VSLibMobileRobotNavigation/VSLibMobileRobotNavigationCollavDWA.h"
#include "../VSLibSensor/VSLibSensorLaserScanner2DExtensionBase.h"

#include <QFutureWatcher>

namespace VSPluginMobileRobotNavigation
{
   class SecurityFilterNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("SecurityFilterNode")
      );

   // construction
   protected:
      SecurityFilterNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~SecurityFilterNode();

   //data
   public:
		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double safetyWidth
			DATA getDataSafetyWidth
			READ getSafetyWidth
			WRITE setSafetyWidth
			UNIT VSD::Unit::Meter
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, SafetyWidth);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double safetyLength
			DATA getDataSafetyLength
			READ getSafetyLength
			WRITE setSafetyLength
			UNIT VSD::Unit::Meter
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, SafetyLength);

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

		VSD_PROPERTY_REF(VSLibSensor::LaserScanner2DExtensionBase laserExtension
			DATA getDataLaserExtension
			READ getLaserExtension
			WRITE setLaserExtension
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_REF_INTERNAL(VSLibSensor::LaserScanner2DExtensionBase, LaserExtension);

   // management
   private:
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

   protected slots:
      // override base function and do nothing
      void slotPropagateDirtyFlag(){};
      void slotPropagateChangeSignal(){};

		// here is the simulation
		//void slotMyInputValueModified(VSDIO::Input* input);
		void slotCheckValidityFinished();
		void slotEnableModified(VSDIO::Input* input);
		void slotLaserscanValueModified(VSDIO::Input* input);
		void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

   private:
		// laser transform
		unsigned int indexRange_;
		std::vector<double> indexAngle_;
		VSM::Frame sensorToBaseTransform_;

		bool validLaserscanInRobotFrameFromRange(std::vector<double> rangeData);
		void setLaserscanConversionData();

		// qtconcurrent
		bool busy_;
		bool valid_;
		void checkValidInThread(std::vector<double> rangeVector);
		QFutureWatcher<void> *resultWatcher;

		// test if properties are valid
		bool propertyCheck();

   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMobileRobotNavigation::SecurityFilterNode);

#endif //VSPluginMobileRobotNavigationSecurityFilterNodeH
