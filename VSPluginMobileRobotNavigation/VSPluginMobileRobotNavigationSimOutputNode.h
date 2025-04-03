#ifndef VSPluginMobileRobotNavigationSimOutputNodeH
#define VSPluginMobileRobotNavigationSimOutputNodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibSensor/VSLibSensorSensorExtensionBase.h"

namespace VSPluginMobileRobotNavigation
{
   class SimOutputNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("SimOutputNode")
      );

   // construction
   protected:
	   SimOutputNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~SimOutputNode();

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

   protected slots:
      // slot functions
      void slotMyInputValueModified(VSDIO::Input* input);
		void slotEnableModified(VSDIO::Input* input);
      //void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

      void slotPropagateDirtyFlag() {};
      void slotPropagateChangeSignal() {};

   //data
   private:

   protected:
		void outputData(double vl, double vr);

   public:
      /// \property double doublePropertyWheelDiameter
      ///
      /// \brief Property - wheel diameter for conversion from rad/s to m/s
      VSD_PROPERTY_VAL(double doublePropertyWheelDiameter
         DATA getDataDoublePropertyWheelDiameter
         READ getDoublePropertyWheelDiameter
         WRITE setDoublePropertyWheelDiameter
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::Meter);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, DoublePropertyWheelDiameter);

		/// \property double doublePropertyWheelDiameter
		///
		/// \brief Property - wheel diameter for conversion from rad/s to m/s
		VSD_PROPERTY_VAL(double doublePropertyWheelDistance
			DATA getDataDoublePropertyWheelDistance
			READ getDoublePropertyWheelDistance
			WRITE setDoublePropertyWheelDistance
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true
			UNIT VSD::Unit::Meter);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, DoublePropertyWheelDistance);
   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMobileRobotNavigation::SimOutputNode);

#endif //VSPluginMobileRobotNavigationNodeH
