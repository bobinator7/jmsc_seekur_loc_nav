#ifndef VSPluginMobileRobotNavigationFrame2Vector3NodeH
#define VSPluginMobileRobotNavigationFrame2Vector3NodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibSensor/VSLibSensorSensorExtensionBase.h"

namespace VSPluginMobileRobotNavigation
{
   class Frame2Vector3Node : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("Frame2Vector3Node")
      );

   // construction
   protected:
	   Frame2Vector3Node(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~Frame2Vector3Node();

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

   public:

   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMobileRobotNavigation::Frame2Vector3Node);

#endif //VSPluginMobileRobotNavigationNodeH
