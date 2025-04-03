#ifndef VSPluginMarkerLocalizationMappingNodeH
#define VSPluginMarkerLocalizationMappingNodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibMarkerLocalization/VSLibMarkerLocalizationGridmap.h"
#include "../VSLibSensor/VSLibSensorLaserScanner2DExtensionBase.h"
#include "../VSLibSensor/VSLibSensorSensorExtensionBase.h"


#include "Lib/VSM/VSMMatrix3x3.h"
#include "Lib/VSM/VSMVector3.h"



namespace VSPluginMarkerLocalization
{
   class MappingNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("MappingNode")
      );

   // construction
   protected:
	   MappingNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~MappingNode();
	     
   // management
   private:
	  // plugin base
     virtual VSDIO::ExtensionIOBoard* getIOBoard();
     virtual void postHasBeenAddedToElementContainer();
     virtual void preWillBeRemovedFromElementContainer();
	  
	  bool propertyCheck();

   protected slots:
      // slot functions
		void slotEnableModified(VSDIO::Input* input);
		void slotInputModified(VSDIO::Input* input);
      void slotLaserscanValueModified(VSDIO::Input* input);
		void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

      void slotPropagateDirtyFlag(){};
      void slotPropagateChangeSignal(){};
		

   //data
	public:

   private:
		bool loadImageFromFile();
		int imgWidth_, imgHeight_;
		int imgMaxGreyValue_;
		std::vector<unsigned char> imgRawData_;

		void setStaticGridmap();
		VSLibMarkerLocalization::Gridmap staticGridmap_;

		unsigned int indexRange_;
		std::vector<double> indexAngle_;
		void setLaserscanConversionData();

		double lastUpdateTime_;


   protected:
         
   public:
   
      // properties
 
      /// \property bool boolEnableVisualization
      ///
      /// \brief enables visualization of ukf poses and covariances
      VSD_PROPERTY_VAL(bool boolEnableVisualization
         DATA getDataBoolEnableVisualization
         READ getBoolEnableVisualization
         WRITE setBoolEnableVisualization
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolEnableVisualization);

		/// \property bool boolUseSimTime
		///
		/// \brief enables switch to using simulation time
		VSD_PROPERTY_VAL(bool boolUseSimTime
			DATA getDataBoolUseSimTime
			READ getBoolUseSimTime
			WRITE setBoolUseSimTime
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolUseSimTime);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double updateRate
			DATA getDataUpdateRate
			READ getUpdateRate
			WRITE setUpdateRate
			UNIT VSD::Unit::Hertz
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, UpdateRate);

      /// \property initial pose vector
      ///
      /// \brief initial pose in x,y,theta
      VSD_PROPERTY_VAL(VSM::Vector3 robotPose 
         UNIT VSD::Unit::Meter
         DATA getDataRobotPose READ getRobotPose WRITE setRobotPose
         SAVE true EDIT_VIEW true
         EDIT_MODIFY false CLONE true SYNC_LOCAL_VIA_PROPERTY true SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Vector3, RobotPose);

		// 
		VSD_PROPERTY_VAL(QUrl staticMarkerMapFile
			DATA getDataStaticMarkerMapFile
			READ getStaticMarkerMapFile
			WRITE setStaticMarkerMapFile
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(QUrl, StaticMarkerMapFile);

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
   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMarkerLocalization::MappingNode);

#endif //VSPluginMarkerLocalizationUKFNodeH
