#ifndef VSPluginMobileRobotNavigationCollisionAvoidanceNodeH
#define VSPluginMobileRobotNavigationCollisionAvoidanceNodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibMarkerLocalization/VSLibMarkerLocalizationGridmap.h"
#include "../VSLibMobileRobotNavigation/VSLibMobileRobotNavigationCollavDWA.h"
#include "../VSPluginMobileRobotNavigation/VSPluginMobileRobotNavigationNodeVisu.h"
#include "../VSLibSensor/VSLibSensorLaserScanner2DExtensionBase.h"

#include <QFutureWatcher>

namespace VSPluginMobileRobotNavigation
{
   class CollisionAvoidanceNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("CollisionAvoidanceNode")
      );

   // construction
   protected:
      CollisionAvoidanceNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~CollisionAvoidanceNode();

   //data
   public:
      /// \property 
      ///
      /// \brief 
      VSD_PROPERTY_VAL(VSM::Vector2 transAccelerationRange
         DATA getDataTransAccelerationRange
         READ getTransAccelerationRange
         WRITE setTransAccelerationRange
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         UNIT VSD::Unit::MeterPerSecondSquared);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Vector2, TransAccelerationRange);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(VSM::Vector2 rotAccelerationRange
			DATA getDataRotAccelerationRange
			READ getRotAccelerationRange
			WRITE setRotAccelerationRange
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true
			UNIT VSD::Unit::RadianPerSecondSquared);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Vector2, RotAccelerationRange);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(VSM::Vector2 transVelocityRange
			DATA getDataTransVelocityRange
			READ getTransVelocityRange
			WRITE setTransVelocityRange
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true
			UNIT VSD::Unit::MeterPerSecond);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Vector2, TransVelocityRange);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(VSM::Vector2 rotVelocityRange
			DATA getDataRotVelocityRange
			READ getRotVelocityRange
			WRITE setRotVelocityRange
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true
			UNIT VSD::Unit::RadianPerSecond);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Vector2, RotVelocityRange);

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

		// properties
		/// \property bool boolUseSimTime
		///
		/// \brief use simulation time instead of real time
		VSD_PROPERTY_VAL(bool boolUseSimTime
			DATA getDataBoolUseSimTime
			READ getBoolUseSimTime
			WRITE setBoolUseSimTime
			DEFAULT false
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolUseSimTime);

		/// \property bool boolUseSimTime
		///
		/// \brief use simulation time instead of real time
		VSD_PROPERTY_VAL(bool boolUseMapPoints
			DATA getDataBoolUseMapPoints
			READ getBoolUseMapPoints
			WRITE setBoolUseMapPoints
			DEFAULT false
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolUseMapPoints);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double updateControlRate
			DATA getDataUpdateControlRate
			READ getUpdateControlRate
			WRITE setUpdateControlRate
			UNIT VSD::Unit::Hertz
			DEFAULT 5.0
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, UpdateControlRate);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double doubleSigma
			DATA getDataDoubleSigma
			READ getDoubleSigma
			WRITE setDoubleSigma
			DEFAULT 1.0
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, DoubleSigma);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double doubleAlpha
			DATA getDataDoubleAlpha
			READ getDoubleAlpha
			WRITE setDoubleAlpha
			DEFAULT 0.2
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, DoubleAlpha);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double doubleBeta
			DATA getDataDoubleBeta
			READ getDoubleBeta
			WRITE setDoubleBeta
			DEFAULT 0.2
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, DoubleBeta);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double doubleGamma
			DATA getDataDoubleGamma
			READ getDoubleGamma
			WRITE setDoubleGamma
			DEFAULT 0.2
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, DoubleGamma);

		/// \property 
		///
		/// \brief 
		VSD_PROPERTY_VAL(double updateMapRate
			DATA getDataUpdateMapRate
			READ getUpdateMapRate
			WRITE setUpdateMapRate
			UNIT VSD::Unit::Hertz
			DEFAULT 2.0
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, UpdateMapRate);

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

		/// \property
		///
		/// \brief
		VSD_PROPERTY_VAL(bool boolUseRealValueForControl
			DATA getDataBoolUseRealValueForControl
			READ getBoolUseRealValueForControl
			WRITE setBoolUseRealValueForControl
			DEFAULT false
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolUseRealValueForControl);

		/// \property
		///
		/// \brief
		VSD_PROPERTY_VAL(bool boolUseRealValueForMapping
			DATA getDataBoolUseRealValueForMapping
			READ getBoolUseRealValueForMapping
			WRITE setBoolUseRealValueForMapping
			DEFAULT false
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolUseRealValueForMapping);

   // management
	public:
		VSM::Vector3 getRobotPose();
		std::vector<std::pair<double, double>> getCollavPointCloud();
		std::vector<std::pair<double, double>> getLaserPointCloud();
		std::vector<std::pair<double, double>> getCollisionPoints();
      std::vector<std::pair<double, double>> getIntersectionPoints();
      std::vector<double> getVecRadius();

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
		void slotEnableModified(VSDIO::Input* input);
		void slotPoseValuesModified(VSDIO::Input* input);
		void slotMovementValueModified(VSDIO::Input* input);
		void slotLaserscanValueModified(VSDIO::Input* input);
		void slotUpdateLoop();
		void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);

		void slotMapThreadFinished();

   private:
		// CA vars
		VSM::Vector3 robotPose_, goalPose_;
		double velTrans_, velRot_;
		std::vector<std::pair<double, double>> laserscanInRobotFrame_;

		// debug vars
		VSM::Vector3 lastRobotPose_;
		std::pair<double, double> lastControl_;
		VSM::Vector3 expectedPose_;

		// timer
		QTimer *timer_;

		// laser transform
		VSM::Frame sensorToBaseTransform_;
		unsigned int indexRange_;
		std::vector<double> indexAngle_;
		std::vector<std::pair<double, double>> laserscanInRobotFrameFromRange(std::vector<double> rangeData);
		void setLaserscanConversionData();

		// CA class
		double lastUpdateControlTime_;
		VSLibMobileRobotNavigation::CollavDWA collavInstance_;

		// Map data
		double lastUpdateMapTime_;
		int imgWidth_, imgHeight_;
		std::vector<double> laserscanRaw_;
		void setStaticGridmap();
		VSLibMarkerLocalization::Gridmap occmap_;

		// qtconcurrent
		bool mapThreadBusy_;
		void calcMapInThread();
		QFutureWatcher<void> *mapThreadWatcher;

		// test if properties are valid
		bool propertyCheck();

		// visu
		CollavVisu* visu_;

   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMobileRobotNavigation::CollisionAvoidanceNode);

#endif //VSPluginMobileRobotNavigationCollisionAvoidanceNodeH
