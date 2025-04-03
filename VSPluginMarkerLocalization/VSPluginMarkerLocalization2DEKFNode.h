#ifndef VSPluginMarkerLocalization2DEKFNodeH
#define VSPluginMarkerLocalization2DEKFNodeH

// base classes
#include "../VSLibSensorDataProcessing/VSLibSensorDataProcessingSensorDataProcessingNodeBase.h"

// other
#include "../VSLibMarkerLocalization/VSLibMarkerLocalization2DEKF.h"
#include "../VSLibSensor/VSLibSensorSensorExtensionBase.h"

#include "../VSPluginMarkerLocalization/VSPluginMarkerLocalizationNodeVisu.h"

#include "Lib/VSM/VSMMatrix3x3.h"
#include "Lib/VSM/VSMVector3.h"

#include <unordered_map>
#include <set>

namespace VSPluginMarkerLocalization
{
   class Marker2DEKFNode : public VSLibSensorDataProcessing::SensorDataProcessingNodeBase
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("Marker2DEKFNode")
      );

   // construction
   protected:
	   Marker2DEKFNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~Marker2DEKFNode();
	     
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
      void slotVelocityModified(VSDIO::Input* input);
		void slotRotationModified(VSDIO::Input* input);
		void slotMeasurementModified(VSDIO::Input* input);
		void slotPropertiesModified(VSD::SimStateInstance* simStateInstance, const VSD::MetaProperty* metaProperty);
		void slotUpdatePoseWithOdom();

      void slotPropagateDirtyFlag(){};
      void slotPropagateChangeSignal(){};
		

   //data
	public:
		VSM::Vector3 getRobotPose();
		VSM::Matrix3x3 getRobotCovariance();
		std::unordered_map<int, VSM::Vector3> getMarkerPoses();
		std::unordered_map<int, VSM::Matrix3x3> getMarkerCovariances();

   private:
		// class fcn
		bool loadStaticMapFromFile();
		VSM::Frame vec2frame(VSM::Vector3 vecIn);

      // visu
		EKFVisu* visu_;

		// input vars
		double inVelocity_;
		double inRotation_;
		VSM::MatrixNxM inMarkerMeasurement_;
		
		// EKF vars
		//VSM::Matrix3x3 Q_, R_;
		VSLibMarkerLocalization::EKF2DInstance filter_;
		VSM::Vector3 robotPose_;
		VSM::Matrix3x3 robotCovariance_;
		std::unordered_map<int, VSM::Vector3> markerPoses_;
		std::unordered_map<int, VSM::Matrix3x3> markerCovariances_;

		// misc vars
		int num_of_input_slots_;
		QTimer *timer;
		double lastUpdateTime_;
		std::unordered_map<std::string, bool> updatedSincePrediction_;
		std::set<int> markerSeenUntilPrediction_;
		std::unordered_map<int, VSM::Vector3> staticMarkerMap_;
   
   protected:
         
   public:
   
      // properties
		/// \property bool boolUseSimTime
		///
		/// \brief use simulation time instead of real time
		VSD_PROPERTY_VAL(bool boolUseSimTime
			DATA getDataBoolUseSimTime
			READ getBoolUseSimTime
			WRITE setBoolUseSimTime
			DEFAULT true
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolUseSimTime);

 
      /// \property bool boolEnableVisualization
      ///
      /// \brief enables visualization of ekf poses and covariances
      VSD_PROPERTY_VAL(bool boolEnableVisualization
         DATA getDataBoolEnableVisualization
         READ getBoolEnableVisualization
         WRITE setBoolEnableVisualization
			DEFAULT false
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(bool, BoolEnableVisualization);

		/// \property bool boolEnableVisualization
		///
		/// \brief enables visualization of ekf poses and covariances
		VSD_PROPERTY_VAL(double updateRate
			DATA getDataUpdateRate
			READ getUpdateRate
			WRITE setUpdateRate
			UNIT VSD::Unit::Hertz
			DEFAULT 20.0
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, UpdateRate);

		/// \property intNumberOfCameras
		///
		/// \brief Property - number of input cameras
		VSD_PROPERTY_VAL(int intNumberOfCameras
			DATA getDataIntNumberOfCameras
			READ getIntNumberOfCameras
			WRITE setIntNumberOfCameras
			DEFAULT 4
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY true
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true
			UNIT VSD::Unit::Undefined);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, IntNumberOfCameras);

      /// \property initial pose vector
      ///
      /// \brief initial pose in x,y,theta
      VSD_PROPERTY_VAL(VSM::Vector3 initialRobotPose 
         UNIT VSD::Unit::Meter
         DATA getDataInitialRobotPose READ getInitialRobotPose WRITE setInitialRobotPose
         SAVE true EDIT_VIEW true
         EDIT_MODIFY true CLONE true SYNC_LOCAL_VIA_PROPERTY true SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Vector3, InitialRobotPose);

		/// \property initial covariane
		///
		/// \brief covariance according to robot pose
		VSD_PROPERTY_VAL(VSM::Matrix3x3 initialRobotCovariance
			UNIT VSD::Unit::Meter
			DATA getDataInitialRobotCovariance READ getInitialRobotCovariance WRITE setInitialRobotCovariance
			SAVE true EDIT_VIEW true
			EDIT_MODIFY true CLONE true SYNC_LOCAL_VIA_PROPERTY true SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Matrix3x3, InitialRobotCovariance);

		/// \property initial covariane
		///
		/// \brief covariance according to robot pose
		VSD_PROPERTY_VAL(VSM::Matrix3x3 systemNoiseCovarianceQ
			UNIT VSD::Unit::Meter
			DATA getDataSystemNoiseCovarianceQ READ getSystemNoiseCovarianceQ WRITE setSystemNoiseCovarianceQ
			SAVE true EDIT_VIEW true
			EDIT_MODIFY true CLONE true SYNC_LOCAL_VIA_PROPERTY true SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Matrix3x3, SystemNoiseCovarianceQ);

		/// \property initial covariane
		///
		/// \brief covariance according to robot pose
		VSD_PROPERTY_VAL(VSM::Matrix3x3 measurementNoiseCovarianceR
			UNIT VSD::Unit::Meter
			DATA getDataMeasurementNoiseCovarianceR READ getMeasurementNoiseCovarianceR WRITE setMeasurementNoiseCovarianceR
			SAVE true EDIT_VIEW true
			EDIT_MODIFY true CLONE true SYNC_LOCAL_VIA_PROPERTY true SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(VSM::Matrix3x3, MeasurementNoiseCovarianceR);

		/// \property int intNumMarkerSeen
		///
		/// \brief 
		VSD_PROPERTY_VAL(int intNumMarkerSeen
			DATA getDataIntNumMarkerSeen
			READ getIntNumMarkerSeen
			WRITE setIntNumMarkerSeen
			DEFAULT 0
			SAVE true
			EDIT_VIEW true
			EDIT_MODIFY false
			CLONE true
			SYNC_LOCAL_VIA_PROPERTY true
			SYNC_DISTRIBUTED_VIA_INSTANCE true);
		VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, IntNumMarkerSeen);

		// properties: camera
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
   };
};
VSD_DECLARE_ENVIRONMENT_INSTANCE(VSPluginMarkerLocalization::Marker2DEKFNode);

#endif //VSPluginMarkerLocalizationEKFNodeH
