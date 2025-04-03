#ifndef VSLibMarkerLocalizationToolsH
#define VSLibMarkerLocalizationToolsH


#include "VSLibMarkerLocalizationExport.h"

// VEROSIM
#include "../../Lib/VSM/VSMMatrixNxM.h"
#include "../../Lib/VSM/VSMPoseVector3Quaternion.h"

// other
#include <Eigen/Dense>

#include <unordered_map>


namespace VSLibMarkerLocalization
{

   class VSLibMarkerLocalization_DECLSPEC Tools
   {
   public:
		// marker map type conversion
      static std::unordered_map<int, VSM::PoseVector3Quaternion> stdUnorderedMap3DFromVsm3DMarkerMat(VSM::MatrixNxM mat_in);

      static VSM::MatrixNxM vsm3DMarkerMatFromStdUnorderedMap3D(std::unordered_map<int, VSM::PoseVector3Quaternion> map_in);

      static VSM::MatrixNxM vsm2DMarkerMatFromStdUnorderedMap3D(std::unordered_map<int, VSM::PoseVector3Quaternion> map_in);

      static VSM::MatrixNxM vsm2DMarkerMatFromStdUnorderedMap2D(std::unordered_map<int, VSM::Vector3> map_in);

		// eigen matrix manipulation
		static void removeEigenMatRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);

		static void removeEigenMatColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);

		static void fixBrokenCorrelationMatrix(Eigen::MatrixXd& matrix);

		// configuration difference with proper angle wrapping
		static void stateWrapAngle(Eigen::VectorXd& vecIn);

   };
} //end namespace VSLibMarkerLocalization

#endif 
