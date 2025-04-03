#ifndef VSPluginMarkerLocalizationNodeVisuH
#define VSPluginMarkerLocalizationNodeVisuH

// specific
#include "../VSLibRenderGL/VSLibRenderGLRenderExtension.h"

#include <Lib/VSD/VSDSimStateInstance.h>

#include <unordered_map>

namespace VEROSIM
{
   class Project;
}

namespace VSPluginMarkerLocalization
{
	// TODO: rename visu class (used by ekf and ukf)
   class EKFVisu : public VSLibRenderGL::RenderExtension
   {
   // construction
   public:
		EKFVisu(VEROSIM::Project* project, VSD::SimStateInstance* caller);
      ~EKFVisu();

      // management
      void render(VSLibRenderGL::RenderOption option);
      virtual void initialize();

   private:
      void renderBox(double edgeLength=1);
      void renderAxis();
		void renderArrow(const float width, const float length, const char col = 'r');

   // data
   private:
      VSD::SimStateInstance* myCaller;
		VSM::Vector3 robotPose_;
		VSM::Matrix3x3 robotCovariance_;
		std::unordered_map<int, VSM::Vector3> markerPoses_;
		std::unordered_map<int, VSM::Matrix3x3> markerCovariances_;

		VSM::Frame getFrameFromPose(VSM::Vector3 inPose);

   }; // class EKFVisu

}; // namespace VSPluginMarkerLocalization

#endif //VSPluginMarkerLocalizationBoxVisuH
