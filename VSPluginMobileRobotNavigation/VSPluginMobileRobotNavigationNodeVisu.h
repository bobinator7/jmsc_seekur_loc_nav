#ifndef VSPluginMobileRobotNavigationNodeVisuH
#define VSPluginMobileRobotNavigationNodeVisuH

// specific
#include "../VSLibRenderGL/VSLibRenderGLRenderExtension.h"

#include <Lib/VSD/VSDSimStateInstance.h>

#include <unordered_map>

namespace VEROSIM
{
   class Project;
}

namespace VSPluginMobileRobotNavigation
{
	// TODO: rename visu class (used by ekf and ukf)
   class CollavVisu : public VSLibRenderGL::RenderExtension
   {
   // construction
   public:
		CollavVisu(VEROSIM::Project* project, VSD::SimStateInstance* caller);
      ~CollavVisu();

      // management
      void render(VSLibRenderGL::RenderOption option);
      virtual void initialize();

   private:
		void renderPointCloud();
      void renderMaxRadialPaths();

   // data
   private:
      VSD::SimStateInstance* myCaller;
		VSM::Vector3 robotPose_;
		std::vector<std::pair<double, double>> collavPointCloudInRobotFrame_;
		std::vector<std::pair<double, double>> laserPointCloudInRobotFrame_;
		std::vector<std::pair<double, double>> collPoints_;
      std::vector<std::pair<double, double>> intersectPoints_;
      std::vector<double> vecRadius_;

		VSM::Frame getFrameFromPose(VSM::Vector3 inPose, double height = 0);

   }; // class CollavVisu

}; // namespace VSPluginMobileRobotNavigation

#endif //VSPluginMobileRobotNavigationBoxVisuH
