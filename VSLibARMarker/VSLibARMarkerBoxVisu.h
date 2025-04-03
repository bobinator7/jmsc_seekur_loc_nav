#ifndef VSLibARMarkerBoxVisuH
#define VSLibARMarkerBoxVisuH

// specific
#include "../VSLibRenderGL/VSLibRenderGLRenderExtension.h"

#include <Lib/VSD/VSDSimStateInstance.h>

namespace VEROSIM
{
   class Project;
}

namespace VSLibARMarker
{
   class LocalizationNode;
   class ExtensionMarkerVisu;

   class BoxVisu : public VSLibRenderGL::RenderExtension
   {
   // construction
   public:
      BoxVisu(VEROSIM::Project* project, VSD::SimStateInstance* caller);
      //BoxVisu(VEROSIM::Project* project, LocalizationNode* callerNode);
      //BoxVisu(VEROSIM::Project* project, ExtensionMarkerVisu* callerExtension);
      ~BoxVisu();

      // management
      void render(VSLibRenderGL::RenderOption option);
      virtual void initialize();

   private:
      void renderBox(double edgeLength=1);
      void renderAxis();

   // data
   private:
      VSD::SimStateInstance* myCaller;
      //LocalizationNode* myCallerNode;
      //ExtensionMarkerVisu* myCallerExtension;

   }; // class BoxVisu
}; // namespace VSLibARMarker

#endif //VSLibARMarkerBoxVisuH
