#ifndef VSLibARMarkerLandmarkH
#define VSLibARMarkerLandmarkH


#include "../../Lib/VSD/VSDModelInstanceExtension.h"
#include "../../Lib/VSM/VSMVector3.h"

//interface
#include "../../Plugin/VSLibVisualGPS/VSLibVisualGPSLandmark.h"

#include "VSLibARMarkerExport.h"


namespace VSLibVisualGPS
{
   class LandmarkExtension;
}

namespace VSLibSensor
{
   class LaserScanner2DExtensionBase;
};

namespace VSLibARMarker
{
   class VSLibARMarker_DECLSPEC Landmark : public VSLibVisualGPS::Landmark
   {
      Q_OBJECT;

   // construction
   public:
      Landmark(VSM::Frame worldFrame, double edgeLength, double trust);
      virtual ~Landmark();

      //virtual void postHasBeenAddedToElementContainer();

      double getEdgeLength() const {return edgeLength;};


   protected:
      VSLibVisualGPS::LandmarkExtension* initExtension(VSD::SimState* simstate, VSD::ModelInstance* parent, QString parentProperty, bool visisble);

      double edgeLength;

   private:
   }; // class MarkerPercept

}; // namespace VSPluginForestLandmarks

#endif //VSLibVFVisualGPSMarkerPerceptH
