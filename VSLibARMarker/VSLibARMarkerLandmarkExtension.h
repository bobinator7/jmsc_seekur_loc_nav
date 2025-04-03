#ifndef VSLibARMarkerLandmarkExtensionH
#define VSLibARMarkerLandmarkExtensionH


   // base
#include "../../Plugin/VSLibVisualGPS/VSLibVisualGPSLandmarkExtension.h"

// specific
#include "VSLibARMarkerExport.h"


namespace VSLibARMarker
{
   //class Landmark;

   class VSLibARMarker_DECLSPEC LandmarkExtension : public VSLibVisualGPS::LandmarkExtension
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("AR Marker Landmark")
         ICON ":/VSLibARMarker/icons/landmark.png" 
      );

   protected:
      LandmarkExtension(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      ~LandmarkExtension();

   public:
      VSM::Frame getWorldFrame();
      virtual VSLibVisualGPS::Landmark* createModel();

   protected:
      VSD_PROPERTY_VAL(double edgeLength
         UNIT VSD::Unit::Meter
         DATA getDataEdgeLength
         READ getEdgeLength
         WRITE setEdgeLength
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, EdgeLength);

VSD_PROPERTY_VAL(int id
         DATA getDataId
         READ getId
         WRITE setId
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, Id);


      VSD_PROPERTY_VAL(int sightings
         DATA getDataSightings
         READ getSightings
         WRITE setSightings
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, Sightings);

   }; // class LandmarkExtension

}; // namespace VSLibARMarker

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::LandmarkExtension);

#endif //VSLibARMarkerLandmarkExtensionH
