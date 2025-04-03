#ifndef VSLibARMarkerNodeH
#define VSLibARMarkerNodeH

//base class
#include "Lib/VSD3D/VSD3DNode.h"
#include "VSLibARMarkerExport.h"
#include "VSLibARMarkerLandmarkExtension.h"

#include "Lib/VSD3D/VSD3DHullNode.h"
#include "Lib/VSD3D/VSD3DMaterialNode.h"
#include "Lib/VSD3D/VSD3DMapNode.h"


namespace VSLibARMarker
{
   class VSLibARMarker_DECLSPEC Node : public VSD3D::Node
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("AR marker Node")
      );

   protected:
      Node(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~Node();

      void postHasBeenAddedToElementContainer();
      void preWillBeRemovedFromElementContainer();

   private:
      VSD3D::HullNode* createNewMarkerFace();
      VSD3D::MaterialNode* createNewMarkerMaterial();
      VSD3D::MapNode* createNewMarkerTexture();
      VSLibARMarker::LandmarkExtension* createLMExtension();

      VSD::SimState* mySimState;

	protected slots:
		void slotMakeARMarkerNodeUnique();
		void slotPrintStaticMapFile();

   protected:

      VSD_PROPERTY_VAL(int id
         DATA getDataId
         READ getId
         WRITE setId
         SAVE false
         EDIT_VIEW true
         EDIT_MODIFY false
         CLONE true
         DEFAULT -1
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(int, Id);

      VSD_PROPERTY_VAL(double markerEdgeLength
         DATA getDataMarkerEdgeLength
         READ getMarkerEdgeLength
         WRITE setMarkerEdgeLength
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         DEFAULT 0.3
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true);
      VSD_PROPERTY_TEMPLATE_VAL_INTERNAL(double, MarkerEdgeLength);

   }; // class Node
}; // namespace VSLibARMarker

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::Node);

#endif //VSLibARMarkerNodeH
