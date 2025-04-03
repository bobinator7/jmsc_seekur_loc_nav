#ifndef VSLibARMarkerInputMarkerH
#define VSLibARMarkerInputMarkerH

// baseclass
#include "../../Lib/VSDIO/VSDIOInputTemplate.h"


#include "VSLibARMarkerExport.h"
#include "VSLibARMarkerDatatypes.h"

namespace VSDIO
{
   class Output;
}

namespace VSLibARMarker
{
   /// \brief  VSDIO::InputTemplate of a aruco::Marker, to be used with IO connections.
   ///

   class VSLibARMarker_DECLSPEC InputMarker : public VSDIO::InputTemplate<Marker>
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("Input aruco marker")
         ICON ":/VSLibARMarker/icons/Input.png"
      );

      // construction
   protected:
      InputMarker(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~InputMarker();

      // services
   public:

      // data
      VSD_PROPERTY_VAL(Marker localValue
         DATA getDataLocalValue 
         READ getLocalValue 
         WRITE setLocalValue
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         GET_METHOD_RETURNS_VALUE false);
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(Marker, LocalValue);

      VSD_PROPERTY_VAL(Marker connectedValue
         DATA getDataConnectedValue 
         READ getConnectedValue 
         WRITE setConnectedValue
         SAVE false
         EDIT_VIEW false
         EDIT_MODIFY false
         CLONE false
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         GET_METHOD_RETURNS_VALUE false);
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(Marker, ConnectedValue);

      VSD_PROPERTY_VAL(Marker forcedValue
         DATA getDataForcedValue 
         READ getForcedValue 
         WRITE setForcedValue
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         GET_METHOD_RETURNS_VALUE false);
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(Marker, ForcedValue);

      VSD_PROPERTY_VAL(Marker errorValue
         DATA getDataErrorValue 
         READ getErrorValue 
         WRITE setErrorValue
         SAVE true
         EDIT_VIEW true
         EDIT_MODIFY true
         CLONE true
         SYNC_LOCAL_VIA_PROPERTY true
         SYNC_DISTRIBUTED_VIA_INSTANCE true
         GET_METHOD_RETURNS_VALUE false);
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(Marker, ErrorValue);

   protected:
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataLocalValue() { return &dataLocalValue; }
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataConnectedValue() { return &dataConnectedValue; }
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataForcedValue() { return &dataForcedValue; }
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataErrorValue() { return &dataErrorValue; }

   }; // class InputMarker

}//end namespace

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::InputMarker);

#endif // VSLibARMarkerInputMarkerH
