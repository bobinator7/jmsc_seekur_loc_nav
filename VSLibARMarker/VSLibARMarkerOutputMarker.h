#ifndef VSLibARMarkerOutputMarkerH
#define VSLibARMarkerOutputMarkerH

// baseclass
#include "Lib/VSDIO/VSDIOOutputTemplate.h"

#include "VSLibARMarkerDatatypes.h"



namespace VSDIO
{
   class Output;
}

namespace VSLibARMarker
{
   /// \brief  VSDIO::OutputTemplate of a aruco::Marker, to be used with IO connections.
   ///
   class VSLibARMarker_DECLSPEC OutputMarker : public VSDIO::OutputTemplate<Marker>
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("Output aruco marker")
         ICON ":/VSLibARMarker/icons/Output.png"
      );

      // construction
   protected:
      OutputMarker(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~OutputMarker();

      // services
   public:

      // data
   VSD_PROPERTY_VAL(VSLibARMarker::Marker localValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(VSLibARMarker::Marker, LocalValue);

      VSD_PROPERTY_VAL(VSLibARMarker::Marker forcedValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(VSLibARMarker::Marker, ForcedValue);

   VSD_PROPERTY_VAL(VSLibARMarker::Marker errorValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(VSLibARMarker::Marker, ErrorValue);

   protected:
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataLocalValue() { return &dataLocalValue; }
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataForcedValue() { return &dataForcedValue; }
      virtual VSD::PropertyValInternalTemplate<Marker>* getDataErrorValue() { return &dataErrorValue; }


   };//end class

}//end namespace

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::OutputMarker);

#endif // VSLibARMarkerOutputMarkerH
