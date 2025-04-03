#ifndef VSLibARMarkerInputMarkerVectorH
#define VSLibARMarkerInputMarkerVectorH

// baseclass
#include "Lib/VSDIO/VSDIOInputTemplate.h"

#include "VSLibARMarkerExport.h"
#include "VSLibARMarkerDatatypes.h"

namespace VSLibARMarker
{
   /// \brief  VSDIO::InputTemplate of a aruco::Marker, to be used with IO connections.
   ///

   class VSLibARMarker_DECLSPEC InputMarkerVector : public VSDIO::InputTemplate<QVector<Marker> >
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("Input vector of aruco markers")
         ICON ":/VSLibARMarker/icons/Input.png"
      );

      // construction
   protected:
      InputMarkerVector(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~InputMarkerVector();

      // services
   public:

      // data
      VSD_PROPERTY_VAL(QVector<Marker> localValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<Marker>, LocalValue);

      VSD_PROPERTY_VAL(QVector<Marker> connectedValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<Marker>, ConnectedValue);

      VSD_PROPERTY_VAL(QVector<Marker> forcedValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<Marker>, ForcedValue);

      VSD_PROPERTY_VAL(QVector<Marker> errorValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<Marker>, ErrorValue);

   protected:
      virtual VSD::PropertyValInternalTemplate<QVector<Marker> >* getDataLocalValue() { return &dataLocalValue; }
      virtual VSD::PropertyValInternalTemplate<QVector<Marker> >* getDataConnectedValue() { return &dataConnectedValue; }
      virtual VSD::PropertyValInternalTemplate<QVector<Marker> >* getDataForcedValue() { return &dataForcedValue; }
      virtual VSD::PropertyValInternalTemplate<QVector<Marker> >* getDataErrorValue() { return &dataErrorValue; }

   }; // class InputMarkerVector

}//end namespace

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::InputMarkerVector);

#endif // VSLibARMarkerInputMarkerVectorH
