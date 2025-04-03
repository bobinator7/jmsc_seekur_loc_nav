#ifndef VSLibARMarkerOutputMarkerVectorH
#define VSLibARMarkerOutputMarkerVectorH

// baseclass
#include "Lib/VSDIO/VSDIOOutputTemplate.h"

#include "VSLibARMarkerExport.h"
#include "VSLibARMarkerDatatypes.h"

namespace VSLibARMarker
{
   /// \brief  VSDIO::OutputTemplate of a aruco::Marker, to be used with IO connections.
   ///

   class VSLibARMarker_DECLSPEC OutputMarkerVector : public VSDIO::OutputTemplate<QVector<VSLibARMarker::Marker> >
   {
      VSD_ENVIRONMENT_INSTANCE(
         DISPLAY_NAME tr("Output vector of aruco markers")
         ICON ":/VSLibARMarker/icons/Output.png"
      );

      // construction
   protected:
      OutputMarkerVector(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance);
      virtual ~OutputMarkerVector();

      // services
   public:

      // data
      VSD_PROPERTY_VAL(QVector<VSLibARMarker::Marker> localValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<VSLibARMarker::Marker>, LocalValue);

      VSD_PROPERTY_VAL(QVector<VSLibARMarker::Marker> forcedValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<VSLibARMarker::Marker>, ForcedValue);

      VSD_PROPERTY_VAL(QVector<VSLibARMarker::Marker> errorValue
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
      VSD_PROPERTY_TEMPLATE_DATA_VAL_INTERNAL(QVector<VSLibARMarker::Marker>, ErrorValue);

   protected:
      virtual VSD::PropertyValInternalTemplate<QVector<VSLibARMarker::Marker> >* getDataLocalValue() { return &dataLocalValue; }
      virtual VSD::PropertyValInternalTemplate<QVector<VSLibARMarker::Marker> >* getDataForcedValue() { return &dataForcedValue; }
      virtual VSD::PropertyValInternalTemplate<QVector<VSLibARMarker::Marker> >* getDataErrorValue() { return &dataErrorValue; }

   }; // class OutputMarkerVector

}//end namespace

VSD_DECLARE_ENVIRONMENT_INSTANCE(VSLibARMarker::OutputMarkerVector);

#endif // VSLibARMarkerOutputMarkerVectorH
