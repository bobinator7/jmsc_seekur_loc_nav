#include "../VSLibARMarkerOutputMarker.h"

#include "../VSLibARMarkerDatatypes.h"


namespace VSLibARMarker
{

// construction
OutputMarker::OutputMarker(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
   : VSDIO::OutputTemplate<Marker>(mySimState, otherSimStateInstance)
   , dataLocalValue(this)
   , dataForcedValue(this)
   , dataErrorValue(this)
{
}

OutputMarker::~OutputMarker()
{
}

}//end namespace