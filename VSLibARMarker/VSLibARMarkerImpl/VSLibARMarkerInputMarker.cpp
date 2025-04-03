#include "../VSLibARMarkerInputMarker.h"

#include "../VSLibARMarkerDatatypes.h"


namespace VSLibARMarker
{

// construction
InputMarker::InputMarker(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
   : VSDIO::InputTemplate<Marker>(mySimState, otherSimStateInstance)
   , dataLocalValue(this)
   , dataConnectedValue(this)
   , dataForcedValue(this)
   , dataErrorValue(this)
{
   // disallow distribuation/propagation via VSPluginDis

   // WARNING, this calls "setOption" on the BASE class, not the current one,
   // see http://www.parashift.com/c++-faq-lite/strange-inheritance.html#faq-23.5
   // and http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.7
   // Apparently, it works anyway.
   this->setOption(VSD::SimStateInstance::SyncDistributed, false);
}


InputMarker::~InputMarker()
{
}


}//end namespace