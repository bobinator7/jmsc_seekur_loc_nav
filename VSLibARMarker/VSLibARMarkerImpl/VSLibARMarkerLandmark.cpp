#include "../../../Lib/VSD/VSDSimState.h"
#include "../../../Lib/VSD/VSDDatabase.h"
#include "../VSLibARMarkerLandmark.h"

#include "../VSLibARMarkerLandmarkExtension.h"



VSLibARMarker::Landmark::Landmark(VSM::Frame worldFrame, double edgeLength, double trust)
: VSLibVisualGPS::Landmark(worldFrame, trust)
, edgeLength (edgeLength)
{
}

VSLibARMarker::Landmark::~Landmark()
{
}

VSLibVisualGPS::LandmarkExtension* VSLibARMarker::Landmark::initExtension(VSD::SimState*simstate, VSD::ModelInstance* parent, QString parentProperty, bool visible)
{
   VSLibARMarker::LandmarkExtension* result =
      simstate->newSimStateInstance<VSLibARMarker::LandmarkExtension>();

   result->setEdgeLength(edgeLength);

   //get property-list from string
   const VSD::MetaProperty* metaProp = parent->getMetaInstance()->findMyOrBaseClassProperty(parentProperty);
   
   if (!metaProp)
      return result;

   VSD::PropertyRefList* refList = metaProp->getData(parent)->toPropertyRefList();
   if (!refList)
      return result;
   
	return result;
}

