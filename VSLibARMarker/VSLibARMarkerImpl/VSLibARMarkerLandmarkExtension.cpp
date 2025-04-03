#include "../VSLibARMarkerLandmark.h"

#include "../../Lib/VSD3D/VSD3DNode.h"

#include "../VSLibARMarkerLandmarkExtension.h"

VSLibARMarker::LandmarkExtension::LandmarkExtension(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibVisualGPS::LandmarkExtension(mySimState, otherSimStateInstance)
, dataEdgeLength(this)
, dataId(this)
, dataSightings(this)
{
}

VSLibARMarker::LandmarkExtension::~LandmarkExtension()
{
}


VSM::Frame VSLibARMarker::LandmarkExtension::getWorldFrame(void)
{
	VSD3D::Node* marker = getParent()->instanceCast<VSD3D::Node*>();
   Q_ASSERT(marker);

   marker->nodeAndAttachedToWorld();
   return marker->getWorldFrame();
}

VSLibVisualGPS::Landmark* VSLibARMarker::LandmarkExtension::createModel()
{

   model = new VSLibARMarker::Landmark(getWorldFrame(), getEdgeLength(), getTrust());

   model->setExtension(this);
   return model;
}