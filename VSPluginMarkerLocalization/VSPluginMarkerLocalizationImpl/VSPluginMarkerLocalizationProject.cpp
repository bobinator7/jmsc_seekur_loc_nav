// header
#include "../VSPluginMarkerLocalizationProject.h"

// standard
#include "Main/VEROSIM/VEROSIMProject.h"

// other

VSPluginMarkerLocalization::Project::Project(VSP::Interface* interface, VEROSIM::Project* project) 
: VSP::Project(interface, project)
{
   connect(project
         , SIGNAL(signalInitProject(VEROSIM::Project*,VSD::SimState*))
         , this
         , SLOT(slotInitProject()));

   connect(project
         , SIGNAL(signalPostProjectLoaded(VEROSIM::Project*,VSD::SimState*))
         , this
         , SLOT(slotPostProjectLoaded(VEROSIM::Project*,VSD::SimState*)));
}

VSPluginMarkerLocalization::Project::~Project()
{
}

void VSPluginMarkerLocalization::Project::slotInitProject(void)
{
}

void VSPluginMarkerLocalization::Project::slotPostProjectLoaded(VEROSIM::Project* project,VSD::SimState* simState)
{
   Q_UNUSED(project);
   Q_UNUSED(simState);
}

void VSPluginMarkerLocalization::Project::slotMyAction(bool triggered)
{
   ShowInfo("VSPluginMarkerLocalization::Project: MyAction was triggered!");
   Q_UNUSED(triggered);
}