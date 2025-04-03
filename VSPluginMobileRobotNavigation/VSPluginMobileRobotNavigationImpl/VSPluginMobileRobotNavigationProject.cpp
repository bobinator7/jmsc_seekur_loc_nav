// header
#include "../VSPluginMobileRobotNavigationProject.h"

// standard
#include "Main/VEROSIM/VEROSIMProject.h"

// other

VSPluginMobileRobotNavigation::Project::Project(VSP::Interface* interface, VEROSIM::Project* project) 
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

VSPluginMobileRobotNavigation::Project::~Project()
{
}

void VSPluginMobileRobotNavigation::Project::slotInitProject(void)
{
}

void VSPluginMobileRobotNavigation::Project::slotPostProjectLoaded(VEROSIM::Project* project,VSD::SimState* simState)
{
   Q_UNUSED(project);
   Q_UNUSED(simState);
}

void VSPluginMobileRobotNavigation::Project::slotMyAction(bool triggered)
{
   ShowInfo("VSPluginMobileRobotNavigation::Project: MyAction was triggered!");
   Q_UNUSED(triggered);
}