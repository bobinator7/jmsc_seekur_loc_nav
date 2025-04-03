// header
#include "../VSPluginMobileRobotNavigationInterface.h"

// specific
#include "../VSPluginMobileRobotNavigationProject.h"
#include "../VSPluginMobileRobotNavigationMainWindow.h"

// standard
#include "Main/VEROSIM/VEROSIMApplication.h"
#include "Main/VEROSIM/VEROSIMFeatures.h"

const bool AllowMultipleProjects = true;

VSPluginMobileRobotNavigation::Interface::Interface(VSL::Application* application, VSL::Library* library) 
: VSP::Interface(application, library, AllowMultipleProjects)
{
   if (0 == getVEROSIMApplication())
   {
      ShowError("%s: This is not a VEROSIM application!", metaObject()->className());
      return;
   }
   
   // connect to project initiators            
   connect(application
         , SIGNAL(signalNewProject(VEROSIM::Project*,VSD::SimState*))
         , this
         , SLOT(slotCreateNewExtProject(VEROSIM::Project*)));

   connect(application
         , SIGNAL(signalNewMainWindow(VEROSIM::MainWindow*))
         , this
         , SLOT(slotCreateNewExtMainWindow(VEROSIM::MainWindow*)));
}

VSPluginMobileRobotNavigation::Interface::~Interface()
{
}

void VSPluginMobileRobotNavigation::Interface::slotCreateNewExtProject(VEROSIM::Project* project)
{
   (void) new VSPluginMobileRobotNavigation::Project(this, project);
}

void VSPluginMobileRobotNavigation::Interface::slotCreateNewExtMainWindow(VEROSIM::MainWindow* mainwindow)
{
   (void) new VSPluginMobileRobotNavigation::MainWindow(this, mainwindow);
}
