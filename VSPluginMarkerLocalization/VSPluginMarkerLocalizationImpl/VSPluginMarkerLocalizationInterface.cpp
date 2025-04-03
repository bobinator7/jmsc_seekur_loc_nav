// header
#include "../VSPluginMarkerLocalizationInterface.h"

// specific
#include "../VSPluginMarkerLocalizationProject.h"
#include "../VSPluginMarkerLocalizationMainWindow.h"

// standard
#include "Main/VEROSIM/VEROSIMApplication.h"
#include "Main/VEROSIM/VEROSIMFeatures.h"

const bool AllowMultipleProjects = true;

VSPluginMarkerLocalization::Interface::Interface(VSL::Application* application, VSL::Library* library)
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

VSPluginMarkerLocalization::Interface::~Interface()
{
}

void VSPluginMarkerLocalization::Interface::slotCreateNewExtProject(VEROSIM::Project* project)
{
   (void) new VSPluginMarkerLocalization::Project(this, project);
}

void VSPluginMarkerLocalization::Interface::slotCreateNewExtMainWindow(VEROSIM::MainWindow* mainwindow)
{
   (void) new VSPluginMarkerLocalization::MainWindow(this, mainwindow);
}
