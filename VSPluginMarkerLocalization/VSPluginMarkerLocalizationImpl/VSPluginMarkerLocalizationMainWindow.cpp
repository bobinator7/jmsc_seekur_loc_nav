// header
#include "../VSPluginMarkerLocalizationMainWindow.h"

// specific
#include "../VSPluginMarkerLocalizationProject.h"

// other
#include "Main/VEROSIM/VEROSIMApplication.h"
#include "Main/VEROSIM/VEROSIMGUIMainWindow.h"
#include "Main/VEROSIM/VEROSIMProject.h"
#include "Lib/VSLibGUI/VSLibGUIAction.h"
#include "Lib/VSUI/VSUIManager.h"

VSPluginMarkerLocalization::MainWindow::MainWindow(VSP::Interface* interface, VEROSIM::MainWindow* mainwindow) 
: VSP::MainWindow(interface, mainwindow)
{
   connect( mainwindow
          , SIGNAL(signalInitMainWindow(VEROSIM::MainWindow*))
          , this
          , SLOT(slotInitMainWindow()) );

   connect( mainwindow
          , SIGNAL(signalProjectAttached(VEROSIM::MainWindow*,VEROSIM::Project*))
          , this
          , SLOT(slotProjectAttached()) );
}

VSPluginMarkerLocalization::MainWindow::~MainWindow()
{
}

void VSPluginMarkerLocalization::MainWindow::slotInitMainWindow()
{
   myAction = new VSLibGUI::Action(  getMainWindow()
                                   , "VSPluginMarkerLocalization.MyAction"
                                   , QIcon(VSUI::Manager::the()->compileLocalIconFileName(QUrl("icon:sensor_data_processing")))
                                   , false);
}

void VSPluginMarkerLocalization::MainWindow::slotProjectAttached()
{   
   Project* extProject = getProject()->findChild<Project*>(); 
   if (extProject)
   {
      bool success = connect(myAction
                           , SIGNAL(triggered(bool))
                           , extProject
                           , SLOT(slotMyAction(bool)));
      Q_ASSERT(success);
      Q_UNUSED(success);
   }
}
