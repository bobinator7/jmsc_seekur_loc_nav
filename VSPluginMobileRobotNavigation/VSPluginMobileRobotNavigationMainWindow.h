#ifndef VSPluginMobileRobotNavigationMainWindowH
#define VSPluginMobileRobotNavigationMainWindowH

// base
#include "Lib/VSP/VSPMainWindow.h"

namespace VSLibGUI 
{ 
   class Action; 
}

namespace VSPluginMobileRobotNavigation 
{
   class MainWindow : public VSP::MainWindow
   {
      Q_OBJECT
   
   // construction
   public:
      MainWindow(VSP::Interface* interface, VEROSIM::MainWindow* mainwindow);      
      virtual ~MainWindow();

   // management
   private slots:
      void slotInitMainWindow();
      void slotProjectAttached();

   // data
   private:
      VSLibGUI::Action* myAction;     

   }; // class MainWindow
}; // namespace VSPluginMobileRobotNavigation

#endif // VSPluginMobileRobotNavigationMainWindowH 
