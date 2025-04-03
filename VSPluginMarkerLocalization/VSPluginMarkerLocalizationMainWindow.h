#ifndef VSPluginMarkerLocalizationMainWindowH
#define VSPluginMarkerLocalizationMainWindowH

// base
#include "Lib/VSP/VSPMainWindow.h"

namespace VSLibGUI 
{ 
   class Action; 
}

namespace VSPluginMarkerLocalization 
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
}; // namespace VSPluginMarkerLocalization

#endif // VSPluginMarkerLocalizationMainWindowH 
