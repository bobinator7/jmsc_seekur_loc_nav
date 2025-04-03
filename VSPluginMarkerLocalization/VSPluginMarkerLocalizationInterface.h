#ifndef VSPluginMarkerLocalizationInterfaceH
#define VSPluginMarkerLocalizationInterfaceH

// base
#include "Lib/VSP/VSPInterface.h"

// specific
namespace VEROSIM
{
   class MainWindow;
   class Project;
}

namespace VSPluginMarkerLocalization 
{
   class Interface : public VSP::Interface
   {
	   Q_OBJECT

   // construction	   
   public:
      Interface(VSL::Application* application, VSL::Library* library);   
      virtual ~Interface();

   // management      
   private slots:
      void slotCreateNewExtProject(VEROSIM::Project* project);
      void slotCreateNewExtMainWindow(VEROSIM::MainWindow* mainwindow);
   }; // class Interface

}; // namespace VSPluginMarkerLocalization

#endif // VSPluginMarkerLocalizationInterfaceH 
