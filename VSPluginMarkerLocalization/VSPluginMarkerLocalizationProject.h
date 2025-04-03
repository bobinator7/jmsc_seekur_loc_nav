#ifndef VSPluginMarkerLocalizationProjectH
#define VSPluginMarkerLocalizationProjectH

// base
#include "Lib/VSP/VSPProject.h"

// other

namespace VSD
{
   class SimState;
}

namespace VSPluginMarkerLocalization 
{
   class Project : public VSP::Project
   {
	   Q_OBJECT

   // construction	   
   public:
      Project(VSP::Interface* interface, VEROSIM::Project* project);   	
      virtual ~Project();

      // management      
   private slots:
      void slotInitProject(void);
      void slotPostProjectLoaded(VEROSIM::Project* project, VSD::SimState* simState);

      void slotMyAction(bool triggered);

   }; // class Project
}; // namespace VSPluginMarkerLocalization

#endif // VSPluginMarkerLocalizationProjectH 

