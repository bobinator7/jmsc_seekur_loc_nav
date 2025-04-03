#ifdef __unix
   #define VSLibMobileRobotNavigation_DECLSPEC
#else
   #ifdef EXPORT_VSLibMobileRobotNavigation
      #define VSLibMobileRobotNavigation_DECLSPEC __declspec(dllexport)
   #else
      #define VSLibMobileRobotNavigation_DECLSPEC __declspec(dllimport)
   #endif
#endif
