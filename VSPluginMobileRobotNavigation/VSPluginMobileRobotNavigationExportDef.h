#ifndef VSPluginSensorDataProcessingExampleExportDefH
#define VSPluginSensorDataProcessingExampleExportDefH

#ifndef VSPluginSensorDataProcessingExample_DECLSPEC
   #ifdef __unix
      #define VSPluginSensorDataProcessingExample_DECLSPEC
   #else
      #ifdef EXTENSION_VSPluginSensorDataProcessingExample
         #define VSPluginSensorDataProcessingExample_DECLSPEC __declspec(dllexport)
      #else
         #define VSPluginSensorDataProcessingExample_DECLSPEC __declspec(dllimport)
      #endif
   #endif
#endif


#endif //VSPluginSensorDataProcessingExampleExportDefH
