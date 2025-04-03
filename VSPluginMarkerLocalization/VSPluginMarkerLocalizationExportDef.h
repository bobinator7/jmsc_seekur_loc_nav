#ifndef VSPluginMarkerLocalizationExportDefH
#define VSPluginMarkerLocalizationExportDefH

#ifndef VSPluginMarkerLocalization_DECLSPEC
   #ifdef __unix
      #define VSPluginMarkerLocalization_DECLSPEC
   #else
      #ifdef EXPORT_VSPluginMarkerLocalization
         #define VSPluginMarkerLocalization_DECLSPEC __declspec(dllexport)
      #else
         #define VSPluginMarkerLocalization_DECLSPEC __declspec(dllimport)
      #endif
   #endif
#endif


#endif //VSPluginMarkerLocalizationExportDefH
