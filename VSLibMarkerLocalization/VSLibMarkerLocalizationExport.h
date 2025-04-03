#ifdef __unix
   #define VSLibMarkerLocalization_DECLSPEC
#else
   #ifdef EXPORT_VSLibMarkerLocalization
      #define VSLibMarkerLocalization_DECLSPEC __declspec(dllexport)
   #else
      #define VSLibMarkerLocalization_DECLSPEC __declspec(dllimport)
   #endif
#endif
