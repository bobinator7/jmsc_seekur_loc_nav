#ifdef __unix
   #define VSLibARMarker_DECLSPEC
#else
   #ifdef EXTENSION_VSLibARMarker
      #define VSLibARMarker_DECLSPEC __declspec(dllexport)
   #else
      #define VSLibARMarker_DECLSPEC __declspec(dllimport)
   #endif
#endif
