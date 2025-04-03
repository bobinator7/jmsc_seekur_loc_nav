#ifndef VSLibARMarkerMetaTypeValAtomicMarkerH
#define VSLibARMarkerMetaTypeValAtomicMarkerH

// baseclass
#include "Lib/VSD/VSDMetaTypeValAtomicBasicWithoutStreamingTemplate.h"

#include "VSLibARMarkerDatatypes.h"

namespace VSLibARMarker
{
   class MetaTypeValAtomicMarker : public VSD::MetaTypeValAtomicBasicWithoutStreamingTemplate<Marker>
   {
      // additional
   public:

      // construction
     public:
      MetaTypeValAtomicMarker();
      virtual ~MetaTypeValAtomicMarker();

      // services
   public:

      // Stream-I/O
      bool load(QTextStream& stream, void* val) const;
      ToStringConversionResult save(QTextStream& stream, const void* val, int maxSizeHint) const;
      bool load(QDataStream& stream, void* val) const;
      bool save(QDataStream& stream, const void* val) const;

      // Stream-I/O für QVariant
      VSD_METATYPE_SIMPLE_VARIANT_STREAMING_FUNCTIONS(Marker);

      // management
   private:

      // data
   private:

   }; // class MetaTypeValAtomicMarker

   // inline/ Marker member functions
   #include "VSLibARMarkerMetaTypeValAtomicMarker.hpp"

}; // namespace VSLibARMarker

#endif // VSLibOpenCVMetaTypeValAtomicMarkerH
