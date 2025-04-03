#ifndef VSLibARMarkerMetaTypeValAtomicMarkerHpp
#define VSLibARMarkerMetaTypeValAtomicMarkerHpp



MetaTypeValAtomicMarker::MetaTypeValAtomicMarker()
   : VSD::MetaTypeValAtomicBasicWithoutStreamingTemplate<Marker>(
                       QStringList()
                     , QStringList()
                     , QList<VSD::AlternativeTypeNameTemplate<Marker> >()
                     , QList<const VSD::MetaTypeValMethod*>())
{
}


MetaTypeValAtomicMarker::~MetaTypeValAtomicMarker()
{
}


bool MetaTypeValAtomicMarker::load(QTextStream& stream, void* val) const
{
   stream >> *((Marker*)val);
   return true; 
}

MetaTypeValAtomicMarker::ToStringConversionResult MetaTypeValAtomicMarker::save(QTextStream& stream, const void* val, int maxSizeHint) const
{
   stream << *((Marker*)val);
   return ToStringConversionSuccess;
   Q_UNUSED(maxSizeHint);
}

bool MetaTypeValAtomicMarker::load(QDataStream& stream, void* val) const
{
   stream >> *((Marker*)val);
   return true;
}

bool MetaTypeValAtomicMarker::save(QDataStream& stream, const void* val) const
{
   stream << *((Marker*)val);
   return true;
}


#endif