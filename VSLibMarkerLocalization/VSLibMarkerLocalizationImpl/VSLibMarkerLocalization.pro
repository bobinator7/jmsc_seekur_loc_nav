TEMPLATE    = lib
CONFIG      += dll
TARGET      = VSLibMarkerLocalization

include(../../../Main/Global/Defines.pro)
CONFIG	    += exceptions

DEFINES += EXPORT_VSLibMarkerLocalization

# 3rd party
include($$(VEROSIMSourceFolder)/3rdParty/Eigen/Eigen-3.2.9.pri)

#Input

HEADERS =  \
   ../VSLibMarkerLocalization2DEKF.h \
   ../VSLibMarkerLocalization2DUKF.h \
   ../VSLibMarkerLocalizationExport.h \
   ../VSLibMarkerLocalizationGridmap.h \
   ../VSLibMarkerLocalizationTools.h \
   ../VSLibMarkerLocalizationTools.hpp \


SOURCES =  \
   VSLibMarkerLocalization2DEKF.cpp \
   VSLibMarkerLocalization2DUKF.cpp \
   VSLibMarkerLocalizationGridmap.cpp \
   VSLibMarkerLocalizationTools.cpp \
   
#RESOURCES = \

#VS_DATA_START

VS_DEPENDENCIES = \
                  VSD \
                  VSD3D \
                  VSDIO \
                  VSG \
                  VSL \
                  VSM \
                  VSP \
                  VSUI \
                  VSLibGUI \
                  VSLibMemory \
                  VSLibSensor \
                  VSLibSensorDataProcessing \
                  VSLibSelection

VS_DESCRIPTION = "Marker Localization Functionality Library"
VS_MAINTAINER = johnson.loh@rwth-aachen.de
VS_ACTIVE = Win64VC14

#VS_DATA_END

