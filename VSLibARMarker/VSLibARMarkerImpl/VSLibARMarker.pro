TEMPLATE    = vclib
CONFIG      += dll
TARGET      = VSLibARMarker

include(../../../Main/Global/Defines.pro)
QT      += opengl

#HOSTNAME=$$system(hostname)

DEFINES += EXTENSION_VSLibARMarker
DEFINES += VGLX_IMPORT

include($$(VEROSIMSourceFolder)/3rdParty/openCV/opencv-3.1.0.pri)
#include($$(VEROSIMSourceFolder)/3rdParty/aruco/aruco-1.2.4.pri)

# Sollen DLLs wenn benötigt geladen werden?
!debug_and_release|build_pass {
    CONFIG(debug, debug|release) {
        MSVCPROJ_LIBOPTIONS += /IMPLIB:$${DESTDIR_PARENT}/debug/VSLibARMarkerD.lib
    }
    CONFIG(release, debug|release) {
        MSVCPROJ_LIBOPTIONS += /IMPLIB:$${DESTDIR_PARENT}/release/VSLibARMarker.lib
    }
}

#Input

SOURCES =  \
   VSLibARMarkerBoxVisu.cpp \
   VSLibARMarkerDatatypes.cpp \
   VSLibARMarkerDetectionNode.cpp \
   VSLibARMarkerExtensionMarkerVisu.cpp \
   VSLibARMarkerNode.cpp \
#   VSLibARMarkerInputMarker.cpp \
#   VSLibARMarkerInputMarkerVector.cpp \
   VSLibARMarkerTools.cpp \
   VSLibARMarkerLandmark.cpp \
    VSLibARMarkerLandmarkExtension.cpp \
   VSLibARMarkerLocalizationNode.cpp \
#   VSLibARMarkerOutputMarker.cpp \
#   VSLibARMarkerOutputMarkerVector.cpp \
   VSLibARMarkerRelativePoseNode.cpp
      
HEADERS =  \
   ../VSLibARMarkerBoxVisu.h \
   ../VSLibARMarkerExport.h \
   ../VSLibARMarkerDatatypes.h \
   ../VSLibARMarkerDetectionNode.h \
   ../VSLibARMarkerExtensionMarkerVisu.h \
   ../VSLibARMarkerNode.h \
#   ../VSLibARMarkerInputMarker.h \
#   ../VSLibARMarkerInputMarkerVector.h \
   ../VSLibARMarkerTools.h \
   ../VSLibARMarkerLandmark.h \
   ../VSLibARMarkerLandmarkExtension.h \
   ../VSLibARMarkerLocalizationNode.h \
   ../VSLibARMarkerMetaTypeVAlAtomicMarker.h \
   ../VSLibARMarkerMetaTypeVAlAtomicMarker.hpp \
#   ../VSLibARMarkerOutputMarker.h \
#   ../VSLibARMarkerOutputMarkerVector.h \
   ../VSLibARMarkerRelativePoseNode.h
   
RESOURCES = \
   VSLibARMarker.qrc

   
#VS_DATA_START

VS_DEPENDENCIES =\
VSLibOpenCV \
VSLibRenderGL \
VSLibImageProcessing \
VSLibSensor \
VSLibSensorDataProcessing \
VSLibVisualGPS \
VSD \
VSDTools \
VSD3D \
VSLibMemory \
VSDIO \
VSM


VS_DESCRIPTION = "Plugin for using AR markers"
VS_MAINTAINER = sondermann@mmi.rwth-aachen.de
VS_ACTIVE = Win64VC14
#VS_DATA_END

