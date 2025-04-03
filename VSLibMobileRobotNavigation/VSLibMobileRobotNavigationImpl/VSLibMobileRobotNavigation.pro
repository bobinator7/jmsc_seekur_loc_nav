TEMPLATE    = lib
CONFIG      += dll
TARGET      = VSLibMobileRobotNavigation

include(../../../Main/Global/Defines.pro)
CONFIG	    += exceptions

DEFINES += EXPORT_VSLibMobileRobotNavigation

# 3rd party
include($$(VEROSIMSourceFolder)/3rdParty/Eigen/Eigen-3.2.9.pri)
#include($$(VEROSIMSourceFolder)/3rdParty/PCL/pcl-1.8.0.pri)
#include($$(VEROSIMSourceFolder)/3rdParty/boost/boost-1.65.0.pri)


#Input

HEADERS =  \
   ../VSLibMobileRobotNavigationExport.h \
   ../VSLibMobileRobotNavigationCollavDWA.h \
   ../VSLibMobileRobotNavigationTools.h \
   ../VSLibMobileRobotNavigationTools.hpp \

SOURCES =  \
   VSLibMobileRobotNavigationTools.cpp \
   VSLibMobileRobotNavigationCollavDWA.cpp \
   
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
                  VSLibMarkerLocalization \
                  VSLibMemory \
                  VSLibSensor \
                  VSLibSensorDataProcessing \
                  VSLibSelection

VS_DESCRIPTION = "Mobile Robot Navigation Functionality Library"
VS_MAINTAINER = johnson.loh@rwth-aachen.de
VS_ACTIVE = Win64VC14

#VS_DATA_END

