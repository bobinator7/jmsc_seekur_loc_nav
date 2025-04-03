# This is a "pro-file", ie a Solution/Project for QMake, Qt's own build system
# see http://qt-project.org/doc/qt-4.8/qmake-project-files.html 
# Comments start with #, as you can see
# The file is extended with commands for SolutionExplorer, VEROSIM's build system based on QMake

# Standard-Options to create DLL
TEMPLATE        =  lib
CONFIG          += dll

# INSERT YOUR PLUGIN NAME BELOW
TARGET          =  VSPluginMarkerLocalization

# Some more default options
include(../../../Main/Global/Defines.pro)
CONFIG          += exceptions

# 3rd party
include($$(VEROSIMSourceFolder)/3rdParty/Eigen/Eigen-3.2.9.pri)

# Input
# When we need to define preprocessor macros, we can do it like this,
# (to export classes via __declspec(dllexport), for example)
DEFINES += EXPORT_VSPluginMarkerLocalization

# List header-files below, end lines with \ to continue
# Remember, this pro-file resides in the /impl-subfolder, so we need to go
# one level up with .. to access headers
HEADERS         =  \
                   ../VSPluginMarkerLocalizationDLLMain.h \
                   ../VSPluginMarkerLocalization2DEKFNode.h \
                   ../VSPluginMarkerLocalization2DUKFNode.h \
                   ../VSPluginMarkerLocalizationInterface.h \
                   ../VSPluginMarkerLocalizationProject.h \
                   ../VSPluginMarkerLocalizationMainWindow.h \
                   ../VSPluginMarkerLocalizationMappingNode.h \
                   ../VSPluginMarkerLocalizationNodeVisu.h \
		   ../VSPluginMarkerLocalizationSensorMergeNode.h \
                   ../VSPluginMarkerLocalizationSimOdomNode.h \
               
# List source-files below, same as above
SOURCES         =  \
                   VSPluginMarkerLocalizationDLLMain.cpp \
                   VSPluginMarkerLocalization2DEKFNode.cpp \
                   VSPluginMarkerLocalization2DUKFNode.cpp \
                   VSPluginMarkerLocalizationInterface.cpp \
                   VSPluginMarkerLocalizationProject.cpp \
                   VSPluginMarkerLocalizationMainWindow.cpp \
                   VSPluginMarkerLocalizationMappingNode.cpp \
                   VSPluginMarkerLocalizationNodeVisu.cpp \
		   VSPluginMarkerLocalizationSensorMergeNode.cpp \
		   VSPluginMarkerLocalizationSimOdomNode.cpp \
   
# End of classic QMake pro-file!
# Now we have a special section for VEROSIM's SolutionExplorer, 
               
#VS_DATA_START

VS_DEPENDENCIES =  \
                   VSD \
                   VSD3D \
                   VSDIO \
                   VSDTools \
                   VSG \
                   VSL \
                   VSM \
                   VSP \
                   VSUI \
                   VSLibGUI \
		   VSLibMarkerLocalization \
                   VSLibMemory \
                   VSLibRenderGL \
                   VSLibSensor \
                   VSLibSensorDataProcessing \
                   VSLibSelection \
                   VSS

VS_DESCRIPTION  =  "VSPluginMarkerLocalization: plugin under construction"
VS_MAINTAINER   =  johnson.loh@rwth-aachen.de
VS_ACTIVE       =  Win32VC9,Win64VC9,Win64VC11,Win64VC14			
#VS_DATA_END

