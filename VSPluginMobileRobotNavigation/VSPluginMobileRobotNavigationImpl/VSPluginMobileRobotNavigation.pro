# This is a "pro-file", ie a Solution/Project for QMake, Qt's own build system
# see http://qt-project.org/doc/qt-4.8/qmake-project-files.html 
# Comments start with #, as you can see
# The file is extended with commands for SolutionExplorer, VEROSIM's build system based on QMake

# Standard-Options to create DLL
TEMPLATE        =  lib
CONFIG          += dll

# INSERT YOUR PLUGIN NAME BELOW
TARGET          =  VSPluginMobileRobotNavigation

# Some more default options
include(../../../Main/Global/Defines.pro)
CONFIG          += exceptions

# Input
# When we need to define preprocessor macros, we can do it like this,
# (to export classes via __declspec(dllexport), for example)
# DEFINES += EXPORT_VSPluginMobileRobotNavigation

# List header-files below, end lines with \ to continue
# Remember, this pro-file resides in the /impl-subfolder, so we need to go
# one level up with .. to access headers
HEADERS         =  \
                   ../VSPluginMobileRobotNavigationDLLMain.h \
                   ../VSPluginMobileRobotNavigationCollisionAvoidanceNode.h \
                   ../VSPluginMobileRobotNavigationInterface.h \
                   ../VSPluginMobileRobotNavigationProject.h \
                   ../VSPluginMobileRobotNavigationMainWindow.h \
                   ../VSPluginMobileRobotNavigationNodeVisu.h \
                   ../VSPluginMobileRobotNavigationSecurityFilterNode.h \
                   ../VSPluginMobileRobotNavigationSimOutputNode.h \
                   ../VSPluginMobileRobotNavigationFrame2Vector3Node.h \
               
# List source-files below, same as above
SOURCES         =  \
                   VSPluginMobileRobotNavigationDLLMain.cpp \
                   VSPluginMobileRobotNavigationCollisionAvoidanceNode.cpp \
                   VSPluginMobileRobotNavigationInterface.cpp \
                   VSPluginMobileRobotNavigationProject.cpp \
                   VSPluginMobileRobotNavigationMainWindow.cpp \
                   VSPluginMobileRobotNavigationNodeVisu.cpp \
                   VSPluginMobileRobotNavigationSecurityFilterNode.cpp \
                   VSPluginMobileRobotNavigationSimOutputNode.cpp \
                   VSPluginMobileRobotNavigationFrame2Vector3Node.cpp \
   
#Translation file for multiple languages   
#TRANSLATIONS    =  \
#                  VSPluginMobileRobotNavigation_de.ts

# End of classic QMake pro-file!
# Now we have a special section for VEROSIM's SolutionExplorer, 
               
#VS_DATA_START

VS_DEPENDENCIES =  \
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
                   VSLibRenderGL \
                   VSLibSensor \
                   VSLibSensorDataProcessing \
                   VSLibSelection \
                   VSLibMarkerLocalization \
                   VSLibMobileRobotNavigation \
                   VSS \

VS_DESCRIPTION  =  "VSPluginMobileRobotNavigation: plugin for navigation of mobile robots"
VS_MAINTAINER   =  johnson.loh@rwth-aachen.de
VS_ACTIVE       =  Win64VC14

#VS_DATA_END

