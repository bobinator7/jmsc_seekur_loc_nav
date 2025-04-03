// header
#include "../VSLibARMarkerBoxVisu.h"

// specifc
#include <Main/VEROSIM/VEROSIMProject.h>
#include <Lib/VSL/VSLArray.h>

#include "../../VSLibRenderGL/VSLibRenderGL.h"
#include "../../VSLibRenderGL/VSLibRenderGLRenderSimStateExtension.h"
#include "../../VSLibRenderGL/VSLibRenderGLFrameBufferObject.h"

#include "../VSLibARMarkerLocalizationNode.h"
#include "../VSLibARMarkerExtensionMarkerVisu.h"
#include "../VSLibARMarkerTools.h"

VSLibARMarker::BoxVisu::BoxVisu(VEROSIM::Project* project, VSD::SimStateInstance* caller)
   : VSLibRenderGL::RenderExtension(project, "VSLibARMarker::BoxVisu")
   , myCaller(caller)
{
   Q_UNUSED(project);
   addToLayer(VSLibRenderGL::LAYER_OVERLAY);
}

//VSLibARMarker::BoxVisu::BoxVisu(VEROSIM::Project* project, LocalizationNode* callerNode)
//: VSLibRenderGL::RenderExtension(project, "VSLibARMarker::BoxVisu")
//, myCallerNode(callerNode)
//, myCallerExtension(0)
//{
//   Q_UNUSED(project);
//   addToLayer(VSLibRenderGL::LAYER_OVERLAY);
//}
//
//VSLibARMarker::BoxVisu::BoxVisu(VEROSIM::Project* project, ExtensionMarkerVisu* callerExtension)
//: VSLibRenderGL::RenderExtension(project, "VSLibARMarker::BoxVisu")
//, myCallerNode(0)
//, myCallerExtension(callerExtension)
//{
//   Q_UNUSED(project);
//   addToLayer(VSLibRenderGL::LAYER_OVERLAY);
//}

VSLibARMarker::BoxVisu::~BoxVisu()
{
}


void VSLibARMarker::BoxVisu::initialize()
{
}


void VSLibARMarker::BoxVisu::render(VSLibRenderGL::RenderOption option)
{
   Q_UNUSED(option);
   MarkerDetections markers;
   double markerEdgeLength;
   VSLibSensor::CameraExtensionBase* cameraExtension;
   bool showBox;
   bool showAxis;
   if (myCaller->inherits<ExtensionMarkerVisu*>())
   {
      ExtensionMarkerVisu* explicitInstance = myCaller->instanceCast<ExtensionMarkerVisu*>();
      if (!explicitInstance->getShowBoxVisu() && !explicitInstance->getShowAxisVisu())
         return;
      if (!explicitInstance->isActive())
         return;
      cameraExtension = explicitInstance->getCameraExtension();
      if (!cameraExtension)
         return;

      markers = explicitInstance->getVisibleMarkers();
      markerEdgeLength = explicitInstance->getMarkerEdgeLength();
      showBox = explicitInstance->getShowBoxVisu();
      showAxis = explicitInstance->getShowAxisVisu();
   }
   else if (myCaller->inherits<LocalizationNode*>())
   {
      LocalizationNode* explicitInstance = myCaller->instanceCast<LocalizationNode*>();
      if (!explicitInstance->getShowBoxVisu() && !explicitInstance->getShowAxisVisu())
         return;
      if (!explicitInstance->isActive())
         return;
      cameraExtension = explicitInstance->getCameraExtension();
      if (!cameraExtension)
         return;

      markers = explicitInstance->getVisibleMarkers();
      markerEdgeLength = explicitInstance->getMarkerEdgeLength();
      showBox = explicitInstance->getShowBoxVisu();
      showAxis = explicitInstance->getShowAxisVisu();
   }

   if (markers.empty())
      return;

   GLdouble glModelView[16];
   glGetDoublev(GL_MODELVIEW_MATRIX, &glModelView[0]);

   glMatrixMode(GL_PROJECTION);
   
   for (int i=0; i<markers.size(); ++i)
   {
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();

      double m[16];
      VSM::Frame frame = Tools::getMarkerWorldFrame(markers.tvecs.at(i), markers.rvecs.at(i), cameraExtension->getParentVSD3DNode()->getWorldFrame());
      frame.copyTo(m);
      glMultMatrixd(m);

      glPushAttrib(GL_LIGHTING_BIT);
      glDisable(GL_LIGHTING);

      //foreach detected marker
      
      if (showBox)
         renderBox(markerEdgeLength);
      if (showAxis)
         renderAxis();

      glPopAttrib(); //GL_LIGHTING_BIT

      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
   }

      // for safety reasons
   glMatrixMode(GL_MODELVIEW);
}


void VSLibARMarker::BoxVisu::renderBox(double edgeLength)
{
   float c=edgeLength/2;
   glColor4f(1.f, 0.f, 0.f, 0.9f);
   glLineWidth(2.f);
   glBegin(GL_LINES);
      glVertex3f(-c, 0, c);
      glVertex3f(c, 0, c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(c, 0, c);
      glVertex3f(c, 0, -c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(c, 0, -c);
      glVertex3f(-c, 0, -c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(-c, 0, -c);
      glVertex3f(-c, 0, c);
   glEnd();

   glBegin(GL_LINES);
      glVertex3f(-c, 0, c);
      glVertex3f(-c, edgeLength, c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(c, 0, c);
      glVertex3f(c, edgeLength, c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(c, 0, -c);
      glVertex3f(c, edgeLength, -c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(-c, 0, -c);
      glVertex3f(-c, edgeLength, -c);
   glEnd();

   glBegin(GL_LINES);
      glVertex3f(-c, edgeLength, c);
      glVertex3f(c, edgeLength, c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(c, edgeLength, c);
      glVertex3f(c, edgeLength, -c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(c, edgeLength, -c);
      glVertex3f(-c, edgeLength, -c);
   glEnd();
   glBegin(GL_LINES);
      glVertex3f(-c, edgeLength, -c);
      glVertex3f(-c, edgeLength, c);
   glEnd();
   glLineWidth(1.f);
}

void VSLibARMarker::BoxVisu::renderAxis()
{
   glLineWidth(2.f);
   glColor4f(1.f, 0.f, 0.f, 0.9f);
   glBegin(GL_LINES);
   glVertex3f(0.f, 0.f, 0.f);
   glVertex3f(1.f, 0.f, 0.f);
   glEnd();
   
   glColor4f(0.f, 1.f, 0.f, 0.9f);
   glBegin(GL_LINES);
   glVertex3f(0.f, 0.f, 0.f);
   glVertex3f(0.f, 1.f, 0.f);
   glEnd();

   glColor4f(0.f, 0.f, 1.f, 0.9f);
   glBegin(GL_LINES);
   glVertex3f(0.f, 0.f, 0.f);
   glVertex3f(0.f, 0.f, 1.f);
   glEnd();
   glLineWidth(1.f);
}