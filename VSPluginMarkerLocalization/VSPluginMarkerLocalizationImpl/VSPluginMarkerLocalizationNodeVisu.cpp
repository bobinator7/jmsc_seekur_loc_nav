// header
#include "../VSPluginMarkerLocalizationNodeVisu.h"

// specifc
#include <Main/VEROSIM/VEROSIMProject.h>
#include <Lib/VSL/VSLArray.h>

#include "../../VSLibRenderGL/VSLibRenderGL.h"
#include "../../VSLibRenderGL/VSLibRenderGLRenderSimStateExtension.h"
#include "../../VSLibRenderGL/VSLibRenderGLFrameBufferObject.h"

#include "../VSPluginMarkerLocalization2DEKFNode.h"
#include "../VSPluginMarkerLocalization2DUKFNode.h"

VSPluginMarkerLocalization::EKFVisu::EKFVisu(VEROSIM::Project* project, VSD::SimStateInstance* caller)
   : VSLibRenderGL::RenderExtension(project, "VSPluginMarkerLocalization::EKFVisu")
   , myCaller(caller)
{
   Q_UNUSED(project);
   addToLayer(VSLibRenderGL::LAYER_OVERLAY);
}

VSPluginMarkerLocalization::EKFVisu::~EKFVisu()
{

}

void VSPluginMarkerLocalization::EKFVisu::initialize()
{
}

void VSPluginMarkerLocalization::EKFVisu::render(VSLibRenderGL::RenderOption option)
{
   Q_UNUSED(option);

	if (myCaller->inherits<Marker2DEKFNode*>())
	{
		Marker2DEKFNode* explicitInstance = myCaller->instanceCast<Marker2DEKFNode*>();
		robotPose_ = explicitInstance->getRobotPose();
		robotCovariance_ = explicitInstance->getRobotCovariance();
		markerPoses_ = explicitInstance->getMarkerPoses();
		markerCovariances_ = explicitInstance->getMarkerCovariances();
	}
	else if (myCaller->inherits<Marker2DUKFNode*>())
	{
		Marker2DUKFNode* explicitInstance = myCaller->instanceCast<Marker2DUKFNode*>();
		robotPose_ = explicitInstance->getRobotPose();
		robotCovariance_ = explicitInstance->getRobotCovariance();
		markerPoses_ = explicitInstance->getMarkerPoses();
		markerCovariances_ = explicitInstance->getMarkerCovariances();
	}
  
   GLdouble glModelView[16];
   glGetDoublev(GL_MODELVIEW_MATRIX, &glModelView[0]);

   glMatrixMode(GL_PROJECTION);
   
	/* start */

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	double m[16];
	VSM::Frame frame = getFrameFromPose(robotPose_);
	frame.copyTo(m);
	glMultMatrixd(m);
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	//foreach detected marker
	renderAxis();
	glPopAttrib(); //GL_LIGHTING_BIT
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	for (auto& cursor : markerPoses_)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		double m[16];
		VSM::Frame frame = getFrameFromPose(cursor.second);
		frame.copyTo(m);
		glMultMatrixd(m);
		glPushAttrib(GL_LIGHTING_BIT);
		glDisable(GL_LIGHTING);
		//foreach detected marker
		renderArrow(0.2,0.5,'r');
		glPopAttrib(); //GL_LIGHTING_BIT
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
	/* end */

   // for safety reasons
   glMatrixMode(GL_MODELVIEW);
}


void VSPluginMarkerLocalization::EKFVisu::renderBox(double edgeLength)
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

void VSPluginMarkerLocalization::EKFVisu::renderAxis()
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

void VSPluginMarkerLocalization::EKFVisu::renderArrow(const float width, const float length, const char col)
{
	glLineWidth(width*25);
	if (col == 'r')
		glColor4f(1.f, 0.f, 0.f, 0.9f);
	else if (col == 'g')
		glColor4f(0.f, 1.f, 0.f, 0.9f);
	else if (col == 'b')
		glColor4f(0.f, 0.f, 1.f, 0.9f);
	else
		glColor4f(0.f, 0.f, 0.f, 0.9f);
	
	glBegin(GL_LINES);
	glVertex3f(-length/2, 0.f, 0.f);
	glVertex3f(length/2, 0.f, 0.f);
	glEnd();
	glLineWidth(1.f);
	glBegin(GL_TRIANGLES);
	glVertex3f(length / 2, width / 2, 0.f);
	glVertex3f(length / 2, -width / 2, 0.f);
	glVertex3f(length, 0.f, 0.f);
	glEnd();

	glLineWidth(1.f);
}

VSM::Frame VSPluginMarkerLocalization::EKFVisu::getFrameFromPose(VSM::Vector3 inPose)
{
	double height = 1;
	VSM::Matrix3x3 orientation = VSM::Matrix3x3(true);
	orientation.setRotZ(inPose[2], true);

	return VSM::Frame(orientation,VSM::Vector3(inPose[0], inPose[1], height));
}