// header
#include "../VSPluginMobileRobotNavigationNodeVisu.h"

// specifc
#include <Main/VEROSIM/VEROSIMProject.h>
#include <Lib/VSL/VSLArray.h>

#include "../../VSLibRenderGL/VSLibRenderGL.h"
#include "../../VSLibRenderGL/VSLibRenderGLPrimitiveCylinder.h"

#include "../VSPluginMobileRobotNavigationCollisionAvoidanceNode.h"

VSPluginMobileRobotNavigation::CollavVisu::CollavVisu(VEROSIM::Project* project, VSD::SimStateInstance* caller)
   : VSLibRenderGL::RenderExtension(project, "VSPluginMobileRobotNavigation::CollavVisu")
   , myCaller(caller)
{
   Q_UNUSED(project);
   addToLayer(VSLibRenderGL::LAYER_OVERLAY);
}

VSPluginMobileRobotNavigation::CollavVisu::~CollavVisu()
{
}

void VSPluginMobileRobotNavigation::CollavVisu::initialize()
{
}

void VSPluginMobileRobotNavigation::CollavVisu::render(VSLibRenderGL::RenderOption option)
{
   Q_UNUSED(option);

	if (myCaller->inherits<CollisionAvoidanceNode*>())
	{
		CollisionAvoidanceNode* explicitInstance = myCaller->instanceCast<CollisionAvoidanceNode*>();
		robotPose_ = explicitInstance->getRobotPose();
		collavPointCloudInRobotFrame_ = explicitInstance->getCollavPointCloud();
		laserPointCloudInRobotFrame_ = explicitInstance->getLaserPointCloud();
		collPoints_ = explicitInstance->getCollisionPoints();
      intersectPoints_ = explicitInstance->getIntersectionPoints();
      vecRadius_ = explicitInstance->getVecRadius();
	}

   /* start */
	glPushAttrib(GL_ENABLE_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	renderPointCloud();
   renderMaxRadialPaths();

	glLineWidth(1);
	glPopAttrib();

	/* end */

   // for safety reasons
   //glMatrixMode(GL_MODELVIEW);
}

void VSPluginMobileRobotNavigation::CollavVisu::renderMaxRadialPaths()
{
   GLdouble glModelView[16];
   glGetDoublev(GL_MODELVIEW_MATRIX, &glModelView[0]);
   glMatrixMode(GL_PROJECTION);
   glMatrixMode(GL_MODELVIEW);
   glPushAttrib(GL_LIGHTING_BIT);
   glDisable(GL_LIGHTING);

   glPushMatrix();
   VSM::Frame robotFrame = getFrameFromPose(robotPose_, 3);
   double m[16];
   robotFrame.copyTo(m);
   glMultMatrixd(m);

   glLineWidth(2.f);
   glColor4f(1.f, 0.f, 1.f, 0.9f);

   // visualize relation collpoints to intersectpoints
   for (int ii = 0; ii < collPoints_.size(); ii++)
   {
      glBegin(GL_LINES);
      glVertex3f(collPoints_.at(ii).first, collPoints_.at(ii).second, 0.f);
      glVertex3f(intersectPoints_.at(ii).first, intersectPoints_.at(ii).second, 0.f);
      glEnd();
   }

   glPopAttrib();
   glMatrixMode(GL_MODELVIEW);
   glPopMatrix();
}

void VSPluginMobileRobotNavigation::CollavVisu::renderPointCloud()
{
	VSM::Frame robotFrame = getFrameFromPose(robotPose_);

	// visualize point cloud used for collision avoidance
	for (int ii = 0; ii < collavPointCloudInRobotFrame_.size(); ii++)
	{
		double height = 1.0;
		double size = 0.02;
		QColor col(0, 255, 0);
		int approx = 21; // 3 

		VSLibRenderGL::Primitive::Cylinder primitive(height, size, col, approx);

		double x = collavPointCloudInRobotFrame_.at(ii).first;
		double y = collavPointCloudInRobotFrame_.at(ii).second;
		VSM::Vector3 vsmPoint = robotFrame.trafo(VSM::Vector3(x, y, 0.0));
		VSM::Frame pointFrame;
		pointFrame.setPosition(VSM::Vector3(vsmPoint.getX(), vsmPoint.getY(), vsmPoint.getZ()));

		primitive.drawGL(pointFrame);

	}

	// visualize points colliding
	for (int ii = 0; ii < collPoints_.size(); ii++)
	{
		double height = 2.0;
		double size = 0.025;
		QColor col(255, 0, 0);
		int approx = 21; // 3 

		VSLibRenderGL::Primitive::Cylinder primitive(height, size, col, approx);
      

		double x = collPoints_.at(ii).first;
		double y = collPoints_.at(ii).second;
		VSM::Vector3 vsmPoint = robotFrame.trafo(VSM::Vector3(x, y, 0.0));
		VSM::Frame pointFrame;
		pointFrame.setPosition(VSM::Vector3(vsmPoint.getX(), vsmPoint.getY(), vsmPoint.getZ()));

		primitive.drawGL(pointFrame);

	}
}

VSM::Frame VSPluginMobileRobotNavigation::CollavVisu::getFrameFromPose(VSM::Vector3 inPose, double height)
{
	VSM::Matrix3x3 orientation = VSM::Matrix3x3(true);
	orientation.setRotZ(inPose[2], true);

	return VSM::Frame(orientation,VSM::Vector3(inPose[0], inPose[1], height));
}