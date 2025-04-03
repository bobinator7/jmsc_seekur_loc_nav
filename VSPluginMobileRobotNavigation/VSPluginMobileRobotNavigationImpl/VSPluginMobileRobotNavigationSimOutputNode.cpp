// header
#include "../VSPluginMobileRobotNavigationSimOutputNode.h"

// other
#include "../VSPluginMobileRobotNavigationProject.h"

#include "Lib/VSDIO/VSDIOInputDouble.h"
#include "Lib/VSDIO/VSDIOOutputDouble.h"

VSPluginMobileRobotNavigation::SimOutputNode::SimOutputNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataDoublePropertyWheelDiameter(this)
, dataDoublePropertyWheelDistance(this)
{
}

VSPluginMobileRobotNavigation::SimOutputNode::~SimOutputNode()
{
}

/* management (start) */
VSDIO::ExtensionIOBoard* VSPluginMobileRobotNavigation::SimOutputNode::getIOBoard()
{
   if (ioBoard == 0)
   {
      ioBoard = findFirstExtension<VSDIO::ExtensionIOBoard*>();
      if (ioBoard == 0)
      {
         ioBoard = getMySimState()->newSimStateInstance<VSDIO::ExtensionIOBoard>();
         this->getExtensions().append(ioBoard);
      }
   }
   return ioBoard;
}

void VSPluginMobileRobotNavigation::SimOutputNode::postHasBeenAddedToElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   // abort if no IO-Board or GUI
   ioBoard = getIOBoard();
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;
   if (!ioBoard)
      return;

   // property modified functions
   //connect(this
   //   , SIGNAL(signalPropertyModified(VSD::SimStateInstance*, const VSD::MetaProperty*))
   //   , this
   //   , SLOT(slotPropertiesModified(VSD::SimStateInstance*, const VSD::MetaProperty*)));
   
   // initialize properties
	dataDoublePropertyWheelDiameter.set(0.2);
	dataDoublePropertyWheelDistance.set(0.6);

   // initialize inputs
   VSDIO::Input* input = ioBoard->findInputByName("v_trans");
   if (!input)
   {
      input = ioBoard->createInput<VSDIO::InputDouble>("v_trans");
   }
   connect(input
         , SIGNAL(signalInputValueModified(VSDIO::Input*))
         , this
         , SLOT(slotMyInputValueModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("v_rot");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputDouble>("v_rot");
	}
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotMyInputValueModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("execute");
	connect(input
		, SIGNAL(signalInputValueModified(VSDIO::Input*))
		, this
		, SLOT(slotEnableModified(VSDIO::Input*)));

   // initialize outputs
   VSDIO::Output* output = ioBoard->findOutputByName("v_l [rad/sec]");
   if (!output)
   {
      output = ioBoard->createOutput<VSDIO::OutputDouble>("v_l [rad/sec]");
   }

	output = ioBoard->findOutputByName("v_r [rad/sec]");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputDouble>("v_r [rad/sec]");
	}

}

void VSPluginMobileRobotNavigation::SimOutputNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();
}

void VSPluginMobileRobotNavigation::SimOutputNode::slotMyInputValueModified(VSDIO::Input* input)
{
   // get value of the bool input "execute" defined by base class
   if (!getInputValueExecute())
      return;

	//
	double vl, vr, vtrans, vrot;
	VSDIO::Input* in = ioBoard->findInputByName("v_trans");
	QVariant variant_data = in->instanceCast<VSDIO::InputDouble*>()->getVariant();
	vtrans = variant_data.value<double>();

	in = ioBoard->findInputByName("v_rot");
	variant_data = in->instanceCast<VSDIO::InputDouble*>()->getVariant();
	vrot = variant_data.value<double>();

	double v_mean, v_diff;
	v_mean = vtrans / getDoublePropertyWheelDiameter();
	v_diff = vrot / getDoublePropertyWheelDiameter() * getDoublePropertyWheelDistance() * 2;

	vl = v_mean - v_diff;
	vr = v_mean + v_diff;


   // process the input value
	outputData(vl, vr);

}
/* management (end) */

void VSPluginMobileRobotNavigation::SimOutputNode::outputData(double vl, double vr)
{
   VSDIO::Output* output = getIOBoard()->findOutputByName("v_l [rad/sec]");
   if (output)
   {
      QVariant outputValue = QVariant::fromValue(vl);
      output->setLocalVariant(outputValue);
   }

   output = getIOBoard()->findOutputByName("v_r [rad/sec]");
	if (output)
	{
		QVariant outputValue = QVariant::fromValue(vr);
		output->setLocalVariant(outputValue);
	}
}

void VSPluginMobileRobotNavigation::SimOutputNode::slotEnableModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
	{
		outputData(0., 0.);
	}
	else
	{
		slotMyInputValueModified(nullptr);
	}
}
