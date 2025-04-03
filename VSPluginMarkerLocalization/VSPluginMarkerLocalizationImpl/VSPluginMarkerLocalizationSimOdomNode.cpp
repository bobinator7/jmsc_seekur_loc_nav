// header
#include "../VSPluginMarkerLocalizationSimOdomNode.h"

// other
#include "../VSPluginMarkerLocalizationProject.h"

#include "Lib/VSDIO/VSDIOInputDouble.h"
#include "Lib/VSDIO/VSDIOOutputDouble.h"

VSPluginMarkerLocalization::SimOdomNode::SimOdomNode(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
, dataDoublePropertyWheelDiameter(this)
, dataDoublePropertyWheelDistance(this)
{
}

VSPluginMarkerLocalization::SimOdomNode::~SimOdomNode()
{
}

/* management (start) */
VSDIO::ExtensionIOBoard* VSPluginMarkerLocalization::SimOdomNode::getIOBoard()
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

void VSPluginMarkerLocalization::SimOdomNode::postHasBeenAddedToElementContainer()
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
   VSDIO::Input* input = ioBoard->findInputByName("v_l [rad/sec]");
   if (!input)
   {
      input = ioBoard->createInput<VSDIO::InputDouble>("v_l [rad/sec]");
   }
   connect(input
         , SIGNAL(signalInputValueModified(VSDIO::Input*))
         , this
         , SLOT(slotMyInputValueModified(VSDIO::Input*)));

	input = ioBoard->findInputByName("v_r [rad/sec]");
	if (!input)
	{
		input = ioBoard->createInput<VSDIO::InputDouble>("v_r [rad/sec]");
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
   VSDIO::Output* output = ioBoard->findOutputByName("v_trans");
   if (!output)
   {
      output = ioBoard->createOutput<VSDIO::OutputDouble>("v_trans");
   }

	output = ioBoard->findOutputByName("v_rot");
	if (!output)
	{
		output = ioBoard->createOutput<VSDIO::OutputDouble>("v_rot");
	}

}

void VSPluginMarkerLocalization::SimOdomNode::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();
}

void VSPluginMarkerLocalization::SimOdomNode::slotMyInputValueModified(VSDIO::Input* input)
{
   // get value of the bool input "execute" defined by base class
   if (!getInputValueExecute())
      return;

	//
	double vl, vr, vtrans, vrot;
	VSDIO::Input* in = ioBoard->findInputByName("v_l [rad/sec]");
	QVariant variant_data = in->instanceCast<VSDIO::InputDouble*>()->getVariant();
	vl = variant_data.value<double>();

	in = ioBoard->findInputByName("v_r [rad/sec]");
	variant_data = in->instanceCast<VSDIO::InputDouble*>()->getVariant();
	vr = variant_data.value<double>();

	vtrans = (vl + vr) / 2 * getDoublePropertyWheelDiameter();
	vrot = (vr - vl) / 2 * (getDoublePropertyWheelDiameter()) / getDoublePropertyWheelDistance();

   // process the input value
	outputData(vtrans, vrot);

}
/* management (end) */

void VSPluginMarkerLocalization::SimOdomNode::outputData(double vTrans, double vRot)
{
   VSDIO::Output* output = getIOBoard()->findOutputByName("v_trans");
   if (output)
   {
      QVariant outputValue = QVariant::fromValue(vTrans);
      output->setLocalVariant(outputValue);
   }

   output = getIOBoard()->findOutputByName("v_rot");
	if (output)
	{
		QVariant outputValue = QVariant::fromValue(vRot);
		output->setLocalVariant(outputValue);
	}
}

void VSPluginMarkerLocalization::SimOdomNode::slotEnableModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
	{
		outputData(0., 0.);
	}
}
