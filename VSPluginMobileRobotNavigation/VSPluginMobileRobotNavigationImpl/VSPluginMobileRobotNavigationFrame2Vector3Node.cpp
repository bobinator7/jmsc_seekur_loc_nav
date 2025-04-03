// header
#include "../VSPluginMobileRobotNavigationFrame2Vector3Node.h"

// other
#include "../VSPluginMobileRobotNavigationProject.h"

#include "Lib/VSDIO/VSDIOInputVSMFrame.h"
#include "Lib/VSDIO/VSDIOOutputVSMVector3.h"

VSPluginMobileRobotNavigation::Frame2Vector3Node::Frame2Vector3Node(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSLibSensorDataProcessing::SensorDataProcessingNodeBase(mySimState, otherSimStateInstance)
{
}

VSPluginMobileRobotNavigation::Frame2Vector3Node::~Frame2Vector3Node()
{
}

/* management (start) */
VSDIO::ExtensionIOBoard* VSPluginMobileRobotNavigation::Frame2Vector3Node::getIOBoard()
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

void VSPluginMobileRobotNavigation::Frame2Vector3Node::postHasBeenAddedToElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::postHasBeenAddedToElementContainer();

   // abort if no IO-Board or GUI
   ioBoard = getIOBoard();
   if (getMyEnvironment()->getOptions() & VSD::Environment::IsSlave)
      return;
   if (!ioBoard)
      return;

   // initialize inputs
   VSDIO::Input* input = ioBoard->findInputByName("inFrame");
   if (!input)
   {
      input = ioBoard->createInput<VSDIO::InputVSMFrame>("inFrame");
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
   VSDIO::Output* output = ioBoard->findOutputByName("outVec");
   if (!output)
   {
      output = ioBoard->createOutput<VSDIO::OutputVSMVector3>("outVec");
   }

}

void VSPluginMobileRobotNavigation::Frame2Vector3Node::preWillBeRemovedFromElementContainer()
{
   VSLibSensorDataProcessing::SensorDataProcessingNodeBase::preWillBeRemovedFromElementContainer();
}

void VSPluginMobileRobotNavigation::Frame2Vector3Node::slotMyInputValueModified(VSDIO::Input* input)
{
   // get value of the bool input "execute" defined by base class
   if (!getInputValueExecute())
      return;

	//

	VSDIO::Input* in = ioBoard->findInputByName("inFrame");
	QVariant variant_data = in->instanceCast<VSDIO::InputVSMFrame*>()->getVariant();
	VSM::Frame frame = variant_data.value<VSM::Frame>();

   // process the input value
	VSM::Vector3 pos = frame.getPosition();
	VSM::Matrix3x3 ori = frame.getOrientation();

	VSM::Vector3 vecZ(1, 0, 0);
	VSM::Vector3 vecZProjected = ori * vecZ;
	double theta = std::atan2(vecZProjected.getY(), vecZProjected.getX());

	VSM::Vector3 out(pos[0], pos[1], theta);

	VSDIO::Output* output = getIOBoard()->findOutputByName("outVec");
	if (output)
	{
		QVariant outputValue = QVariant::fromValue(out);
		output->setLocalVariant(outputValue);
	}

}
/* management (end) */
void VSPluginMobileRobotNavigation::Frame2Vector3Node::slotEnableModified(VSDIO::Input* input)
{
	// get value of the bool input "execute" defined by base class
	if (!getInputValueExecute())
	{
		//outputData(0., 0.);
		VSDIO::Output* output = getIOBoard()->findOutputByName("outVec");
		if (output)
		{
			QVariant outputValue = QVariant::fromValue(VSM::Vector3(0,0,0));
			output->setLocalVariant(outputValue);
		}
	}
	else
	{
		slotMyInputValueModified(nullptr);
	}
}
