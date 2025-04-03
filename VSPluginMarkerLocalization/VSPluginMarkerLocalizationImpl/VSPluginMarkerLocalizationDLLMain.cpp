// header
#include "../VSPluginMarkerLocalizationDLLMain.h"

// specific
#include "../VSPluginMarkerLocalizationInterface.h"

VEROSIM_DLL_ENTRY(VSPluginMarkerLocalization::Interface)

#include "Lib/VSD/VSDDatabase.h"
#include "Lib/VSD/VSDNode.h"
#include "Lib/VSD/VSDPropertyRefListInternal.h"
#include "Lib/VSD/VSDPropertyRefListInternalElement.h"
#include "Lib/VSD/VSDPropertyRefListIterator.h"
#include "Plugin/VSLibSelection/VSLibSelectionComponentSimStateInstance.h"

#if 0
#include "Lib/VSD/VSDPropertyRefListConstIterator.h"

   // instance creation and deletion
   void instanceCreationAndDeletion(VSD::SimState* simState)
   {
      // sim state instance creation (new operator cannot be used)
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();
      const VSD::MetaInstance* metaInst =VSD::Manager::the()->findMetaInstance("VSD::Node");
      node =metaInst->newSimStateInstance(simState)->instanceCast<VSD::Node*>();

      // instance creation, new operator can be used in derived classes
      VSLibSelection::ComponentSimStateInstance* comp =new VSLibSelection::ComponentSimStateInstance(node, node->getParentProperty());

      // instance deletion
      comp->destroyNow();
      delete comp; // better: VSL_DELETE(comp);

      // sim state instance deletion
      node->removeFromParents(); // normally
      node->destroyNow(); // in rare cases
   }

   // Smart pointer ...
   void smartPointer(VSD::SimState* simState)
   {
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();

      // store two refs
      VSD::Ref<VSD::Node> nodeRef1(node, true);
      VSD::Ref<VSD::Node> nodeRef2(node, true);

      // reset the refs
      nodeRef1=0;
      nodeRef2=0; // node is deleted
   }

   // Type information
   void typeInformation(VSD::SimState* simState)
   {
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();

      // type information
      VSD::Instance* nodeInst =node;
      node =nodeInst->instanceCast<VSD::Node*>();

      bool isNode =nodeInst->isA<VSD::Node*>();
      bool inheritsNode =nodeInst->inherits<VSD::Node*>();

      // meta data
      const VSD::MetaInstance* metaInst =nodeInst->getMetaInstance(); // returns meta instance of VSD::Node
                               metaInst =nodeInst->getThisMetaInstance(); // return meta instance of VSD::Instance
                               metaInst =VSD::Instance::getThisMetaInstance(); // return meta instance of VSD::Instance

      int metaInstIndex =metaInst->getMetaInstanceIndex();
      Q_UNUSED(isNode);
      Q_UNUSED(inheritsNode);
      Q_UNUSED(metaInstIndex);
   }

   // Val properties
   void valProperties(VSD::SimState* simState)
   {
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();

      // using the get and set functions
      QString name =node->getName();
      node->setName(name);

      // using the get and set functions of the property member
      name =node->getDataName()->get();
      node->getDataName()->set(name);
      node->getDataName()->setIfModified(name);

      // variant interface
      QVariant varName =node->getDataName()->getVariant();
      node->getDataName()->setVariant(varName);
      node->getDataName()->setVariantViaInstance(varName); // uses the set function "setName()"
   }

   // RefLists
   void refLists(VSD::SimState* simState)
   {
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();
      VSD::Node* node2 =simState->newSimStateInstance<VSD::Node>();
      VSD::Node* node3 =simState->newSimStateInstance<VSD::Node>();

      //
      node->getChildNodes().append(node2);
      node->getChildNodes().prepend(node2);
      node->getChildNodes().insert(node3, node2);
      node->getChildNodes().remove(node2);

      //
      node->getChildNodes().clear();

      //
      VSD::Node* firstChild =node->getChildNodes().getFirstInstance();
      VSD::Node* lastChild =node->getChildNodes().getLastInstance();

      //
      bool found =node->getChildNodes().contains(node2);

      // please NEVER do this!
      for (int index=0; index < node->getChildNodes().compileCount(); index++)
      {
         qDebug() << node->getChildNodes().findInstance(index)->getName();
      }

      Q_UNUSED(firstChild);
      Q_UNUSED(lastChild);
      Q_UNUSED(found);
   }

   // RefList traversion
   void refListTraversion(VSD::SimState* simState)
   {
      // Traverse all elements in the database (exact class match)
      VSDForAllElements(VSD::Node*, node, simState->getDatabase())
      {
         qDebug() << node->getName();
      }

      // Traverse all elements in the database (specified and derived classes)
      VSDForAllElementsInherits(VSD::Node*, node, simState->getDatabase())
      {
         qDebug() << node->getName();
      }

      // Traverse all instances of a single RefList
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();
      VSDForAllRefListInstances(VSD::Node*, child, node->getChildNodes())
      {
         qDebug() << child->getName();
      }

      // Traverse all properties
      const QList<const VSD::MetaPropertyRefBase*>& propertieslist =node->getMetaInstance()->getAllAutoDelProperties();
      for (QList<const VSD::MetaPropertyRefBase*>::const_iterator it =propertieslist.begin(); it != propertieslist.end(); ++it)
      {
         const VSD::MetaProperty* metaprop =*it;

         // Continue if this is a no-data-property
         VSD::Property* prop =metaprop->getData(node);
         if (prop == 0)
            continue;
         if (prop->toPropertyRefBase()->isEmpty())
            continue;

         // Do something here
         ;
      }
   }

   void refListIteratorAPI(VSD::SimState* simState)
   {
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();

      // VSD::PropertyRefListIterator
      VSD::PropertyRefListIterator<VSD::Node*> it1 =node->getChildNodes().begin();
      VSD::PropertyRefListIterator<VSD::Node*> it2 =node->getChildNodes().begin();
      VSD::PropertyRefListIterator<VSD::Node*> it3 =it2+1;
      VSD::PropertyRefListIterator<VSD::Node*> it4 =it2;
      VSD::PropertyRefListIterator<VSD::Node*> it5 =it4++;
      VSD::PropertyRefListIterator<VSD::Node*> end =node->getChildNodes().end();

      for (VSD::PropertyRefListIterator<VSD::Node*> it =node->getChildNodes().begin(); it != node->getChildNodes().end(); ++it)
      {
         qDebug() << it->getMetaInstance()->getClassName() << it->getName();
      }

      VSD::Node* node2 =0;
      VSD::PropertyRefListWrapperTemplate<VSD::Node>::const_iterator it =qFind(node->getChildNodes(), node2);

      VSD::PropertyRefListIterator<VSD::Node*> it2b =node->getChildNodes().insert(it3, node2);
      VSD::PropertyRefListIterator<VSD::Node*> ret =node->getChildNodes().erase(it2b);
   }

   void streaming(VSD::SimState* simState)
   {
      //
      QTextStream stream;
      VSD::Node* node =simState->newSimStateInstance<VSD::Node>();

      // save property to file
      const VSD::MetaPropertyVal* metaProp =node->getDataName()->getMetaProperty()->toMetaPropertyVal();
      bool ok= metaProp->saveViaInstance(node, stream); 

      // read property from file
      ok= metaProp->loadViaInstance(node, stream);

      // or: use the meta type itself
      const VSD::MetaTypeVal* metaType = node->getDataFlags()->getMetaTypeVal();
      
      QString res;
      VSD::MetaTypeVal::ToStringConversionResult convRes= metaType->toString(node->getDataFlags()->getVariant(), -1, res);

      node->getDataFlags()->setVariant(metaType->fromString(res));
      Q_UNUSED(convRes);
   }
#endif
