// header
#include "../VSLibARMarkerNode.h"

#include "Lib/VSD/VSDModelNode.h"
#include "Lib/VSD3D/VSD3DGeometryFace.h"

#include <fstream>

VSLibARMarker::Node::Node(VSD::SimState* mySimState, VSD::SimStateInstance* otherSimStateInstance)
: VSD3D::Node(mySimState, otherSimStateInstance)
, mySimState(mySimState)
, dataId(this)
, dataMarkerEdgeLength(this)
{
}

VSLibARMarker::Node::~Node()
{
}

void VSLibARMarker::Node::slotPrintStaticMapFile()
{
	std::string abspath("E:\\VEROSIM\\002\\Modelle\\John_MA\\Simulation_Flur\\map\\new.csv");

	VSD::ModelNode* parentModelNode = getMyModel();
	if (!parentModelNode)
		return;

	std::ofstream file(abspath, std::ios::out);

	int id = 0;
	double x, y, theta = 0.0;
	VSDForAllRecursiveChildrenInherits(VSLibARMarker::Node*, markerNode, parentModelNode)
	{
		id = markerNode->getId();

		//VSD3D::HullNode* faceNode = markerNode->findFirstChildNodeInherits<VSD3D::HullNode*>();
		VSM::Frame frame = markerNode->getRelFrame();

		VSM::Vector3 pos = frame.getPosition();
		VSM::Matrix3x3 ori = frame.getOrientation();
		x = pos.getX();
		y = pos.getY();
		
		VSM::Vector3 vecY(0, 1, 0);
		VSM::Vector3 vecYProjected = ori * vecY;
		theta = std::atan2(vecYProjected.getY(), vecYProjected.getX());
		
		file << id << " " << x << " " << y << " " << theta << "\n";
	}
}

void VSLibARMarker::Node::slotMakeARMarkerNodeUnique()
{
	VSD::ModelNode* parentModelNode = getMyModel();
   if (!parentModelNode)
      return;

   QList<int> usedIndices;
   double markerEdgeLength = 0.3;
	VSD3D::MaterialNode* markerMaterial = findFirstChildNodeInherits<VSD3D::MaterialNode*>();
   VSDForAllRecursiveChildrenInherits(VSLibARMarker::Node*, markerNode, parentModelNode)
   {
		if (markerNode == this)
			continue;

      usedIndices.append(markerNode->getId());

      if (markerMaterial == 0)
      {
         markerMaterial = markerNode->findFirstChildNodeInherits<VSD3D::MaterialNode*>();
         markerEdgeLength = markerNode->getMarkerEdgeLength();
      }
         
   }

   int id = 0;
   while (usedIndices.contains(id) && id < 1024)
      id++;
   if (id > 1023)
      return;
   setId(id);

	// adjust face
   VSD3D::HullNode* faceNode = findFirstChildNodeInherits<VSD3D::HullNode*>();
	if (faceNode)
	{
		faceNode->setName(QString("Marker%1").arg(getId()));
		faceNode->setShow(true);
		//getChildNodes().append(faceNode);

		// construct geometry
		const VSD::MetaProperty* childProperty = faceNode->getMetaInstance()->findMyOrBaseClassProperty("geometry");
		if (!childProperty)
			return;

		VSD3D::GeometryFace* newGeometryFace = mySimState->newSimStateInstance<VSD3D::GeometryFace>();
		if (!faceNode)
			return;

		double edgeLength = getMarkerEdgeLength();
		newGeometryFace->setSize(VSM::Vector2(edgeLength, edgeLength));
		faceNode->setGeometry(newGeometryFace);

		VSM::Frame relFrame = VSM::Frame(VSM::Matrix3x3(180, 0, 90), VSM::Vector3(edgeLength / 2, 0, -edgeLength / 2));

		faceNode->setRelFrame(relFrame);
	}
	else
	{
	   faceNode = createNewMarkerFace();
	}




	// adjust material
	markerMaterial = findFirstChildNodeInherits<VSD3D::MaterialNode*>();
	if (!markerMaterial)
	{
		markerMaterial = createNewMarkerMaterial();
		faceNode->setMaterial(markerMaterial);
	}   

	// adjust texture
   VSD3D::MapNode* markerTexture = findFirstChildNodeInherits<VSD3D::MapNode*>();
	if (markerTexture)
	{
		markerTexture->setName(QString("MarkerTexture%1").arg(getId()));
		markerTexture->setUrl(QString("textures/marker%1.jpg").arg(getId()));
		//getChildNodes().append(markerTexture);
	}
	else
	{
		markerTexture = createNewMarkerTexture();
		QList<VSD::SimStateInstance*> maps;
		maps.append(markerTexture);
		faceNode->setMaps(maps);
	}

	// adjust landmark
   VSLibARMarker::LandmarkExtension* lmExtension = findFirstExtensionInherits<VSLibARMarker::LandmarkExtension*>();
	if (lmExtension)
	{
		lmExtension->setId(getId());
		lmExtension->setEdgeLength(getMarkerEdgeLength());
		//getExtensions().append(lmExtension);
	}
	else
	{
		lmExtension = createLMExtension();
	}

   //attachedToWorld();
}

void VSLibARMarker::Node::postHasBeenAddedToElementContainer()
{

   VSD::ModelNode* parentModelNode = getMyModel();
   if (!parentModelNode)
      return;

   VSD3D::MaterialNode* markerMaterial = findFirstChildNodeInherits<VSD3D::MaterialNode*>();

   if (getId() < 0)
   {
      QList<int> usedIndices;
      double markerEdgeLength = 0.3;
      VSDForAllRecursiveChildrenInherits(VSLibARMarker::Node*, markerNode, parentModelNode)
      {
         usedIndices.append(markerNode->getId());

         if (markerMaterial == 0)
         {
            markerMaterial = markerNode->findFirstChildNodeInherits<VSD3D::MaterialNode*>();
            markerEdgeLength = markerNode->getMarkerEdgeLength();
         }
         
      }

      int id = 0;
      while (usedIndices.contains(id) && id < 1024)
         id++;
      if (id > 1023)
         return;
      setId(id);
   }

   VSD3D::HullNode* faceNode = findFirstChildNodeInherits<VSD3D::HullNode*>();
   if (!faceNode)
      faceNode = createNewMarkerFace();
   if (!faceNode)
      return;
   if (!markerMaterial)
      markerMaterial = createNewMarkerMaterial();
   if (!markerMaterial)
      return;
   faceNode->setMaterial(markerMaterial);
   VSD3D::MapNode* markerTexture = findFirstChildNodeInherits<VSD3D::MapNode*>();
   if (!markerTexture)
      markerTexture = createNewMarkerTexture();
   if (!markerTexture)
      return;
   QList<VSD::SimStateInstance*> maps;
   maps.append(markerTexture);
   faceNode->setMaps(maps);

   VSLibARMarker::LandmarkExtension* lmExtension = findFirstExtensionInherits<VSLibARMarker::LandmarkExtension*>();
   if (!lmExtension)
      lmExtension = createLMExtension();

   attachedToWorld();
}

VSD3D::HullNode* VSLibARMarker::Node::createNewMarkerFace()
{
	VSD3D::HullNode* newHullNode = mySimState->newSimStateInstance<VSD3D::HullNode>();
   if (!newHullNode)
      return 0;

   newHullNode->setName(QString("Marker%1").arg(getId()));
   newHullNode->setShow(true);
   getChildNodes().append(newHullNode);

   // construct geometry
   const VSD::MetaProperty* childProperty = newHullNode->getMetaInstance()->findMyOrBaseClassProperty("geometry");
   if (!childProperty)
      return 0;
   
   VSD3D::GeometryFace* newGeometryFace = mySimState->newSimStateInstance<VSD3D::GeometryFace>();
   if (!newHullNode)
      return 0;

   double edgeLength = getMarkerEdgeLength();
   newGeometryFace->setSize(VSM::Vector2(edgeLength, edgeLength));
   newHullNode->setGeometry(newGeometryFace);

   VSM::Frame relFrame = VSM::Frame(VSM::Matrix3x3(180, 0, 90), VSM::Vector3(edgeLength/2, 0, -edgeLength/2));
   
   newHullNode->setRelFrame(relFrame);
   
   return newHullNode;
}

VSD3D::MaterialNode* VSLibARMarker::Node::createNewMarkerMaterial()
{
   VSD3D::MaterialNode* materialNode = mySimState->newSimStateInstance<VSD3D::MaterialNode>();
   if (!materialNode)
      return 0;

   materialNode->setName("MarkerMaterial");
   materialNode->setAmbient(QColor(255, 255, 255));
   materialNode->setDiffuse(QColor(255, 255, 255));

   getChildNodes().append(materialNode);

   return materialNode;
}

VSD3D::MapNode* VSLibARMarker::Node::createNewMarkerTexture()
{
   VSD3D::MapNode* markerTexture = mySimState->newSimStateInstance<VSD3D::MapNode>();
   if (!markerTexture)
      return 0;

   markerTexture->setName(QString("MarkerTexture%1").arg(getId()));
   markerTexture->setUrl(QString("textures/marker%1.jpg").arg(getId()));
   markerTexture->setMinFilterType(VSD3D::MapNode::MipmapNearest);
   markerTexture->setMagFilterType(VSD3D::MapNode::MipmapNearest);
   markerTexture->setDisableTextureCompression(true);
   getChildNodes().append(markerTexture);

   return markerTexture;
}

VSLibARMarker::LandmarkExtension* VSLibARMarker::Node::createLMExtension()
{
   VSLibARMarker::LandmarkExtension* lmExtension = mySimState->newSimStateInstance<VSLibARMarker::LandmarkExtension>();
   if (!lmExtension)
      return 0;
   lmExtension->setId(getId());
   lmExtension->setEdgeLength(getMarkerEdgeLength());
   getExtensions().append(lmExtension);
   return lmExtension;
}

void VSLibARMarker::Node::preWillBeRemovedFromElementContainer()
{
   //call base class method
   //VSD3D::Node::preWillBeRemovedFromElementContainer();
}
