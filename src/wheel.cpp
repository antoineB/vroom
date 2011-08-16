#include "wheel.hpp"
#include "world.hpp"
#include "utils.hpp"

#include <OGRE/OgreMath.h>

Wheel::Wheel(): type(Type::UNDEFINED) {}


void Wheel::update() {
  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);
}


Wheel::~Wheel() {}


void Wheel::reset() {
  dSpaceID space = dGeomGetSpace(ph.geom);
  dGeomDestroy(ph.geom);
  dBodyDestroy(ph.body);

  Utils::Xml x(cst.xmlFile, "wheel");
  
  createPhysics(x, space);
  disposePhysics(x);
}


void Wheel::createPhysics(Utils::Xml &x, dSpaceID space) {

  if (x.mustString("nature") == "sphere") {
    ph.geom = dCreateSphere(space, x.mustOReal("radius"));
    dMassSetSphereTotal(&ph.mass, x.mustOReal("mass"), x.mustOReal("radius"));    
  }
  else {
    ph.geom = dCreateCylinder(space, x.mustOReal("radius"), x.mustOReal("width"));
    dMassSetCylinderTotal(&ph.mass, x.mustOReal("mass"), 1, x.mustOReal("radius"), x.mustOReal("width"));    
  }
  
  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);
}


void Wheel::createNodesAndMesh(Utils::Xml &x) {
  Ogre::Entity *e;
  Ogre::SceneNode *n;

    n  = sceneMgr_->getRootSceneNode()->createChildSceneNode(cst.nodeName);
  cst.wheelNode = n;
  

  e = sceneMgr_->createEntity(cst.nodeName+"t", x.mustString("tire-mesh"));
  e->setMaterialName(x.mustString("tire-material"));
  e->setCastShadows(true);
  n->attachObject(e);

  e = sceneMgr_->createEntity(cst.nodeName+"h", x.mustString("hubcap-mesh"));
  e->setMaterialName(x.mustString("hubcap-material"));
  n->attachObject(e);

  n->scale(x.mustOReal("scale.x"), x.mustOReal("scale.y"), x.mustOReal("scale.z"));
}


void Wheel::disposePhysics(Utils::Xml &x) {
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, 
		       Ogre::Degree(x.mustOReal("rotation.y")).valueRadians());
    dBodySetRotation(ph.body, R);
    
    Utils::Xml w("../xml/car.xml", "car");
    dGeomSetPosition(ph.geom, 
		     w.mustOReal("global-position.x") + x.mustOReal("position.x"), 
		     w.mustOReal("global-position.y") + x.mustOReal("position.y"), 
		     w.mustOReal("global-position.z") + x.mustOReal("position.z")
		     );
}


void Wheel::initXml(const char* xmlFile, dSpaceID space) {
  Utils::Xml x(xmlFile, "wheel");
  
  cst.xmlFile = xmlFile;
  cst.nodeName = x.mustString("name");
  
  createNodesAndMesh(x);
  createPhysics(x, space);

  disposePhysics(x);
  
  setupContact(x);

  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);

}

void Wheel::setupContact(Utils::Xml &x) {
	x.fillDContact("contact", this->type);
	dGeomSetData(ph.geom, &(this->type));
}
