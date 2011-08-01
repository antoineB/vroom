#include "wheel.hpp"
#include "world.hpp"
#include "utils.hpp"

#include <OGRE/OgreMath.h>

using namespace Utils;


DContactType Wheel::type(Type::CAR_WHEEL);


Wheel::Wheel(){}


void Wheel::fillContact() {
  Wheel::type.contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  Wheel::type.contact.surface.mu = dInfinity;
  Wheel::type.contact.surface.bounce = 1.0;
  Wheel::type.contact.surface.bounce_vel = 0.1;
  Wheel::type.contact.surface.soft_cfm = 0.01;  
  Wheel::type.contact.surface.soft_erp = 0.3;  
  Wheel::type.contact.surface.slip1 = 0.5;
  Wheel::type.contact.surface.slip2 = 0.5;
}

void Wheel::fillContact(dContact &mod) {
  memcpy(&Wheel::type.contact.surface.mu, &mod.surface.mu, sizeof(mod.surface) - sizeof(mod.surface.mode));
}


void Wheel::update() {
  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);
}


Wheel::~Wheel() {}


void Wheel::reset() {
  dSpaceID space = dGeomGetSpace(ph.geom);
  dGeomDestroy(ph.geom);
  dBodyDestroy(ph.body);

  Xml::begin(cst.xmlFile, "wheel");
  
  createPhysicsXml(space);
  disposePhysicsXml();

  Xml::end();
}


void Wheel::createPhysicsXml(dSpaceID space) {

  if (Xml::mustString("nature") == "sphere") {
    ph.geom = dCreateSphere(space, Xml::mustOReal("radius"));
    dMassSetSphereTotal(&ph.mass, Xml::mustOReal("mass"), Xml::mustOReal("radius"));    
  }
  else {
    ph.geom = dCreateCylinder(space, Xml::mustOReal("radius"), Xml::mustOReal("width"));
    dMassSetCylinderTotal(&ph.mass, Xml::mustOReal("mass"), 1, Xml::mustOReal("radius"), Xml::mustOReal("width"));    
  }
  
  dGeomSetData(ph.geom,(void*)&Wheel::type);
  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);
}


void Wheel::createNodesAndMeshXml() {
  Ogre::Entity *e;
  Ogre::SceneNode *n;

    n  = sceneMgr_->getRootSceneNode()->createChildSceneNode(cst.nodeName);
  cst.wheelNode = n;
  
  e = sceneMgr_->createEntity(cst.nodeName+"t", Xml::mustString("tire-mesh"));
  e->setMaterialName(Xml::mustString("tire-material"));
  e->setCastShadows(true);
  n->attachObject(e);

  e = sceneMgr_->createEntity(cst.nodeName+"h", Xml::mustString("hubcap-mesh"));
  e->setMaterialName(Xml::mustString("hubcap-material"));
  n->attachObject(e);

  n->scale(Conf::Wheel::SCALE[0], Conf::Wheel::SCALE[1], Conf::Wheel::SCALE[2]);
}


void Wheel::disposePhysicsXml() {
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, 
		       Ogre::Degree(Xml::mustOReal("rotation.y")).valueRadians());
    dBodySetRotation(ph.body, R);
    
    dGeomSetPosition(ph.geom, 
		     Conf::Car::POS[0] + Xml::mustOReal("position.x"), 
		     Conf::Car::POS[1] + Xml::mustOReal("position.y"), 
		     Conf::Car::POS[2] + Xml::mustOReal("position.z")
		     );
}


void Wheel::createXml(const char* xmlFile, dSpaceID space) {
  Xml::begin(xmlFile, "wheel");
  
  cst.xmlFile = xmlFile;
  cst.nodeName = Xml::mustString("name");
  
  createNodesAndMeshXml();
  createPhysicsXml(space);

  disposePhysicsXml();
  
  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);

  Xml::end();
}
