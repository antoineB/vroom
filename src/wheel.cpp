#include "wheel.hpp"
#include "world.hpp"
#include "utils.hpp"

#include <OGRE/OgreMath.h>

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
  
  dGeomSetData(ph.geom,(void*)&Wheel::type);
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

  n->scale(Conf::Wheel::SCALE[0], Conf::Wheel::SCALE[1], Conf::Wheel::SCALE[2]);
}


void Wheel::disposePhysics(Utils::Xml &x) {
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, 
		       Ogre::Degree(x.mustOReal("rotation.y")).valueRadians());
    dBodySetRotation(ph.body, R);
    
    dGeomSetPosition(ph.geom, 
		     Conf::Car::POS[0] + x.mustOReal("position.x"), 
		     Conf::Car::POS[1] + x.mustOReal("position.y"), 
		     Conf::Car::POS[2] + x.mustOReal("position.z")
		     );
}


void Wheel::createXml(const char* xmlFile, dSpaceID space) {
  Utils::Xml x(xmlFile, "wheel");
  
  cst.xmlFile = xmlFile;
  cst.nodeName = x.mustString("name");
  
  createNodesAndMesh(x);
  createPhysics(x, space);

  disposePhysics(x);
  
  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);

}
