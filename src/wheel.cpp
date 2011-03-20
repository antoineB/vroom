#include "wheel.hpp"
#include "world.hpp"


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


void Wheel::createPhysics(dSpaceID space, unsigned int nbWheel, bool sphereWheel) {
  fillContact();

  if (sphereWheel) {
    ph.geom = dCreateSphere(space, Conf::Wheel::RADIUS);
    dMassSetSphere (&ph.mass, Conf::Wheel::DENSITY, Conf::Wheel::RADIUS);
  }
  else {
    ph.geom = dCreateCylinder(space, Conf::Wheel::RADIUS, Conf::Wheel::WIDTH);
    dMassSetCylinder(&ph.mass, 1, Conf::Wheel::DENSITY, Conf::Wheel::RADIUS, Conf::Wheel::WIDTH);
  }
  
  dGeomSetData(ph.geom,(void*)&Wheel::type);

  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);

}


void Wheel::createPhysics(dSpaceID space, unsigned int nbWheel, bool sphereWheel, Conf::Wheel::Param &mod) {
  memcpy(&ph.mass, &mod.mass, sizeof(dMass));

  if (sphereWheel)
    ph.geom = dCreateSphere(space, Conf::Wheel::RADIUS);
  else
    ph.geom = dCreateCylinder(space, Conf::Wheel::RADIUS, Conf::Wheel::WIDTH);
  
  dGeomSetData(ph.geom,(void*)&Wheel::type);

  //MUST CHANGE
  dMassSetCylinder(&ph.mass, Conf::Wheel::DENSITY, 1, Conf::Wheel::RADIUS,  Conf::Wheel::WIDTH);


  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);
}


void Wheel::update() {
  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);
}


Wheel::~Wheel() {}


std::string Wheel::generateName(unsigned int number) {
  std::string name = "wheel_";
  std::ostringstream out;
  out << number;
  name += out.str();
  return name;
}


void Wheel::createNodesAndMesh(Ogre::SceneNode *carNode) {
  Ogre::Entity *e;
  Ogre::SceneNode *n;

  n  = carNode->createChildSceneNode(cst.nodeName);
  cst.wheelNode = n;
  
  e = sceneMgr_->createEntity(cst.nodeName+"_tire", "wheel_tire.mesh");
  e->setMaterialName("Wheels/Tire");
  e->setCastShadows(true);
  n->attachObject(e);

  e = sceneMgr_->createEntity(cst.nodeName+"_hubcap", "wheel_hubcap.mesh");
  e->setMaterialName("Wheels/Hubcap");
  n->attachObject(e);

  n->scale(Conf::Wheel::SCALE[0], Conf::Wheel::SCALE[1], Conf::Wheel::SCALE[2]);
}

void Wheel::disposePhysics(unsigned int nbWheel) {
  {//changing orientation of body depend on wheel side
    dMatrix3 R;
    if (nbWheel % 2)
      dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, -Conf::Math::PI/2);//beware of the sgn of PI/2
    else
      dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, Conf::Math::PI/2);
    
    dBodySetRotation(ph.body, R);
  }
  
  dGeomSetPosition(ph.geom, 
		   Conf::Car::POS[0] + Conf::Wheel::POS[nbWheel][0], 
		   Conf::Car::POS[1] + Conf::Wheel::POS[nbWheel][1], 
		   Conf::Car::POS[2] + Conf::Wheel::POS[nbWheel][2]
		   );
}


void Wheel::create(dSpaceID space, Ogre::SceneNode *carNode)
{
  static int nb = 0;
  if (nb > 3) nb = 0; //the case when we call reset

  cst.nodeName = generateName(nb);
  createNodesAndMesh(carNode);

  createPhysics(space, nb);
  disposePhysics(nb);

  MyTools::byOdeToOgre(ph.geom, cst.wheelNode);

  nb++;
}


void Wheel::reset(Conf::Wheel::Param &mod) {
  static int nb = 0;
  if (nb > 3) nb = 0;

  dSpaceID space = dGeomGetSpace(ph.geom);
  dGeomDestroy(ph.geom);
  dBodyDestroy(ph.body);
  
  createPhysics(space, nb, false, mod);
  disposePhysics(nb);

  nb++;
}
