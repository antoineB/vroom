#include "world.hpp"
#include "ground.hpp"

#include <tinyxml.h>
#include <assert.h>

using namespace std;

#define CONSTANT_SIZE sizeof(dReal)*6

#define SIMULATION_PACE 0.5
#define GRAVITY_X 0.0
#define GRAVITY_Y -0.2
#define GRAVITY_Z 0.0
#define CFM 1e-5

template<> World* Ogre::Singleton<World>::ms_Singleton = 0;

World::World(string xmlFileName){
  dInitODE();

  world = dWorldCreate();

  contactGroup = dJointGroupCreate(0);
  space = dHashSpaceCreate(0);
  
  _glb.worldUp = true;

  parseXml(xmlFileName);

  dWorldSetGravity (world, cst.gravity[0], cst.gravity[1], cst.gravity[2]);
  dWorldSetCFM (world, cst.cfm);
  //  dWorldSetERP()
}

void World::parseXml(string fileName) throw(std::string){
  Utils::Xml::begin(fileName, "world");
  
  TiXmlElement* constant = Utils::Xml::mustNode("constant");
  
  cst.cfm = Utils::Xml::mustFloat("cfm", 0, constant);
  cst.simulationPace = Utils::Xml::mustFloat("simulation-pace", 0, constant);
  cst.gravity[0] = Utils::Xml::mustFloat("gravity.x", 0, constant);
  cst.gravity[1] = Utils::Xml::mustFloat("gravity.y", 0, constant);
  cst.gravity[2] = Utils::Xml::mustFloat("gravity.z", 0, constant);

  Utils::Xml::end();
}

dReal World::atodr(const char *str){
  return (dReal)atof(str);
}

dBodyID World::add(dGeomID g, dMass *m){
    dBodyID b = dBodyCreate( world );

    //needed if geom is trimesh
    //      dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);

    dBodySetMass (b, m);
    dGeomSetBody ( g , b );
    return b;
}

dJointID World::addHinge2(dBodyID b1, dBodyID b2, dJointGroupID jg){
  dJointID j = dJointCreateHinge2(world, jg);
  dJointAttach(j, b1, b2);
  return j;
}

dJointID World::addHinge(dBodyID b1, dBodyID b2, dJointGroupID jg){
  dJointID j = dJointCreateHinge(world, jg);
  dJointAttach(j, b1, b2);
  return j;
}


dWorldID World::getWorld() const {
  return world;
}

dJointGroupID World::getContactGroup() const {
  return contactGroup;
}

World::~World(){
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  _glb.worldUp=false;
}

inline void World::beforeCollidingSpaces() {
  //adding the contact_points_node if unexistant
  if (!sceneMgr_->hasSceneNode("contact_points_node")) {
    sceneMgr_->getRootSceneNode()->createChildSceneNode("contact_points_node");
  }
  //removing the dCollision points
  else {
    for (int i = 0; i < _glb.collidingPoints.size(); i++) {
      while(_glb.collidingPoints[i]->numAttachedObjects())
	sceneMgr_->destroyEntity((Ogre::Entity*)_glb.collidingPoints[i]->detachObject((short unsigned int)0));
      sceneMgr_->destroySceneNode(_glb.collidingPoints[i]);
    }
    _glb.collidingPoints.clear();
  }
}

inline void World::afterCollidingSpaces() {
  //adding the swayBars forces
  extern Car car;  
  //car.swayBars();
}

inline void World::beforePhysicalStep() {}

inline void World::afterPhysicalStep() {}

void World::update(){

  beforeCollidingSpaces();

  dSpaceCollide(space, NULL/*what is this*/, &nearCallback);

  afterCollidingSpaces();
  beforePhysicalStep();

  dWorldQuickStep(world,cst.simulationPace);
  dJointGroupEmpty(contactGroup);

  afterPhysicalStep();
}

#include "type.hpp"

void nearCallback (void *data, dGeomID o1, dGeomID o2){
  assert(dGeomGetData(o1) != NULL);
  assert(dGeomGetData(o2) != NULL);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))  
    dSpaceCollide2( o1, o2, NULL, &nearCallback);

  DContactType *c1 = (DContactType*) dGeomGetData(o1);
  DContactType *c2 = (DContactType*) dGeomGetData(o2);
  
  c1->dealWith(o1, o2);
}


#undef CONSTANT_SIZE 
#undef SIMULATION_PACE
#undef GRAVITY_X
#undef GRAVITY_Y
#undef GRAVITY_Z
#undef CFM
