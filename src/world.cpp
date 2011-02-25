#include "world.hpp"
#include "ground.hpp"

#include <tinyxml.h>
#include <assert.h>

using namespace std;

#define CONSTANT_SIZE sizeof(dReal)*6

#define PI 3.14159265
#define SIMULATION_PACE 0.5
#define GRAVITY_X 0.0
#define GRAVITY_Y -0.2
#define GRAVITY_Z 0.0
#define CFM 1e-5

template<> World* Ogre::Singleton<World>::ms_Singleton = 0;

void World::preInit(){
  cst.pi=PI;
  cst.simulationPace=SIMULATION_PACE;
  cst.gravity[0]=GRAVITY_X;
  cst.gravity[1]=GRAVITY_Y;
  cst.gravity[2]=GRAVITY_Z;
  cst.cfm=CFM;
  
  dInitODE();

  world=dWorldCreate();

  contactGroup=dJointGroupCreate(0);
  space=dHashSpaceCreate(0);
  
  _glb.worldUp=true;
}

void World::postInit(){
  dWorldSetGravity (world, cst.gravity[0], cst.gravity[1], cst.gravity[2]);
  dWorldSetCFM (world, cst.cfm);
}

World::World(string xmlFileName){
  preInit();
  parseXml(xmlFileName);
  postInit();
}

World::World(dReal *array){
  preInit();
  memcpy(&cst, array, CONSTANT_SIZE);
  postInit();
}

void World::parseXml(string fileName) throw(std::string){
  std::cout<<"verification"<<std::endl;
  TiXmlDocument doc(fileName);
  if(!doc.LoadFile()){
    throw std::string("IO error in xml file");
    return ;
  }
  TiXmlHandle docH( &doc );
  TiXmlHandle cstH = docH.FirstChild( "world" ).FirstChild( "constant" );
  {
    TiXmlElement *elem;
    if((elem=cstH.FirstChild( "cfm" ).ToElement()))
      cst.cfm=World::atodr(elem->GetText());
    if((elem=cstH.FirstChild( "pi" ).ToElement()))
      cst.pi=World::atodr(elem->GetText());
    if((elem=cstH.FirstChild( "simulation_pace" ).ToElement()))
      cst.simulationPace=World::atodr(elem->GetText());
    
    TiXmlHandle gravH=cstH.FirstChild( "gravity" );
    {
      if((elem=gravH.FirstChild( "x" ).ToElement()))
	cst.gravity[0]=World::atodr(elem->GetText());
      else
	throw std::string("missing parameter for <gravity>");
      if((elem=gravH.FirstChild( "y" ).ToElement()))
	cst.gravity[1]=World::atodr(elem->GetText());
      else
	throw std::string("missing parameter for <gravity>");
      if((elem=gravH.FirstChild( "z" ).ToElement())) 
	cst.gravity[2]=World::atodr(elem->GetText());
      else
	throw std::string("missing parameter for <gravity>");
    }
  }
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

inline void World::beforeCollidingSpaces() {}

inline void World::afterCollidingSpaces() {
  extern Car car;  
  car.swayBars();
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

static void mixdContact(dContact* res, dContact* c1, dContact* c2){
  //  memset(res,0, sizeof(dContact));

  int m1 = c1->surface.mode, m2 = c2->surface.mode ;

  memset(&(res->surface.mode),0,sizeof(dSurfaceParameters));

  //set the mix
  if(m1 & dContactBounce){
    res->surface.mode |= dContactBounce;
      res->surface.bounce += c1->surface.bounce;  
  }
  
  if(m2 & dContactBounce){
    res->surface.mode |= dContactBounce;
    res->surface.bounce += c2->surface.bounce;
  }

  if(m1 & dContactSoftCFM){
    res->surface.mode |= dContactSoftCFM;
    res->surface.soft_cfm += c1->surface.soft_cfm;
  }

  if(m2 & dContactSoftCFM){
    res->surface.mode |= dContactSoftCFM;
    res->surface.soft_cfm += c2->surface.soft_cfm;
  }

  if(m1 & dContactSoftERP){
    res->surface.mode |= dContactSoftERP;
    res->surface.soft_erp += c1->surface.soft_erp;
  }
  
  if(m2 & dContactSoftERP){
    res->surface.mode |= dContactSoftERP;
    res->surface.soft_erp += c2->surface.soft_erp;
  }

  if(m1 & dContactSlip1){
    res->surface.mode |= dContactSlip1;
    res->surface.slip1 += c1->surface.slip1;
    
  }
  
  if(m2 & dContactSlip1){
    res->surface.mode |= dContactSlip1;
    res->surface.slip1 += c2->surface.slip1;
  }

  if(m1 & dContactSlip2){
    res->surface.mode |= dContactSlip2;
    res->surface.slip2 += c1->surface.slip2;
  }

  if(m2 & dContactSlip2){
    res->surface.mode |= dContactSlip2;
    res->surface.slip2 += c2->surface.slip2;
  }


  if(m1 & dContactMu2){
    res->surface.mode |= dContactMu2;
    if(!res->surface.mu2 == dInfinity)//crappy
      res->surface.mu2 += c1->surface.mu2;
  }

  if(m2 & dContactMu2){
    res->surface.mode |= dContactMu2;
    if(!res->surface.mu2 == dInfinity)
      res->surface.mu2 += c2->surface.mu2;
  }

  if(c2->surface.mu == dInfinity)
    res->surface.mu = dInfinity;
  else if(c1->surface.mu == dInfinity)
    res->surface.mu = dInfinity;
  else
    res->surface.mu = c2->surface.mu + c1->surface.mu ;

  res->surface.bounce_vel = (c1->surface.bounce_vel < c2->surface.bounce_vel)? c1->surface.bounce_vel : c2->surface.bounce_vel ;

  //take care of possibles results
  if(res->surface.mu <0) res->surface.mu =0;
  if(res->surface.bounce_vel < 0) res->surface.bounce_vel =0;

  if(res->surface.slip1<0) res->surface.slip1=0;

  if(res->surface.slip2<0) res->surface.slip2=0;
  
}

#define drawContactPoints_(BOOL)					\
  if (BOOL) {								\
  static int n = _glb.nbTurn;						\
  static std::vector<Ogre::Entity*> ents;				\
  if (n < _glb.nbTurn) {						\
  n = _glb.nbTurn;							\
  for (int i = ents.size()-1; i >= 0; i--) {				\
      sceneMgr_->destroySceneNode(ents[i]->getParentSceneNode());	\
      sceneMgr_->destroyEntity(ents[i]);				\
      ents.pop_back();							\
  }									\
  }									\
  Ogre::SceneNode* node;						\
  try{ node = sceneMgr_->getSceneNode("contact_node"); }		\
  catch (Ogre::Exception e) {						\
    node=sceneMgr_->getRootSceneNode()					\
      ->createChildSceneNode("contact_node");				\
  }									\
  for(int i=0; i < numc; i++) {						\
  Ogre::Entity* e = sceneMgr_->createEntity("collision_point.mesh");	\
  Ogre::SceneNode* n=node->createChildSceneNode(			\
    Ogre::Vector3( (Ogre::Real)contact[i].geom.pos[0],			\
		   (Ogre::Real)contact[i].geom.pos[1],			\
		   (Ogre::Real)contact[i].geom.pos[2]) );		\
  ents.push_back(e);							\
  n->attachObject(e);							\
  n->scale(0.35, 0.35, 0.35);						\
  }    									\
  }									\


static dReal avgDepth(dContact *contacts, int nbContacts) {
  dReal sum = 0;
  for (int i = 0; i < nbContacts; i++)
    sum += contacts[i].geom.depth;
  return sum/nbContacts;
}

static dReal maxDepth(dContact *contacts, int nbContacts) {
  dReal max = 0;
  for (int i = 0; i < nbContacts; i++)
    if (contacts[i].geom.depth > max)
      max = contacts[i].geom.depth;
  return max;
}

static void avgNormal(dVector3 avgNorm, dContact *contacts, int nbContacts) {
  avgNorm[0] = .0;
  avgNorm[1] = .0;
  avgNorm[2] = .0;

  for (int i = 0; i < nbContacts; i++) {
    avgNorm[0] += contacts[i].geom.normal[0];
    avgNorm[1] += contacts[i].geom.normal[1];
    avgNorm[2] += contacts[i].geom.normal[2];
  } 

  avgNorm[0] /= nbContacts;
  avgNorm[1] /= nbContacts;
  avgNorm[2] /= nbContacts;
}

#include "type.hpp"

void nearCallback (void *data, dGeomID o1, dGeomID o2){
  assert(dGeomGetData(o1) != NULL);
  assert(dGeomGetData(o2) != NULL);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))  
    dSpaceCollide2( o1, o2, NULL, &nearCallback);

  DContactType *c1 = (DContactType*) dGeomGetData(o1);
  DContactType *c2 = (DContactType*) dGeomGetData(o2);
  
  cout<<"avant"<<endl;
  c1->dealWith(o1, o2);
  cout<<"apres"<<endl;
}


#undef CONSTANT_SIZE 
#undef PI
#undef SIMULATION_PACE
#undef GRAVITY_X
#undef GRAVITY_Y
#undef GRAVITY_Z
#undef CFM
