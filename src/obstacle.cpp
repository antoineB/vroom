#include "obstacle.hpp"
#include "mytools.hpp"
#include "world.hpp"

DContactType Obstacle::type(Type::OBSTACLE);

void Obstacle::initGeom(void* ptr){
  char * cPtr=(char*)ptr;
  Ogre::SceneNode* n=sceneMgr_->getRootSceneNode()->createChildSceneNode(name);
  Ogre::Entity* e=sceneMgr_->createEntity(name, cPtr);
  e->setCastShadows(true);
  n->attachObject(e);

  //creating trimesh
  dTriMeshDataID data = MyTools::dTriMeshDataFromMesh(e);
    
  //  geom = World::getSingletonPtr()->addTriMesh(space,data,0,0,0);
  geom = World::getSingletonPtr()->addTriMesh(data);

  type.contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  type.contact.surface.mu = dInfinity;
  type.contact.surface.bounce = 0.01;
  type.contact.surface.bounce_vel = 0.7;
  type.contact.surface.soft_cfm = 0.01;  
  type.contact.surface.soft_erp = 0.3;  
  type.contact.surface.slip1 = 0.01;
  type.contact.surface.slip2 = 0.01;

  dGeomSetData(geom,(void*)&type);
}

Obstacle::Obstacle(const char *n, const char *meshName, dReal x, dReal y, dReal z)
 : name(n)
{
  Obstacle::initGeom((void*)meshName);
  dGeomSetPosition (geom, x, y, z);
  MyTools::byOdeToOgre(geom,sceneMgr_->getSceneNode(name));
}

void Obstacle::update(){}

void Obstacle::setMaterial(const char* na) const{
  Ogre::Entity* e=sceneMgr_->getEntity(name);
  e->setMaterialName(na);
}
