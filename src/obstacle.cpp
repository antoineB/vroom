#include "obstacle.hpp"
#include "mytools.hpp"
#include "world.hpp"

dContact Obstacle::contact;

void Obstacle::initGeom(void* ptr){
  char * cPtr=(char*)ptr;
  Ogre::SceneNode* n=_sceneMgr->getRootSceneNode()->createChildSceneNode(name);
  Ogre::Entity* e=_sceneMgr->createEntity(name, cPtr);
  e->setCastShadows(true);
  n->attachObject(e);

  //creating trimesh
  dTriMeshDataID data = MyTools::dTriMeshDataFromMesh(e);
    
  //  geom = World::getSingletonPtr()->addTriMesh(space,data,0,0,0);
  geom = World::getSingletonPtr()->addTriMesh(data);
  contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  contact.surface.mu = dInfinity;
  contact.surface.bounce = 0.01;
  contact.surface.bounce_vel = 0.7;
  contact.surface.soft_cfm = 0.01;  
  contact.surface.soft_erp = 0.3;  
  contact.surface.slip1 = 0.01;
  contact.surface.slip2 = 0.01;
  dGeomSetData(geom,(void*)&contact);
}

Obstacle::Obstacle(const char *n, const char *meshName, dReal x, dReal y, dReal z)
 : name(n)
{
  Obstacle::initGeom((void*)meshName);
  dGeomSetPosition (geom, x, y, z);
  MyTools::byOdeToOgre(geom,_sceneMgr->getSceneNode(name));
}

void Obstacle::update(){}

void Obstacle::setMaterial(const char* na) const{
  Ogre::Entity* e=_sceneMgr->getEntity(name);
  e->setMaterialName(na);
}
