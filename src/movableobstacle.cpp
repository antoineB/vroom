#include "movableobstacle.hpp"
#include "mytools.hpp"
#include "world.hpp"

DContactType MovableObstacle::type(Type::MOVABLE_OBSTACLE);

MovableObstacle::MovableObstacle(const char *n, const char* meshName, dReal x, dReal y, dReal z) {
  

  Ogre::SceneNode* node = sceneMgr_->getRootSceneNode()->createChildSceneNode(name);
  Ogre::Entity* ent = sceneMgr_->createEntity(name, meshName);
  ent->setCastShadows(false);
  node->attachObject(ent);

  //creating trimesh
  dTriMeshDataID data = MyTools::dTriMeshDataFromMesh(ent);
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

    dMassSetTrimesh(&mass, 1.0, geom);
  //  std::cout<<"mass "<<mass.mass<<std::endl;
  dMassAdjust(&mass, 1000.0);
  //  std::cout<<"mass "<<mass.mass<<std::endl;
  dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
  //  dMassAdjust(&mass, 10.0);  
  body = World::getSingletonPtr()->add(geom ,&mass);

  
  dGeomSetOffsetPosition(geom, 0.0, +6.0, 0.0);
  
  dBodySetPosition(body, x, y, z);

  dBodyDisable(body); 
}

void MovableObstacle::update() {
  Ogre::SceneNode *node = sceneMgr_->getSceneNode(name);
  MyTools::byOdeToOgre(geom, node);
}

void MovableObstacle::setMaterial(const char* nameMat) const{
  Ogre::Entity* e = sceneMgr_->getEntity(name);
  e->setMaterialName(nameMat);
}
