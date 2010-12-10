#include "ground.hpp"

//what is this?
template<> Ground* Ogre::Singleton<Ground>::ms_Singleton = 0;

const std::string Ground::name="ground";

dContact FlatGround::contact;

dGeomID Ground::getPlane(){ return plane; }

FlatGround::FlatGround(const char* material){
  FlatGround::init(material);
}

void FlatGround::init(const char* material){
  Ogre::Plane oPlane(Ogre::Vector3::UNIT_Y, 0);
  Ogre::MeshManager::getSingleton()
    .createPlane("land", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		 oPlane, 4*150, 4*150, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
  
  Ogre::Entity* entGround = sceneMgr_->createEntity(name, "land");
  sceneMgr_->getRootSceneNode()->createChildSceneNode(name)
    ->attachObject(entGround);
  
  entGround->setMaterialName(material);
  entGround->setCastShadows(false);
  plane=World::getSingletonPtr()->addPlane(0.0,1.0,0.0,0.0);
  contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  contact.surface.mu = dInfinity;
  contact.surface.bounce = 0.01;
  contact.surface.bounce_vel = 0.5;
  contact.surface.soft_cfm = 0.01;  
  contact.surface.soft_erp = 0.3;  
  contact.surface.slip1 = 0.01;
  contact.surface.slip2 = 0.01;
  dGeomSetData(plane,(void*)&contact);

  dContact * c2 = (dContact*) dGeomGetData(plane);
  if(c2==NULL)
    std::cout<<"plane is null"<<std::endl;

  
}
