#include <OgreWindowEventUtilities.h>
#include "mytools.hpp"
#include "demo.hpp"
#include <fstream>
#include "world.hpp"
#include "ground.hpp"
#include "car.hpp"
#include "obstacle.hpp"

static dContact ballContact(){
  dContact contact;
  contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  contact.surface.mu = dInfinity;
  contact.surface.bounce = 0.5;
  contact.surface.bounce_vel = 0.1;
  contact.surface.soft_cfm = 0.01;  
  contact.surface.soft_erp = 0.3;  
  contact.surface.slip1 = 0.0;
  contact.surface.slip2 = 0.0;
  return contact;
}

void createBall(std::string name, dReal x, dReal y, dReal z){
  Ogre::Entity* e;
  e=_sceneMgr->createEntity(name.c_str(), "sphere.mesh");
  Ogre::SceneNode* n;
  n=_sceneMgr->getRootSceneNode()->createChildSceneNode(name.c_str());

  n->attachObject(e); 
  n->scale(0.01, 0.01, 0.01);
  e->setMaterialName("MyOwn/Sphere");

  dBodyID b;
  dGeomID g;
  dMass m;

  static dContact contact=ballContact();

  g = World::getSingletonPtr()->addSphere(1.0);
  dMassSetSphere (&m,1,1.0);

  b=World::getSingletonPtr()->add(g,&m);
  
  dGeomSetData(g,(void*)&contact);

  names.push_back(name);
  geoms.push_back(g);
  dBodySetPosition (b, x, y, z);
  MyTools::byOdeToOgre(b, n);
}

static dContact boxContact(){
  dContact contact;
  contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  contact.surface.mu = dInfinity;
  contact.surface.bounce = 0.01;
  contact.surface.bounce_vel = 0.7;
  contact.surface.soft_cfm = 0.01;  
  contact.surface.soft_erp = 0.3;  
  contact.surface.slip1 = 0.01;
  contact.surface.slip2 = 0.01;
  return contact;
}

void createBox(std::string name, dReal x, dReal y, dReal z){
  Ogre::Entity* e;
  e=_sceneMgr->createEntity(name.c_str(), "cube.mesh");

  Ogre::SceneNode* n;
  n=_sceneMgr->getRootSceneNode()->createChildSceneNode(name.c_str());

  n->attachObject(e); 
  n->scale(0.02, 0.02, 0.02);
  e->setMaterialName("Car/Subframe");

  dBodyID b;
  dGeomID g;
  dMass m;

  static dContact contact=boxContact();

  g = World::getSingletonPtr()->addBox(2.0, 2.0, 2.0);
  dMassSetBox (&m, 1 ,2.0, 2.0, 2.0);

  b=World::getSingletonPtr()->add(g,&m);
  
  dGeomSetData(g,(void*)&contact);

  names.push_back(name);//pour l'update
  geoms.push_back(g);
  dBodySetPosition (b, x, y, z);
  MyTools::byOdeToOgre(b, n);
}

demo::demo(){
  m_pCubeNode		= 0;
  m_pCubeEntity		= 0;
}

demo::~demo(){
  delete OgreFramework::getSingletonPtr();
}

void demo::startDemo(){
  new OgreFramework();
  if(!OgreFramework::getSingletonPtr()->initOgre("demo v1.0", this, 0))
    return;

  m_bShutdown = false;

  _glb.nbTurn=0;

  _log("Demo initialized");
	
  setupDemoScene();
  
  runDemo();
}

void demo::setupDemoScene(){
  _sceneMgr->createLight("Light")->setPosition(75,75,75);
  _sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE);
  //  _sceneMgr->setSkyBox(true, "cloud" ,200, false);
  _sceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8, 100);

  new FlatGround("Examples/Rockwall");
  
  Obstacle o("obstacle","obstacle.mesh",1.0,1.0,-20.0);
  o.setMaterial("Obstacle/Essai");

  Obstacle stair("starirway","stairway.mesh",-20.0, 3.5/2, 0.0);
  stair.setMaterial("Obstacle/Stairway");

  Obstacle stair1("starirway1","stairway.mesh",-20.0, 3.5 +3.5/2 +0.01, -20.0);
  stair1.setMaterial("Obstacle/Stairway");

  Obstacle stair2("starirway2","stairway.mesh",-20.0, 3.5*2 +3.5/2 +0.01, -20.0*2);
  stair2.setMaterial("Obstacle/Stairway");

  /*
  createBox(std::string("ball1"), 6, 2.01, 3);
  createBox(std::string("ball2"), 6, 2.01, 5.1);
  createBox(std::string("ball3"), 6, 2.01, 7.2);
  createBox(std::string("ball4"), 6, 2.01, 9.3);
  createBox(std::string("ball5"), 6, 2.01, 11.4);
  createBox(std::string("ball6"), 6, 2.01, 13.5);

  createBox(std::string("ball7"), 6, 4.01, 3);
  createBox(std::string("ball8"), 6, 4.01, 5.1);
  createBox(std::string("ball9"), 6, 4.01, 7.2);
  createBox(std::string("ball10"), 6, 4.01, 9.3);
  createBox(std::string("ball11"), 6, 4.01, 11.4);
  createBox(std::string("ball12"), 6, 4.01, 13.5);

  createBox(std::string("ball13"), 6, 6.01, 3);
  createBox(std::string("ball14"), 6, 6.01, 5.1);
  createBox(std::string("ball15"), 6, 6.01, 7.2);
  createBox(std::string("ball16"), 6, 6.01, 9.3);
  createBox(std::string("ball17"), 6, 6.01, 11.4);
  createBox(std::string("ball18"), 6, 6.01, 13.5);

  createBox(std::string("ball19"), 6, 8.01, 3);
  createBox(std::string("ball20"), 6, 8.01, 5.1);
  createBox(std::string("ball21"), 6, 8.01, 7.2);
  createBox(std::string("ball22"), 6, 8.01, 9.3);
  createBox(std::string("ball23"), 6, 8.01, 11.4);
  createBox(std::string("ball24"), 6, 8.01, 13.5);
  */

  /*  Ogre::SceneNode *node;
  Ogre::Entity *e;  

  e = _sceneMgr->createEntity("test","subframe.mesh");
  node = _sceneMgr->getRootSceneNode()->createChildSceneNode("test");
  node->attachObject(e);
  node->yaw(Ogre::Degree(90));*/

  extern Car car;
  car.init("car", _sceneMgr->getRootSceneNode());
}

void demo::runDemo(){
  _log("Start main loop");
	
  double timeSinceLastFrame =0;

  _log("reset stats");
  _renderWnd->resetStatistics();

  while(!m_bShutdown && !OgreFramework::getSingletonPtr()->isOgreToBeShutDown()){

    if(_renderWnd->isClosed())
      m_bShutdown = true;

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_LINUX
    Ogre::WindowEventUtilities::messagePump();
#endif	
      if(_renderWnd->isActive()){
	double startTime = _timer->getMillisecondsCPU();
	
	OgreFramework::getSingletonPtr()->m_pKeyboard->capture();
	OgreFramework::getSingletonPtr()->m_pMouse->capture();
	OgreFramework::getSingletonPtr()->updateOgre(timeSinceLastFrame);
	OgreFramework::getSingletonPtr()->m_pRoot->renderOneFrame();

	const int timeForEachFrame = 17;
	forFrameDo(timeForEachFrame);
		
	timeSinceLastFrame = _timer->getMillisecondsCPU() - startTime;
	int lazyTime=timeForEachFrame-timeSinceLastFrame;
	if(lazyTime>0){
	  usleep(lazyTime*1000);//usleep meaning micro
	}
      }
      else{
	usleep(1000);
      }
  }

  _log("Main loop quit");
  _log("Shutdown OGRE");
}

void demo::forFrameDo(unsigned int time){
  _glb.nbTurn++;
  
  //simulation loop
  World::getSingletonPtr()->update();
   
  {//mise a jour des object visuels par les object physique
    for(int i=0; i<geoms.size(); i++){
      MyTools::byOdeToOgre(geoms[i] ,_sceneMgr->getSceneNode(names[i].c_str()));
    }

    extern Car car;
    car.update();
  }
    
}

bool demo::keyPressed(const OIS::KeyEvent &keyEventRef){
  OgreFramework::getSingletonPtr()->keyPressed(keyEventRef);
  
  extern Car car;
	
  
  switch(keyEventRef.key){
    
  case OIS::KC_UP :
    car.accelerate();
    break;

  case OIS::KC_DOWN :
    car.slowDown();
    break;

  case OIS::KC_RIGHT :
    car.turnRight();
    break;

  case OIS::KC_LEFT :
    car.turnLeft();
    break;

  case OIS::KC_END :
    car.setSteer(0.0);
    car.setSpeed(0.0);
    break;

  case OIS::KC_SPACE :
    car.setBrake(true);
    break;


  case OIS::KC_B :
    static std::string ba("");
    ba+="a";
    createBox(ba,-15.0, 10.0, 0.0);
    break;

  case OIS::KC_N :
    static std::string bo("");
    bo+="o";
    createBall(bo,-25.0, 10.0, 0.0);
    break;

  }

  return true;
}


bool demo::keyReleased(const OIS::KeyEvent &keyEventRef){
  OgreFramework::getSingletonPtr()->keyReleased(keyEventRef);

  extern Car car;

  switch(keyEventRef.key){

  case OIS::KC_UP :
    car.setSpeed(.0);
    break;
    
  case OIS::KC_DOWN :
    car.setSpeed(.0);
    break;
    
  case OIS::KC_RIGHT :
    car.setSteer(0.0);
    break;
    
  case OIS::KC_LEFT :
    car.setSteer(0.0);
    break;

  case OIS::KC_SPACE :
    car.setBrake(false);
    break;

  }
	
  return true;
}
