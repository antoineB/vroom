#include <OGRE/OgreWindowEventUtilities.h>
#include "mytools.hpp"
#include "demo.hpp"
#include <fstream>
#include "world.hpp"
#include "ground.hpp"
#include "car.hpp"
#include "obstacle.hpp"
#include "movableobstacle.hpp"

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
  e=sceneMgr_->createEntity(name.c_str(), "sphere.mesh");
  Ogre::SceneNode* n;
  n=sceneMgr_->getRootSceneNode()->createChildSceneNode(name.c_str());

  n->attachObject(e); 
  n->scale(0.03, 0.03, 0.03);
  e->setMaterialName("MyOwn/Sphere");

  dBodyID b;
  dGeomID g;
  dMass m;

  static dContact contact=ballContact();

  g = World::getSingletonPtr()->addSphere(3.0);
  dMassSetSphere (&m,1,3.0);

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

void createBox(std::string name, dReal x, dReal y, dReal z) 
{
  Ogre::Entity* e;
  e=sceneMgr_->createEntity(name.c_str(), "cube.mesh");
  Ogre::SceneNode* n;
  n=sceneMgr_->getRootSceneNode()->createChildSceneNode(name.c_str());

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

  names.push_back(name);//for updating
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

static void setupCegui() {
  using namespace CEGUI;
  {
    Window *bErp = windowMgr_->getWindow("root/wheels/back/erp");
    Window *bCfm = windowMgr_->getWindow("root/wheels/back/cfm");
    
    Window *fErp = windowMgr_->getWindow("root/wheels/front/erp");
    Window *fCfm = windowMgr_->getWindow("root/wheels/front/cfm");
    
    extern Car car;
    ((Scrollbar*)bErp)->setScrollPosition((float)(car.getBackWheelsErp()));
    ((Scrollbar*)bCfm)->setScrollPosition((float)(car.getBackWheelsCfm()));
    ((Scrollbar*)fErp)->setScrollPosition((float)(car.getFrontWheelsErp()));
    ((Scrollbar*)fCfm)->setScrollPosition((float)(car.getFrontWheelsCfm()));
    
    bErp->subscribeEvent(Scrollbar::EventScrollPositionChanged, 
			 Event::Subscriber(&OgreFramework::setBackWheelsErp,
					   OgreFramework::getSingletonPtr()
					   ));
    
    bCfm->subscribeEvent(Scrollbar::EventScrollPositionChanged, 
			 Event::Subscriber(&OgreFramework::setBackWheelsCfm, 
					   OgreFramework::getSingletonPtr()
					   ));
    
    fErp->subscribeEvent(Scrollbar::EventScrollPositionChanged, 
			 Event::Subscriber(&OgreFramework::setFrontWheelsErp, 
					   OgreFramework::getSingletonPtr()));
    
    fCfm->subscribeEvent(Scrollbar::EventScrollPositionChanged, 
			 Event::Subscriber(&OgreFramework::setFrontWheelsCfm, 
					   OgreFramework::getSingletonPtr()));
  }

  {
    extern Car car;

    Window *mass = windowMgr_->getWindow("root/car_param/mass/total/value");
    Window *x = windowMgr_->getWindow("root/car_param/mass/pos/x");
    Window *y = windowMgr_->getWindow("root/car_param/mass/pos/y");
    Window *z = windowMgr_->getWindow("root/car_param/mass/pos/z");
    
    std::string s;
    convDS_(car.ph.mass.mass, s);
    mass->setText(s);
    
    convDS_(Conf::Car::BOX[0], s);
    x->setText(s);
    
    convDS_(Conf::Car::BOX[1], s);
    y->setText(s);
    
    convDS_(Conf::Car::BOX[2], s);
    z->setText(s);
  }

  {
    Window *x = windowMgr_->getWindow("root/car_param/geom/box/x");
    Window *y = windowMgr_->getWindow("root/car_param/geom/box/y");
    Window *z = windowMgr_->getWindow("root/car_param/geom/box/z");

    std::string s;
    convDS_(Conf::Car::BOX[0], s);
    x->setText(s);
    
    convDS_(Conf::Car::BOX[1], s);
    y->setText(s);
    
    convDS_(Conf::Car::BOX[2], s);
    z->setText(s);
  }

  {
    Window *x = windowMgr_->getWindow("root/car_param/geom/offset/x");
    Window *y = windowMgr_->getWindow("root/car_param/geom/offset/y");
    Window *z = windowMgr_->getWindow("root/car_param/geom/offset/z");

    std::string s;
    convDS_(Conf::Car::POSOFFSET[0], s);
    x->setText(s);
    
    convDS_(Conf::Car::POSOFFSET[1], s);
    y->setText(s);
    
    convDS_(Conf::Car::POSOFFSET[2], s);
    z->setText(s);
  }

  {
    std::string s;

    Window *x = windowMgr_->getWindow("root/car_param/joint/front/left/axis1/x");
    Window *y = windowMgr_->getWindow("root/car_param/joint/front/left/axis1/y");
    Window *z = windowMgr_->getWindow("root/car_param/joint/front/left/axis1/z");
    convDS_(Conf::Car::AXIS1[0][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS1[0][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS1[0][2], s);
    z->setText(s);

    x = windowMgr_->getWindow("root/car_param/joint/front/left/axis2/x");
    y = windowMgr_->getWindow("root/car_param/joint/front/left/axis2/y");
    z = windowMgr_->getWindow("root/car_param/joint/front/left/axis2/z");
    convDS_(Conf::Car::AXIS2[0][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS2[0][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS2[0][2], s);
    z->setText(s);

    x = windowMgr_->getWindow("root/car_param/joint/front/right/axis1/x");
    y = windowMgr_->getWindow("root/car_param/joint/front/right/axis1/y");
    z = windowMgr_->getWindow("root/car_param/joint/front/right/axis1/z");
    convDS_(Conf::Car::AXIS1[1][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS1[1][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS1[1][2], s);
    z->setText(s);

    x = windowMgr_->getWindow("root/car_param/joint/front/right/axis2/x");
    y = windowMgr_->getWindow("root/car_param/joint/front/right/axis2/y");
    z = windowMgr_->getWindow("root/car_param/joint/front/right/axis2/z");
    convDS_(Conf::Car::AXIS2[1][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS2[1][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS2[1][2], s);
    z->setText(s);


    x = windowMgr_->getWindow("root/car_param/joint/back/left/axis1/x");
    y = windowMgr_->getWindow("root/car_param/joint/back/left/axis1/y");
    z = windowMgr_->getWindow("root/car_param/joint/back/left/axis1/z");
    convDS_(Conf::Car::AXIS1[2][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS1[2][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS1[2][2], s);
    z->setText(s);

    x = windowMgr_->getWindow("root/car_param/joint/back/left/axis2/x");
    y = windowMgr_->getWindow("root/car_param/joint/back/left/axis2/y");
    z = windowMgr_->getWindow("root/car_param/joint/back/left/axis2/z");
    convDS_(Conf::Car::AXIS2[2][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS2[2][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS2[2][2], s);
    z->setText(s);

    x = windowMgr_->getWindow("root/car_param/joint/back/right/axis1/x");
    y = windowMgr_->getWindow("root/car_param/joint/back/right/axis1/y");
    z = windowMgr_->getWindow("root/car_param/joint/back/right/axis1/z");
    convDS_(Conf::Car::AXIS1[3][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS1[3][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS1[3][2], s);
    z->setText(s);

    x = windowMgr_->getWindow("root/car_param/joint/back/right/axis2/x");
    y = windowMgr_->getWindow("root/car_param/joint/back/right/axis2/y");
    z = windowMgr_->getWindow("root/car_param/joint/back/right/axis2/z");
    convDS_(Conf::Car::AXIS2[3][0], s);
    x->setText(s);
    convDS_(Conf::Car::AXIS2[3][1], s);
    y->setText(s);
    convDS_(Conf::Car::AXIS2[3][2], s);
    z->setText(s);
  }

  {
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/car/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/car/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/car/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/car/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/car/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/car/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/car/slip2/value");
    
    std::string s;
    convDS_(Car::type.contact.surface.mu, s);
    mu->setText(s);
    convDS_(Car::type.contact.surface.bounce, s);
    bounce->setText(s);
    convDS_(Car::type.contact.surface.bounce_vel, s);
    bounceVel->setText(s);
    convDS_(Car::type.contact.surface.soft_erp, s);
    erp->setText(s);
    convDS_(Car::type.contact.surface.soft_cfm, s);
    cfm->setText(s);
    convDS_(Car::type.contact.surface.slip1, s);
    slip1->setText(s);
    convDS_(Car::type.contact.surface.slip2, s);
    slip2->setText(s);
  }

  {
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/ground/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/ground/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/ground/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/ground/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/ground/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/ground/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/ground/slip2/value");

    std::string s;
    convDS_(FlatGround::type.contact.surface.mu, s);
    mu->setText(s);
    convDS_(FlatGround::type.contact.surface.bounce, s);
    bounce->setText(s);
    convDS_(FlatGround::type.contact.surface.bounce_vel, s);
    bounceVel->setText(s);
    convDS_(FlatGround::type.contact.surface.soft_erp, s);
    erp->setText(s);
    convDS_(FlatGround::type.contact.surface.soft_cfm, s);
    cfm->setText(s);
    convDS_(FlatGround::type.contact.surface.slip1, s);
    slip1->setText(s);
    convDS_(FlatGround::type.contact.surface.slip2, s);
    slip2->setText(s);
  }

  {
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/wheel/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/wheel/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/wheel/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/wheel/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/wheel/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/wheel/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/wheel/slip2/value");

    std::string s; 
    convDS_(Wheel::type.contact.surface.mu, s);
    mu->setText(s);
    convDS_(Wheel::type.contact.surface.bounce, s);
    bounce->setText(s);
    convDS_(Wheel::type.contact.surface.bounce_vel, s);
    bounceVel->setText(s);
    convDS_(Wheel::type.contact.surface.soft_erp, s);
    erp->setText(s);
    convDS_(Wheel::type.contact.surface.soft_cfm, s);
    cfm->setText(s);
    convDS_(Wheel::type.contact.surface.slip1, s);
    slip1->setText(s);
    convDS_(Wheel::type.contact.surface.slip2, s);
    slip2->setText(s);
 }

  
  Window *button = windowMgr_->getWindow("root/car_param/go");
  
  button->subscribeEvent(PushButton::EventClicked,
			 Event::Subscriber(&OgreFramework::changeCarValue,
					   OgreFramework::getSingletonPtr()));
  					 					 
}


void demo::startDemo(){
  new OgreFramework();
  if(!OgreFramework::getSingletonPtr()->initOgre("demo v1.0", this, 0))
    return;

  m_bShutdown = false;

  _glb.nbFrame = 0;

  log_("Demo initialized");
	
  setupDemoScene();

  setupCegui();
  
  runDemo();
}



void demo::setupDemoScene(){
  sceneMgr_->createLight("Light")->setPosition(75,75,75);
  sceneMgr_->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE);
  sceneMgr_->setSkyDome(true, "Sky/CloudySky", 5, 8, 100);

  new FlatGround("Ground/Dirt");
  
  Obstacle o("obstacle","obstacle.mesh",1.0,1.0,-20.0);
  o.setMaterial("Obstacle/Essai");

  Obstacle ramp("ramp","ramp.mesh",25.0,1.0,-20.0);
  ramp.setMaterial("Obstacle/Ramp");

  Obstacle ramp_side("ramp_side","ramp_side.mesh",60.0,1.0,-20.0);
  ramp_side.setMaterial("Obstacle/Ramp_Side");

  MovableObstacle *cross = new MovableObstacle("cross", "cross.mesh", -60.0, 0.7, 20.0);
  _glb.cross = (void*)cross;

  {
    static DContactType type(Type::OBSTACLE_SPACE);
    type.contact.surface.mode= dContactBounce | dContactSoftCFM
      | dContactSoftERP | dContactSlip1 | dContactSlip2;
    type.contact.surface.mu = dInfinity;
    type.contact.surface.bounce = 1.0;
    type.contact.surface.bounce_vel = 0.1;
    type.contact.surface.soft_cfm = 0.01;  
    type.contact.surface.soft_erp = 0.3;  
    type.contact.surface.slip1 = 0.01;
    type.contact.surface.slip2 = 0.01;

    dSpaceID stairSpace = World::getSingletonPtr()->addSimpleSpace();
    dGeomSetData((dGeomID)stairSpace,(void*)&type);
    dSpaceID space = World::getSingletonPtr()->getSpace();
    Obstacle stair("starirway","stairway.mesh",-20.0, 3.5/2, 0.0);
    stair.setMaterial("Obstacle/Stairway");
    dSpaceRemove(space,stair.getGeom());
    dSpaceAdd(stairSpace,stair.getGeom());
    Obstacle stair1("starirway1","stairway.mesh",-20.0, 3.5 +3.5/2 +0.01, -20.0);
    stair1.setMaterial("Obstacle/Stairway");
    dSpaceRemove(space,stair1.getGeom());
    dSpaceAdd(stairSpace,stair1.getGeom());
    Obstacle stair2("starirway2","stairway.mesh",-20.0, 3.5*2 +3.5/2 +0.01, -20.0*2);
    stair2.setMaterial("Obstacle/Stairway");
    dSpaceRemove(space,stair2.getGeom());
    dSpaceAdd(stairSpace,stair2.getGeom());
  }
   
  extern Car car;
  car.init("car", sceneMgr_->getRootSceneNode());
}



void demo::forFrameDo(unsigned int time){
  _glb.nbFrame++;
  
  //simulation loop
  World::getSingletonPtr()->update();
   
  {//updating visuals objects with physicals ones
    for(int i=0; i<geoms.size(); i++){
      MyTools::byOdeToOgre(geoms[i] ,sceneMgr_->getSceneNode(names[i].c_str()));
    }

    ((MovableObstacle*)_glb.cross)->update();

    extern Car car;
    car.update();

    OgreFramework::getSingletonPtr()->updateGui();
  }
    
}
void demo::runDemo(){
  log_("Start main loop");
	
  double timeSinceLastFrame =0;

  log_("reset stats");
  renderWnd_->resetStatistics();

  while(!m_bShutdown && !OgreFramework::getSingletonPtr()->isOgreToBeShutDown()){

    if(renderWnd_->isClosed())
      m_bShutdown = true;

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_LINUX
    Ogre::WindowEventUtilities::messagePump();
#endif	
      if(renderWnd_->isActive()){
	double startTime = timer_->getMillisecondsCPU();
	
	OgreFramework::getSingletonPtr()->m_pKeyboard->capture();
	OgreFramework::getSingletonPtr()->m_pMouse->capture();
	OgreFramework::getSingletonPtr()->updateOgre(timeSinceLastFrame);
	OgreFramework::getSingletonPtr()->m_pRoot->renderOneFrame();

	const int timeForEachFrame = 10;
	forFrameDo(timeForEachFrame);

	timeSinceLastFrame = timer_->getMillisecondsCPU() - startTime;
	int lazyTime=timeForEachFrame-timeSinceLastFrame;
	if(lazyTime>0){
	  usleep(lazyTime*1000);//usleep mean micro
	}
      }
      else{
	usleep(1000);
      }
  }

  log_("Main loop quit");
  log_("Shutdown OGRE");
}

bool demo::keyPressed(const OIS::KeyEvent &keyEventRef){
  OgreFramework::getSingletonPtr()->keyPressed(keyEventRef);
  
  extern Car car;
	
  
  switch(keyEventRef.key){
    
  case OIS::KC_ESCAPE :
    OgreFramework::getSingletonPtr()->quit();
    return true;

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

  case OIS::KC_K :
    car.lowRideBack();
    break;

  case OIS::KC_I :
    car.lowRideFront();
    break;
  
    /*  case OIS::KC_U :
    car.dropDoors();
    break;*/

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


