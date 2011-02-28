#include "car.hpp"
#include "mytools.hpp"
#include <sstream>
#include "world.hpp"

Wheel::Wheel(){}

DContactType Wheel::type(Type::CAR_WHEEL);
DContactType Car::type(Type::CAR);

void Wheel::init(dSpaceID s){
  //g = dCreateSphere(0, W_RADIUS);
  g = dCreateCylinder(0, W_RADIUS,0.5);

  Wheel::type.contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  Wheel::type.contact.surface.mu = dInfinity;
  Wheel::type.contact.surface.bounce = 1.0;
  Wheel::type.contact.surface.bounce_vel = 0.1;
  Wheel::type.contact.surface.soft_cfm = 0.01;  
  Wheel::type.contact.surface.soft_erp = 0.3;  
  Wheel::type.contact.surface.slip1 = 0.5;
  Wheel::type.contact.surface.slip2 = 0.5;

  dGeomSetData(g,(void*)&Wheel::type);
  
  dSpaceAdd(s,g);
  //  dMassSetSphere (&m, W_DENSITY, W_RADIUS);
  dMassSetCylinder(&m, 1,W_DENSITY, W_RADIUS, 0.5);
  b=World::getSingletonPtr()->add(g,&m);

  dMatrix3 R;
  const dReal PI=3.14159265;
  static short int nbWheel = 1;

  if (nbWheel%2)
    dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, PI/2);
  else
    dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, -PI/2);//beware of the sgn of PI/2
  dBodySetRotation(b,R);
  nbWheel+=1;

  //  dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, PI/20);
  //  dBodySetRotation(b,R);
}

void Wheel::update(){
  Ogre::SceneNode *n=sceneMgr_->getSceneNode(name.c_str());
  MyTools::byOdeToOgre(g, n );
}

dBodyID Wheel::getBody(){
  return b;
}

Wheel::~Wheel(){}

void Wheel::create(dSpaceID s, Wheel::Position p, Ogre::SceneNode *node)
{
  init(s);
  Ogre::Entity *e;
  Ogre::SceneNode *n;

  static unsigned int nb=0;
  {
    std::string nam="wheel";
    std::ostringstream out;
    out << nb++;
    nam+=out.str();
    name=nam;
  }
  pos=p;
  n  = node->createChildSceneNode(name.c_str());
  e = sceneMgr_->createEntity((name+"_tire").c_str(), "wheel_tire.mesh");
  e->setMaterialName("Wheels/Tire");
  e->setCastShadows(true);
  n->attachObject(e);
  e = sceneMgr_->createEntity((name+"_hubcap").c_str(), "wheel_hubcap.mesh");
  e->setMaterialName("Wheels/Hubcap");
  n->attachObject(e);
  //n->scale();
  //  n->yaw(Ogre::Degree(90));

  dReal d[3];
  getPositionFromCar(d);
  dBodySetPosition(b, d[0], d[1], d[2]);
  dMatrix3 R;
  const dReal PI=3.14159265;
  MyTools::byOdeToOgre(g, n);

  //ugly
  if (nb > 2) {
    n->scale(2.45, 2.45, 2.45);
  }
  else {
    n->scale(2.45, 2.45, 2.45);
  }
}

void Car::update() {
  //graphical
  MyTools::byOdeToOgre(g, sceneMgr_->getSceneNode(nodeName.c_str()));
  
  Ogre::SceneNode *lnode=sceneMgr_->getSceneNode("left_door");
  MyTools::byOdeToOgre(leftDoorBody, lnode);

  Ogre::SceneNode *rnode=sceneMgr_->getSceneNode("right_door");
  MyTools::byOdeToOgre(rightDoorBody, rnode);


  for (int i = 0; i < 4; i++)
    w[i].update();

  //physical
  updateMotor();
  updateSteering();
}

Car::~Car(){}
Car::Car(): speed(0.0), steer(0.0), g(NULL), b(NULL) ,brake(false){}

void Car::dropDoors() {
  if (dJointIsEnabled(leftDoorJoint)) {
    dJointDisable(leftDoorJoint);
    dJointDisable(rightDoorJoint);
  }
  else {
    dJointEnable(leftDoorJoint);
    dJointEnable(rightDoorJoint);
  }
}

void Car::init(const char *n, Ogre::SceneNode *no){
  //if Ogre isn't set...
  if(sceneMgr_==NULL){
    log_("Ogre isn't lunched before the car is set up");
    exit(0);
  }
  
  //if Ode isn't set...
  if(_glb.worldUp==false){
    log_("Ode isn't lunched before the car is set up");
    exit(0);
  }
  
  nodeName=n;
  
  Ogre::SceneNode *node;
  Ogre::Entity *e;  
  //  Ogre::Entity *b_car;    

  //  e = sceneMgr_->createEntity("subframe","subframe.mesh");
  //  e->setMaterialName("Car/Subframe");
  node = no->createChildSceneNode(n);
  //  node->attachObject(e);
  //  node->scale(0.9, 0.9, 0.9);

  Ogre::SceneNode *fnode = node->createChildSceneNode("ford");
  fnode->scale(0.35, 0.35, 0.35);
  fnode->yaw(Ogre::Degree(180));
  fnode->translate(0.0, 1.9, 0.0);

  Ogre::SceneNode *cam = fnode->createChildSceneNode("cam_pos");
  cam->translate(0.0, 9.0, -15.0);

  Ogre::SceneNode *camT = fnode->createChildSceneNode("cam_target");
  camT->translate(0.0, 4.0, 5.0);

  cam->setAutoTracking (true, camT);
  cam->setFixedYawAxis (true); 

  e=sceneMgr_->createEntity("bonet", "bonet.mesh");
    e->setMaterialName("Ford/Top");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("back", "back.mesh");
    e->setMaterialName("Ford/Back");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("front", "front.mesh");
    e->setMaterialName("Ford/Front");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("bottom", "bottom.mesh");
    e->setMaterialName("Ford/Bottom");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("top", "top.mesh");
    e->setMaterialName("Ford/Top");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("wind_window", "wind_window.mesh");
    e->setMaterialName("Ford/TopWindow");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("back_top", "back_top.mesh");
    e->setMaterialName("Ford/Top");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("back_window", "back_window.mesh");
    e->setMaterialName("Ford/TopWindow");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("wind_window_frame", "wind_window_frame.mesh");
    e->setMaterialName("Ford/Top");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("left_back", "left_back.mesh");
    e->setMaterialName("Ford/LeftDoor");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("left_front", "left_front.mesh");
    e->setMaterialName("Ford/LeftDoor");
    fnode->attachObject(e); 

    Ogre::SceneNode *lnode = sceneMgr_->getRootSceneNode()->createChildSceneNode("left_door");
    {
      lnode->scale(0.35, 0.35, 0.35);

      e=sceneMgr_->createEntity("left_door", "left_door.mesh");
      e->setMaterialName("Ford/LeftDoor");
      lnode->attachObject(e); 
      
      e=sceneMgr_->createEntity("left_window", "left_window.mesh");
      e->setMaterialName("Ford/LeftWindow");
      lnode->attachObject(e); 
    
      e=sceneMgr_->createEntity("left_little_window", "left_little_window.mesh");
      e->setMaterialName("Ford/LeftWindow");
      lnode->attachObject(e); 
    }

    e=sceneMgr_->createEntity("right_back", "right_back.mesh");
    e->setMaterialName("Ford/RightDoor");
    fnode->attachObject(e); 

    e=sceneMgr_->createEntity("right_front", "right_front.mesh");
    e->setMaterialName("Ford/RightDoor");
    fnode->attachObject(e); 

    Ogre::SceneNode *rnode = sceneMgr_->getRootSceneNode()->createChildSceneNode("right_door");
    {
      rnode->scale(0.35, 0.35, 0.35);

      e=sceneMgr_->createEntity("right_door", "right_door.mesh");
      e->setMaterialName("Ford/RightDoor");
      rnode->attachObject(e); 
      
      e=sceneMgr_->createEntity("right_window", "right_window.mesh");
      e->setMaterialName("Ford/RightWindow");
      rnode->attachObject(e); 
      
      e=sceneMgr_->createEntity("right_little_window", "right_little_window.mesh");
      e->setMaterialName("Ford/RightWindow");
      rnode->attachObject(e); 
    }
  
  //partie physique
  
  space=World::getSingletonPtr()->addSimpleSpace();
  dSpaceSetCleanup(space,0); //to avoid destroying geoms when  the space is destroyed
  //when is it destroyed?

  const dReal x=7.48 , y=0.60 , z=17.56 ;
  g= addBox ( x, y, z);
  Car::type.contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  Car::type.contact.surface.mu = dInfinity;
  Car::type.contact.surface.bounce = 0.01;
  Car::type.contact.surface.bounce_vel = 0.7;
  Car::type.contact.surface.soft_cfm = 0.01;  
  Car::type.contact.surface.soft_erp = 0.3;  
  Car::type.contact.surface.slip1 = 0.07;
  Car::type.contact.surface.slip2 = 0.07;

  dGeomSetData(g,(void*)&Car::type);

  dGeomSetData((dGeomID)space,(void*)&Car::type);

  dMassSetBox(&m, 1.0, 1.3 * x, 2*y, z);
  b=World::getSingletonPtr()->add(g,&m);
  dGeomSetPosition (g, C_X, C_Y, C_Z);
  
  dGeomSetOffsetPosition(g, 0.0, 0.3, 0.0);

  {//left door
    const dReal x = 9.513 * 0.35;
    const dReal z = 7.612 * 0.35;
    const dReal y = 0.404 * 0.35;


    {
      const dReal xbis = 1.183 * 0.35 * 2.0;
      const dReal zbis = 7.61 * 0.35 * 2.0;
      const dReal ybis = 5.128 * 0.35 * 2.0;
      
      leftDoorGeom = addBox ( xbis, ybis, zbis);
    }

    dMassSetBox(&leftDoorMass, 1.0, 1.0, 1.5, 2.0);
    leftDoorBody = World::getSingletonPtr()->add(leftDoorGeom, &leftDoorMass);
    dGeomSetData(leftDoorGeom, (void*)&Car::type);
    
    //rotation 90
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, 3.14159);
        dBodySetRotation(leftDoorBody, R);
    //translation
	{
	  const dReal xb = 1.183 * 0.35;
	  const dReal zb = 7.61 * 0.35;
	  const dReal yb = 0.38 * 0.35;

	  dGeomSetOffsetPosition(leftDoorGeom, xb, -yb, -zb);
	}
	dBodySetPosition(leftDoorBody, -x, C_Y - y +2.2, -z);
	//creation du joint
	leftDoorJoint = World::getSingletonPtr()->addHinge(leftDoorBody, b, 0);
	dJointSetHingeParam (leftDoorJoint, dParamLoStop, -3.14159/3);
	dJointSetHingeParam (leftDoorJoint, dParamHiStop, .0);
	//    dJointSetHingeParam (leftDoorJoint, dParamStopERP, 1.0);
	//    dJointSetHingeParam (leftDoorJoint, dParamStopCFM, 1.0);

    dJointSetHingeAxis(leftDoorJoint, 0, 1, 0);
    dJointSetHingeAnchor(leftDoorJoint, -x, C_Y -y, -z);

    MyTools::byOdeToOgre(leftDoorBody, lnode);    
  }


  {//right door
    const dReal x = 9.513 * 0.35;
    const dReal z = 7.612 * 0.35;
    const dReal y = 0.404 * 0.35;


    {
      const dReal xbis = 1.183 * 0.35 * 2.0;
      const dReal zbis = 7.61 * 0.35 * 2.0;
      const dReal ybis = 5.128 * 0.35 * 2.0;
      
      rightDoorGeom = addBox ( xbis, ybis, zbis);
    }

    dMassSetBox(&rightDoorMass, 1.0, 1.0, 1.5, 2.0);
    rightDoorBody = World::getSingletonPtr()->add(rightDoorGeom, &rightDoorMass);
    dGeomSetData(rightDoorGeom, (void*)&Car::type);
    
    //rotation 90
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, -3.14159);
        dBodySetRotation(rightDoorBody, R);
    //translation
	{
	  const dReal xb = 1.183 * 0.35;
	  const dReal zb = 7.61 * 0.35;
	  const dReal yb = 0.38 * 0.35;

	  dGeomSetOffsetPosition(rightDoorGeom, -xb, -yb, -zb);
	}
	dBodySetPosition(rightDoorBody, +x, C_Y - y +2.2, -z);
	//creation du joint
	rightDoorJoint = World::getSingletonPtr()->addHinge(rightDoorBody, b, 0);
	dJointSetHingeParam (rightDoorJoint, dParamLoStop, .0);
	dJointSetHingeParam (rightDoorJoint, dParamHiStop, 3.13159/3);
	//    dJointSetHingeParam (rightDoorJoint, dParamStopERP, 1.0);
	//    dJointSetHingeParam (rightDoorJoint, dParamStopCFM, 1.0);

    dJointSetHingeAxis(rightDoorJoint, 0, 1, 0);
    dJointSetHingeAnchor(rightDoorJoint, x, C_Y -y, -z);

    MyTools::byOdeToOgre(rightDoorBody, rnode);    
  }

 
  MyTools::byOdeToOgre(g, node);
  
  {
    Ogre::SceneNode* carNode=sceneMgr_->getRootSceneNode();
    struct Wheel::Position tmp[4]={
      {W_FR_X, W_FR_Y, W_FR_Z},
      {W_FL_X, W_FL_Y, W_FL_Z},
      {W_BR_X, W_BR_Y, W_BR_Z},
      {W_BL_X, W_BL_Y, W_BL_Z} };
    
    for(int i=0; i<4; i++)
      w[i].create(space, tmp[i], carNode);
  }
 
  //joints part
  for(int i=0;i<4; i++){
    j[i]=World::getSingletonPtr()->addHinge2(b,w[i].getBody(),0);
    const dReal *a=dBodyGetPosition(w[i].getBody());
    dJointSetHinge2Anchor(j[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1(j[i],0,1,0);
    dJointSetHinge2Axis2(j[i],1,0,0);

    /*
      ERP = h kp / (h kp + kd)
      CFM = 1 / (h kp + kd)
    */

#define KP 40.0
#define KD 5.0

    dJointSetHinge2Param(j[i], dParamSuspensionERP, 0.2 * KP / (0.2 * KP + KD));
    dJointSetHinge2Param(j[i], dParamSuspensionCFM, 1 / (0.2 * KP + KD));
  }

  //prismatic & rotoid
  //  dJointID c = dJointCreatePR( World::getSingletonPtr()->getWorld() ,0); wtf?
  // dJointAttach(c, w[0].getBody(), w[1].getBody());

  //forbid all Y rotation for j[3] j[4]
  for (int i = 0; i < 4; i++) {
    dJointSetHinge2Param (j[i],dParamLoStop,0);
    dJointSetHinge2Param (j[i],dParamHiStop,0);

    if (i > 1) {
      dJointSetHinge2Param (j[i],dParamLoStop2,0);//TEST
      dJointSetHinge2Param (j[i],dParamHiStop2,0);//
      dJointSetHinge2Param (j[i],dParamStopERP, 1.0); //normaly to get the the back wheel not rotate
      dJointSetHinge2Param (j[i],dParamStopCFM, 0.0);
    }

  }

  /*  for(int i=0; i<2; i++){
    const int PI = 3.14159265;
    dJointSetHinge2Param (j[i],dParamLoStop, -PI/3);
    dJointSetHinge2Param (j[i],dParamHiStop,PI/3);
    }*/

}

void Car::reset() {
  dGeomSetPosition (g, C_X, C_Y, C_Z);
}

void Car::accelerate(){ speed+=10.5; }

void Car::slowDown(){ speed-=20.5; }

void Car::setSpeed(float s){ speed=s; }

void Car::turnRight(){ steer+=0.08; }

void Car::turnLeft(){ steer-=0.08; }

void Car::setSteer(float s){ steer=s; }

void Car::updateSteering() {
  static float st=0.0;
  if (steer == 0) {
    st=0;
    for (int i = 0; i < 2; i++) {
      dJointSetHinge2Param (j[i], dParamLoStop, 0);
      dJointSetHinge2Param (j[i], dParamHiStop, 0);
    }
  }
  else {
    st+=steer;
 
    for(int i=0; i<2; i++)
    {
      dReal v = st - dJointGetHinge2Angle1 (j[i]);
      if (v > 0.1) v = 0.1;
      if (v < -0.1) v = -0.1;
      v *= 10.0;
      dJointSetHinge2Param (j[i],dParamVel,v);
      dJointSetHinge2Param (j[i],dParamFMax,1.2); //
      dJointSetHinge2Param (j[i],dParamLoStop, -0.65);
      dJointSetHinge2Param (j[i],dParamHiStop, 0.65);
      dJointSetHinge2Param (j[i],dParamFudgeFactor,1.0);
    }
  // {
  //   dReal v = st - dJointGetHinge2Angle1 (j[1]);
  //   if (v > 0.1) v = 0.1;
  //   if (v < -0.1) v = -0.1;
  //   v *= 10.0;
  //   dJointSetHinge2Param (j[1],dParamVel,v);
  //   dJointSetHinge2Param (j[1],dParamFMax,0.2);
  //   dJointSetHinge2Param (j[1],dParamLoStop,-0.75);
  //   dJointSetHinge2Param (j[1],dParamHiStop,0.75);
  //   dJointSetHinge2Param (j[1],dParamFudgeFactor,0.1);
  // }
  }
}

void Car::lowRideFront() {
  dBodyAddRelForceAtRelPos(b, 0.0, 500.0, 0.0, 0.0, 0.0, W_FR_Z);
}

void Car::lowRideBack() {
  dBodyAddForceAtRelPos(b, 0.0, 500.0, 0.0, 0.0, 0.0, W_BR_Z);
}

void Car::setBrake(bool b){
  brake = b;
  if (brake == true) {
    Wheel::type.contact.surface.slip1 = 1.0;
    Wheel::type.contact.surface.slip2 = 1.0;
  }
  else {
    Wheel::type.contact.surface.slip1 = 0.5;
    Wheel::type.contact.surface.slip2 = 0.5;
  }
}

void Car::updateMotor(){
  const float max=50.0;
  const float palier1=30.0;

  if(brake){
    for(int i = 2; i<4; i++){
      dJointSetHinge2Param(j[i], dParamVel2, 0);
      dJointSetHinge2Param(j[i], dParamFMax2, 5*10.5);        
      }
    return ;
    }

  static float sp=0.0;
  if (speed == 0) {
    sp = 0;
    for (int i = 2; i < 4; i++)
      dJointSetHinge2Param(j[i], dParamFMax2, 0.01);
    return ;
  }
  /*  else{
    float y= speed <0 ? -25.0 : 11.0;
    dBodyAddRelForceAtRelPos(b, 0.0, y, 0.0, 
			     0.0, 0.0, W_FR_Z ); //adding a lift up effect of the car's acceleration
    sp+=speed;
    }*/

  if (speed < -10) speed = -10;
  else if (speed > 10) speed = 10;

  sp += speed;
  if (sp < -100) sp = -100;
  else if (sp > 10) sp = 10;

  for (int i = 2; i < 4; i++) {
    dJointSetHinge2Param(j[i], dParamVel2, sp);
    dJointSetHinge2Param(j[i], dParamFMax2, 4.0 * 10.0);    
  }

  /*if (sp < palier1) sp+=speed;
  else if(sp>max) sp=max;


 for (int i = 2; i < 4; i++) {
    dJointSetHinge2Param(j[i], dParamVel2, sp);

    if(sp<0)
      dJointSetHinge2Param(j[i], dParamFMax2, 1.5*10.5);    
    else if(sp>100*speed)
      dJointSetHinge2Param(j[i], dParamFMax2, 1.0*10.5);
    else if(sp>50*speed)
      dJointSetHinge2Param(j[i], dParamFMax2, 1.5*10.5);
    else
      dJointSetHinge2Param(j[i], dParamFMax2, 5.5*10.5);
      }*/
}

Ogre::Vector3 Car::cam() {
    const dReal *pos = dBodyGetPosition (b);
    return Ogre::Vector3((Ogre::Real)pos[0],
			 (Ogre::Real)pos[1]+3.2, 
			 (Ogre::Real)pos[2]);
}

Ogre::Vector3 Car::getPosition() {
  return sceneMgr_->getSceneNode(nodeName.c_str())->getPosition();
}

Ogre::Vector3 Car::getDirection() {
    const dReal *pos = dBodyGetPosition (b);
    return Ogre::Vector3((Ogre::Real)pos[0],
			 (Ogre::Real)pos[1], 
			 (Ogre::Real)pos[2]);
    //  return sceneMgr_->getSceneNode(nodeName.c_str())->getDirection();
}


Ogre::Quaternion Car::getOrientation() {
  return sceneMgr_->getSceneNode(nodeName.c_str())->getOrientation();
}


void Car::swayBars() {
  const float swayForce = 400.0;
  const float swayForceLimit = 40.0;

  for(int i = 0; i < 4; ++i) {
    
    dVector3 tmpAnchor2, tmpAnchor1, tmpAxis;
    dJointGetHinge2Anchor2( j[i], tmpAnchor2 );//not sure that the axis are the good one
    dJointGetHinge2Anchor( j[i], tmpAnchor1 );
    dJointGetHinge2Axis1( j[i], tmpAxis );

    Ogre::Vector3 anchor2((Ogre::Real*)tmpAnchor2);
    Ogre::Vector3 anchor1((Ogre::Real*)tmpAnchor1);
    Ogre::Vector3 axis((Ogre::Real*)tmpAxis);
    float displacement;      

    displacement = (anchor1 - anchor2).dotProduct(axis);

    if (displacement > 0) {
      //      std::cout<<"displacement > 0"<<std::endl;
      float amt = displacement * swayForce;
      if (amt > swayForceLimit) {
	amt = swayForceLimit;
	std::cout<<"limit reach"<<std::endl;
      }
      //the axis are inversed
      //dBodyAddForce( w[i].getBody(), -axis.x * amt, -axis.y * amt, -axis.z * amt );
      dReal const * wp = dBodyGetPosition( w[i].getBody() );
      dBodyAddForceAtPos( b, -axis.x*amt, -axis.y*amt, -axis.z*amt, wp[0], wp[1], wp[2] );
      //dBodyAddForce( w[i^1].getBody(), axis.x * amt, axis.y * amt, axis.z * amt );
      wp = dBodyGetPosition( w[i^1].getBody() );
      dBodyAddForceAtPos( b, axis.x*amt, axis.y*amt, axis.z*amt, wp[0], wp[1], wp[2] );
      }
   }
}


#include <math.h>

dReal Car::getPunch() {
  const dReal *V = dBodyGetLinearVel(b);

  /*  std::cout<<"Force: "<<V[0]<<" - "<<V[1]<<" - "<<V[2]<<std::endl;

  {
  const dReal *T = dBodyGetTorque(b);
  std::cout<<"Torque: "<<T[0]<<" - "<<T[1]<<" - "<<T[2]<<std::endl;
  }

  {
    const dReal *T = dBodyGetLinearVel(b);
    std::cout<<"Vel: "<<T[0]<<" - "<<T[1]<<" - "<<T[2]<<std::endl;
    }*/

  return sqrt(pow(V[0], 2) + pow(V[1], 2) + pow(V[2], 2)) * m.mass;
}
