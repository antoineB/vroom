#include "car.hpp"
#include "mytools.hpp"
#include <sstream>
#include "world.hpp"
#include <math.h>

using namespace Utils;

DContactType Car::type(Type::CAR);

void Car::update() {
  //graphical
  MyTools::byOdeToOgre(ph.geom, cst.carNode);
  //  MyTools::byOdeToOgre(ph.leftDoor.body, cst.leftDoorNode);
  //  MyTools::byOdeToOgre(ph.rightDoor.body, cst.rightDoorNode);

  for (int i = 0; i < 4; i++)
    ph.wheels[i].update();

  //physical
  updateMotor();
  updateSteering();
}


void Car::fillContact() {
  Car::type.contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  Car::type.contact.surface.mu = dInfinity;
  Car::type.contact.surface.bounce = 0.01;
  Car::type.contact.surface.bounce_vel = 0.7;
  Car::type.contact.surface.soft_cfm = 0.01;  
  Car::type.contact.surface.soft_erp = 0.3;  
  Car::type.contact.surface.slip1 = 0.07;
  Car::type.contact.surface.slip2 = 0.07;
}

void Car::fillContact(Conf::Car::Param &mod) {
  memcpy(&Car::type.contact.surface.mu, &mod.contact.surface.mu, sizeof(mod.contact.surface) - sizeof(mod.contact.surface.mode));
}

Car::~Car(){}

Car::Car(): speed(0.0), steer(0.0), brake(false){}

void Car::createSpace() {
  space = World::getSingletonPtr()->addSimpleSpace();
  //to avoid destroying geoms when the space is destroyed
  dSpaceSetCleanup(space,0);
}

void Car::createPhysics() {
  fillContact();

  ph.geom = addBox( Conf::Car::BOX[0], Conf::Car::BOX[1], Conf::Car::BOX[2]);
  dGeomSetData(ph.geom,(void*)&Car::type);
  dGeomSetData((dGeomID)space,(void*)&Car::type);
  dMassSetBox(&ph.mass, 1.0, Conf::Car::BOX[0], Conf::Car::BOX[1], Conf::Car::BOX[2]);
  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);

  createJoints();
}

void Car::createPhysics(Utils::Xml &x) {
  ph.geom = addBox(x.mustOReal("box.x"), x.mustOReal("box.y"), x.mustOReal("box.z"));
  dGeomSetData(ph.geom,(void*)&Car::type);
  dGeomSetData((dGeomID)space,(void*)&Car::type);
  dMassSetBoxTotal(&ph.mass, x.mustOReal("mass"), x.mustOReal("box.x"),
		   x.mustOReal("box.y"), x.mustOReal("box.z"));
  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);

  createJoints(x);
}

void Car::createPhysics(Conf::Car::Param &mod) {
  memcpy(&ph.mass, &mod.mass, sizeof(dMass));

  ph.geom = addBox( mod.box[0], mod.box[1], mod.box[2]);
  dGeomSetData(ph.geom,(void*)&Car::type);
  dGeomSetData((dGeomID)space,(void*)&Car::type);
  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);

  createJoints();
}


void Car::createJoints() {
  for(int i = 0; i < 4; i++) {
    ph.joints[i] = World::getSingletonPtr()->addHinge2(ph.body , ph.wheels[i].ph.body, 0);

    /*
      ERP = h kp / (h kp + kd)
      CFM = 1 / (h kp + kd)
    */

    const dReal KP = 40.0;
    const dReal KD = 5.0;

    dJointSetHinge2Param(ph.joints[i], dParamSuspensionERP, 0.2 * KP / (0.2 * KP + KD));
    dJointSetHinge2Param(ph.joints[i], dParamSuspensionCFM, 1 / (0.2 * KP + KD));
  
    //joints can't turn around Y
    dJointSetHinge2Param (ph.joints[i],dParamLoStop,0);
    dJointSetHinge2Param (ph.joints[i],dParamHiStop,0);
  
    if (i > 1) { //back wheels
      dJointSetHinge2Param (ph.joints[i],dParamStopERP, 1.0); //normaly to get the the back wheel not rotate
      dJointSetHinge2Param (ph.joints[i],dParamStopCFM, 0.0);
    }
    else { //front wheels
      //choose ERP, CFm for front wheels
    }
  }
}

void Car::createJoints(Utils::Xml &x) {
  for(int i = 0; i < 4; i++) {
    ph.joints[i] = World::getSingletonPtr()->addHinge2(ph.body , ph.wheels[i].ph.body, 0);
    dJointSetHinge2Param (ph.joints[i],dParamLoStop,0);
    dJointSetHinge2Param (ph.joints[i],dParamHiStop,0);
  
    if (i > 1) {  //to get the the back wheel not rotate in y
      dJointSetHinge2Param (ph.joints[i],dParamStopERP, 1.0);
      dJointSetHinge2Param (ph.joints[i],dParamStopCFM, 0.0);
    }
  }
  dJointSetHinge2Param(ph.joints[0], dParamSuspensionERP, 
		       x.mustOReal("joints.front-right.erp"));
  dJointSetHinge2Param(ph.joints[0], dParamSuspensionCFM,
		       x.mustOReal("joints.front-right.cfm"));
  
  dJointSetHinge2Param(ph.joints[1], dParamSuspensionERP, 
		       x.mustOReal("joints.front-left.erp"));
  dJointSetHinge2Param(ph.joints[1], dParamSuspensionCFM, 
		       x.mustOReal("joints.front-left.cfm"));
  
  dJointSetHinge2Param(ph.joints[2], dParamSuspensionERP, 
		       x.mustOReal("joints.back-right.erp"));
  dJointSetHinge2Param(ph.joints[2], dParamSuspensionCFM, 
		       x.mustOReal("joints.back-right.cfm"));
  
  dJointSetHinge2Param(ph.joints[3], dParamSuspensionERP, 
		       x.mustOReal("joints.back-left.erp"));
  dJointSetHinge2Param(ph.joints[3], dParamSuspensionCFM,
		       x.mustOReal("joints.back-left.cfm"));
    
}

void Car::createJoints(Conf::Car::Param &mod) {
  for(int i = 0; i < 4; i++) {
    ph.joints[i] = World::getSingletonPtr()->addHinge2(ph.body , ph.wheels[i].ph.body, 0);

    const dReal KP = 40.0;
    const dReal KD = 5.0;

    dJointSetHinge2Param(ph.joints[i], dParamSuspensionERP, 0.2 * KP / (0.2 * KP + KD));
    dJointSetHinge2Param(ph.joints[i], dParamSuspensionCFM, 1 / (0.2 * KP + KD));
  
    //joints can't turn around Y
    dJointSetHinge2Param (ph.joints[i],dParamLoStop,0);
    dJointSetHinge2Param (ph.joints[i],dParamHiStop,0);
  
    if (i > 1) {
      dJointSetHinge2Param (ph.joints[i],dParamStopERP, 1.0); //normaly to get the the back wheel not rotate
      dJointSetHinge2Param (ph.joints[i],dParamStopCFM, 0.0);
    }
    else {
      //choose ERP, CFm for front wheels
    }
  }
}


void Car::disposePhysics() {
  disposeGeoms();
  disposeJoints();
}

void Car::disposePhysics(Utils::Xml &x) {
  disposeGeoms(x);
  disposeJoints(x);
}

void Car::disposePhysics(Conf::Car::Param &mod) {
  disposeGeoms(mod);
  disposeJoints(mod);
}


void Car::disposeGeoms() {
  dGeomSetPosition (ph.geom, Conf::Car::POS[0], Conf::Car::POS[1], Conf::Car::POS[2]);
  dGeomSetOffsetPosition(ph.geom, Conf::Car::POSOFFSET[0], Conf::Car::POSOFFSET[1], Conf::Car::POSOFFSET[2]);
}

void Car::disposeGeoms(Utils::Xml &x) {
    Ogre::Real a = x.mustOReal("gravity-center.x");
    Ogre::Real b = x.mustOReal("gravity-center.y");
    Ogre::Real c = x.mustOReal("gravity-center.z");

  dBodySetPosition(ph.body, a, b, c);
  dGeomSetOffsetPosition(ph.geom,   
			 x.mustOReal("global-position.x") - a,
			 x.mustOReal("global-position.y") - b,
			 x.mustOReal("global-position.z") - c);
}


void Car::disposeGeoms(Conf::Car::Param &mod) {
  dGeomSetPosition (ph.geom, Conf::Car::POS[0], Conf::Car::POS[1], Conf::Car::POS[2]);
  dGeomSetOffsetPosition(ph.geom, mod.offset[0], mod.offset[1], mod.offset[2]);
}


void Car::disposeJoints() {
  for(int i = 0; i < 4; i++) {
    dJointSetHinge2Anchor(ph.joints[i],
			  Conf::Car::POS[0] + Conf::Wheel::POS[i][0], 
			  Conf::Car::POS[1] + Conf::Wheel::POS[i][1],
			  Conf::Car::POS[2] + Conf::Wheel::POS[i][2]
			  );
    dJointSetHinge2Axis1(ph.joints[i], Conf::Car::AXIS1[i][0], Conf::Car::AXIS1[i][1], Conf::Car::AXIS1[i][2]);
    dJointSetHinge2Axis2(ph.joints[i], Conf::Car::AXIS2[i][0], Conf::Car::AXIS2[i][1], Conf::Car::AXIS2[i][2]);
  }    
}

void Car::disposeJoints(Utils::Xml &x) {
  Ogre::Real a = x.mustOReal("global-position.x");
  Ogre::Real b = x.mustOReal("global-position.y");
  Ogre::Real c = x.mustOReal("global-position.z");

  std::string uris[] = {
    "../xml/" + x.mustString("wheels.uri", 0),
    "../xml/" + x.mustString("wheels.uri", 1),
    "../xml/" + x.mustString("wheels.uri", 2),
    "../xml/" + x.mustString("wheels.uri", 3)
  };			

  std::string names[] = {
    "joints.front-right.axis1.x", "joints.front-right.axis1.y", "joints.front-right.axis1.z",
    "joints.front-right.axis2.x", "joints.front-right.axis2.y", "joints.front-right.axis2.z",

    "joints.front-left.axis1.x", "joints.front-left.axis1.y", "joints.front-left.axis1.z",
    "joints.front-left.axis2.x", "joints.front-left.axis2.y", "joints.front-left.axis2.z",

    "joints.back-right.axis1.x", "joints.back-right.axis1.y", "joints.back-right.axis1.z",
    "joints.back-right.axis2.x", "joints.back-right.axis2.y", "joints.back-right.axis2.z",

    "joints.back-left.axis1.x", "joints.back-left.axis1.y", "joints.back-left.axis1.z",
    "joints.back-left.axis2.x", "joints.back-left.axis2.y", "joints.front-left.axis2.z"   
  };

  for (int i = 0; i < 4; i++) {
    {
      Utils::Xml w(uris[i].c_str(), "wheel");
      dJointSetHinge2Anchor(ph.joints[i],
			    a + w.mustOReal("position.x"), 
			    b + w.mustOReal("position.y"),
			    c + w.mustOReal("position.z")
			    );

      std::cout<<a + w.mustOReal("position.x") << " - " << b + w.mustOReal("position.y") << " - " << c + w.mustOReal("position.z") << std::endl;
		
    }
    dJointSetHinge2Axis1(ph.joints[i], 
			 x.mustOReal(names[i * 6].c_str()),
			 x.mustOReal(names[i * 6 + 1].c_str()),
			 x.mustOReal(names[i * 6 + 2].c_str())
			 );
    dJointSetHinge2Axis2(ph.joints[i], 
			 x.mustOReal(names[i * 6 + 3].c_str()),
			 x.mustOReal(names[i * 6 + 4].c_str()),
			 x.mustOReal(names[i * 6 + 5].c_str())
			 );
  }
}

void Car::disposeJoints(Conf::Car::Param &mod) {
  for(int i = 0; i < 4; i++) {
    dJointSetHinge2Anchor(ph.joints[i],
			  Conf::Car::POS[0] + Conf::Wheel::POS[i][0], 
			  Conf::Car::POS[1] + Conf::Wheel::POS[i][1],
			  Conf::Car::POS[2] + Conf::Wheel::POS[i][2]
			  );
    dJointSetHinge2Axis1(ph.joints[i], mod.axis1[i][0], mod.axis1[i][1], mod.axis1[i][2]);
    dJointSetHinge2Axis2(ph.joints[i], mod.axis2[i][0], mod.axis2[i][1], mod.axis2[i][2]);
  }    
}


void Car::reset(Conf::Param &mod) {
  dJointDestroy(ph.joints[0]);
  dJointDestroy(ph.joints[1]);
  dJointDestroy(ph.joints[2]);
  dJointDestroy(ph.joints[3]);
  dGeomDestroy(ph.geom);
  dBodyDestroy(ph.body);

  ph.wheels[0].reset();
  ph.wheels[1].reset();
  ph.wheels[2].reset();
  ph.wheels[3].reset();

  createPhysics(mod.car);
  disposePhysics(mod.car);

  Wheel::fillContact(mod.car.wheel_contact);
  Car::fillContact(mod.car);
  FlatGround::fillContact(mod.ground);
  Obstacle::fillContact(mod.obstacles);
}


void Car::init(const char *nodeName, Ogre::SceneNode *root) {
  envUp_();

  createSpace();

  createNodesAndMeshes(nodeName, root);
  createCamNodes();

  ph.wheels[0].initXml("../xml/wheel_front_right.xml", space);
  ph.wheels[1].initXml("../xml/wheel_front_left.xml", space);
  ph.wheels[2].initXml("../xml/wheel_back_right.xml", space);
  ph.wheels[3].initXml("../xml/wheel_back_left.xml", space);

  
  createPhysics();
  disposePhysics();

  MyTools::byOdeToOgre(ph.geom, cst.carNode);
}

void Car::createNodesAndMeshes(Utils::Xml &x) {
  Ogre::SceneNode *node = sceneMgr_->getRootSceneNode()->createChildSceneNode(cst.nodeName);
  Ogre::SceneNode *fnode = node->createChildSceneNode("ford");

  cst.carNode = node;
  cst.subCarNode = fnode;

  fnode->scale(0.35, 0.35, 0.35);
  fnode->yaw(Ogre::Degree(x.mustOReal("rotation.y")));
  fnode->translate(0.0, 1.9, 0.0);

  std::string names[] = {
    "bonet", "back", "front", "bottom", "top", "wind_window", "back_top",
    "back_window", "wind_window_frame", "left_back", "left_front",
    "right_back", "right_front"
  };
      
  for (int i = 0; i < 13; i++) {
    std::string mesh("meshes." + names[i]);
    std::string mat("materials." + names[i]);
    createAndAttachEntity(names[i], x.mustString(mesh.c_str()), x.mustString(mat.c_str()), fnode);
  }

  //  createLeftDoorGraphic();
  //  createRightDoorGraphic();
}

void Car::initXml(const char *xmlFile, Ogre::SceneNode *root) {
  envUp_();

  createSpace();

  Utils::Xml x(xmlFile, "car");

  cst.nodeName = x.mustString("name");
  
  createNodesAndMeshes(x);

  std::cerr<<"mesh and nodes created"<<std::endl;
  //createCamNodes(x);

  std::string uris[] = {
    "../xml/" + x.mustString("wheels.uri", 0),
    "../xml/" + x.mustString("wheels.uri", 1),
    "../xml/" + x.mustString("wheels.uri", 2),
    "../xml/" + x.mustString("wheels.uri", 3)
  };			

  for (int i = 0; i < 4; i++)
    ph.wheels[i].initXml(uris[i].c_str(), space);

  std::cerr<<"wheels created"<<std::endl;

  
  createPhysics(x);

  std::cerr<<"physics created"<<std::endl;

  disposePhysics(x);

  std::cerr<<"physics disposed"<<std::endl;


  MyTools::byOdeToOgre(ph.geom, cst.carNode);
}

void Car::printRotationMatrix() {
  const dReal *R = dGeomGetRotation(ph.geom);
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      std::cout<<i<<" "<<j<<" = "<<R[i + j]<<std::endl;
    }
  }
}


void Car::rotateWheels(dMatrix3 *R) {
  for (int i = 0; i < 3; i++)
    dGeomSetRotation(ph.wheels[i].ph.geom, *R);
}



void Car::accelerate(){ speed += 10.5; }


void Car::slowDown(){ speed -= 20.5; }


void Car::setSpeed(float s){ speed=s; }


void Car::turnRight(){ steer+=0.08; }


void Car::turnLeft(){ steer-=0.08; }


void Car::setSteer(float s){ steer=s; }


const dReal* Car::getSpeed() {
  return dBodyGetLinearVel(ph.body);
}


void Car::updateSteering() {
  static float st=0.0;
  if (steer == 0) {
    st=0;
    for (int i = 0; i < 2; i++) {
      dJointSetHinge2Param (ph.joints[i], dParamLoStop, 0);
      dJointSetHinge2Param (ph.joints[i], dParamHiStop, 0);
    }
  }
  else {
    st+=steer;
 
    for(int i=0; i<2; i++)
    {
      dReal v = st - dJointGetHinge2Angle1 (ph.joints[i]);
      if (v > 0.1) v = 0.1;
      if (v < -0.1) v = -0.1;
      v *= 10.0;
      dJointSetHinge2Param (ph.joints[i],dParamVel,v);
      dJointSetHinge2Param (ph.joints[i],dParamFMax,1.2);
      dJointSetHinge2Param (ph.joints[i],dParamLoStop, -0.65);
      dJointSetHinge2Param (ph.joints[i],dParamHiStop, 0.65);
      dJointSetHinge2Param (ph.joints[i],dParamFudgeFactor,1.0);
    }
  }
}


void Car::lowRideFront() {
  dBodyAddRelForceAtRelPos(ph.body, 0.0, 500.0, 0.0, 0.0, 0.0, Conf::Wheel::POS[0][2]);
}


void Car::lowRideBack() {
  dBodyAddForceAtRelPos(ph.body, 0.0, 500.0, 0.0, 0.0, 0.0, Conf::Wheel::POS[2][2]);
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
  if(brake){
    for(int i = 2; i<4; i++){
      //a quoi sert de mettre la vitesse a zero?
      dJointSetHinge2Param(ph.joints[i], dParamVel2, 0);
      dJointSetHinge2Param(ph.joints[i], dParamFMax2, 5*10.5);        
      }
    return ;
    }

  static float sp=0.0;
  if (speed == 0) {
    sp = 0;
    for (int i = 2; i < 4; i++)
      dJointSetHinge2Param(ph.joints[i], dParamFMax2, 0.01);
    return ;
  }

  if (speed < -10) speed = -10;
  else if (speed > 10) speed = 10;

  sp += speed;
  if (sp < -100) sp = -100;
  else if (sp > 10) sp = 10;

  for (int i = 2; i < 4; i++) {
    dJointSetHinge2Param(ph.joints[i], dParamVel2, sp);
    dJointSetHinge2Param(ph.joints[i], dParamFMax2, 4.0 * 10.0);    
  }
}


Ogre::Vector3 Car::cam() {
    const dReal *pos = dBodyGetPosition (ph.body);
    return Ogre::Vector3((Ogre::Real)pos[0],
			 (Ogre::Real)pos[1]+3.2, 
			 (Ogre::Real)pos[2]);
}


Ogre::Vector3 Car::getPosition() {
  return cst.carNode->getPosition();
}


Ogre::Vector3 Car::getDirection() {
    const dReal *pos = dBodyGetPosition (ph.body);
    return Ogre::Vector3((Ogre::Real)pos[0],
			 (Ogre::Real)pos[1], 
			 (Ogre::Real)pos[2]);
    //  return sceneMgr_->getSceneNode(nodeName.c_str())->getDirection();
}


Ogre::Quaternion Car::getOrientation() {
  return cst.carNode->getOrientation();
}


void Car::swayBars() {
  const float swayForce = 400.0;
  const float swayForceLimit = 40.0;

  for(int i = 0; i < 4; ++i) {
    
    dVector3 tmpAnchor2, tmpAnchor1, tmpAxis;
    dJointGetHinge2Anchor2( ph.joints[i], tmpAnchor2 );//not sure that the axis are the good one
    dJointGetHinge2Anchor( ph.joints[i], tmpAnchor1 );
    dJointGetHinge2Axis1( ph.joints[i], tmpAxis );

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
      dReal const * wp = dBodyGetPosition(ph.wheels[i].ph.body);
      dBodyAddForceAtPos(ph.body, -axis.x*amt, -axis.y*amt, -axis.z*amt, wp[0], wp[1], wp[2] );
      //dBodyAddForce( w[i^1].getBody(), axis.x * amt, axis.y * amt, axis.z * amt );
      wp = dBodyGetPosition(ph.wheels[i^1].ph.body );
      dBodyAddForceAtPos(ph.body, axis.x*amt, axis.y*amt, axis.z*amt, wp[0], wp[1], wp[2] );
      }
   }
}


dReal Car::getPunch() {
  const dReal *V = dBodyGetLinearVel(ph.body);
  return sqrt(pow(V[0], 2) + pow(V[1], 2) + pow(V[2], 2)) * ph.mass.mass;
}


dReal Car::getFrontWheelsErp() {
  return dJointGetHinge2Param(ph.joints[0], dParamSuspensionERP);
}


dReal Car::getBackWheelsErp() {
  return dJointGetHinge2Param(ph.joints[2], dParamSuspensionERP);
}


void Car::setBackWheelsErp(dReal erp) {
    dJointSetHinge2Param(ph.joints[2], dParamSuspensionERP, erp);
    dJointSetHinge2Param(ph.joints[3], dParamSuspensionERP, erp);
}


void Car::setFrontWheelsErp(dReal erp) {
  dJointSetHinge2Param(ph.joints[0], dParamSuspensionERP, erp);
  dJointSetHinge2Param(ph.joints[1], dParamSuspensionERP, erp);
}


dReal Car::getFrontWheelsCfm() {
  return dJointGetHinge2Param(ph.joints[0], dParamSuspensionCFM);
}


dReal Car::getBackWheelsCfm() {
    return dJointGetHinge2Param(ph.joints[2], dParamSuspensionCFM);
}


void Car::setBackWheelsCfm(dReal cfm) {
  dJointSetHinge2Param(ph.joints[2], dParamSuspensionCFM, cfm);
  dJointSetHinge2Param(ph.joints[3], dParamSuspensionCFM, cfm);
}


void Car::setFrontWheelsCfm(dReal cfm) {
  dJointSetHinge2Param(ph.joints[0], dParamSuspensionCFM, cfm);
  dJointSetHinge2Param(ph.joints[1], dParamSuspensionCFM, cfm);
}


void Car::createLeftDoorGraphic() {
  Ogre::SceneNode *lnode = sceneMgr_->getRootSceneNode()->createChildSceneNode("left_door");
    
  cst.leftDoorNode = lnode;

  lnode->scale(0.35, 0.35, 0.35);

  createAndAttachEntity("left_door", "left_door.mesh", "Ford/LeftDoor", lnode);
  createAndAttachEntity("left_window", "left_window.mesh", "Ford/LeftWindow", lnode);
  createAndAttachEntity("left_little_window", "left_little_window.mesh", "Ford/LeftWindow", lnode);
}


void Car::createRightDoorGraphic() {
  Ogre::SceneNode *rnode = sceneMgr_->getRootSceneNode()->createChildSceneNode("right_door");

  cst.rightDoorNode = rnode;
    
  rnode->scale(0.35, 0.35, 0.35);

  createAndAttachEntity("right_door", "right_door.mesh", "Ford/RightDoor", rnode); 
  createAndAttachEntity("right_window", "right_window.mesh", "Ford/RightWindow", rnode); 
  createAndAttachEntity("right_little_window", "right_little_window.mesh", "Ford/RightWindow", rnode); 
}


void Car::createAndAttachEntity(const std::string &name, const std::string &meshName, const std::string &materialName, Ogre::SceneNode *node) const {
  Ogre::Entity *e = sceneMgr_->createEntity(name, meshName);
  e->setMaterialName(materialName);
  node->attachObject(e);
}


void Car::setMass(dReal total, dReal x, dReal y, dReal z) {
  dMassSetBoxTotal(&ph.mass, total, 3.0, 3.0, 3.0);
  std::cout<<total<<" "<<x<<" "<<y<<" "<<z<<std::endl;
}


void Car::createNodesAndMeshes(std::string nodeName, Ogre::SceneNode *parentNode) {
  Ogre::SceneNode *node = parentNode->createChildSceneNode(nodeName);
  Ogre::SceneNode *fnode = node->createChildSceneNode("ford");

  cst.nodeName = nodeName;
  cst.carNode = node;
  cst.subCarNode = fnode;

  fnode->scale(0.35, 0.35, 0.35);
  fnode->yaw(Ogre::Degree(180));
  fnode->translate(0.0, 1.9, 0.0);

  {
    std::string names[] = {
      "bonet", "back", "front", "bottom", "top", "wind_window", "back_top",
      "back_window", "wind_window_frame", "left_back", "left_front",
      "right_back", "right_front"
    };
      
    std::string meshNames[] = {
      "bonet.mesh", "back.mesh", "front.mesh", "bottom.mesh", "top.mesh",
      "wind_window.mesh", "back_top.mesh", "back_window.mesh", 
      "wind_window_frame.mesh", "left_back.mesh", "left_front.mesh",
      "right_back.mesh", "right_front.mesh"
    };
    
    std::string materialNames[] = {
      "Ford/Top", "Ford/Back", "Ford/Front", "Ford/Bottom", "Ford/Top",
      "Ford/TopWindow", "Ford/Top", "Ford/TopWindow", "Ford/Top",
      "Ford/LeftDoor", "Ford/LeftDoor", "Ford/RightDoor", "Ford/RightDoor"
    };

    for (int i = 0; i < 13; i++)
      createAndAttachEntity(names[i], meshNames[i], materialNames[i], fnode);
  }

  createLeftDoorGraphic();
  createRightDoorGraphic();

}


void Car::createCamNodes() {
  Ogre::SceneNode *cam = cst.subCarNode->createChildSceneNode("cam_pos");
  //MUST BE CHANGE
  cam->translate(0.0, 9.0, -15.0);
  
  Ogre::SceneNode *camT = cst.subCarNode->createChildSceneNode("cam_target");
  //MUST BE CHANGE
  camT->translate(0.0, 4.0, 5.0);

  //don't what it used for
  cam->setAutoTracking(true, camT);
  cam->setFixedYawAxis(true); 
}
