#include "car.hpp"
#include "mytools.hpp"
#include <sstream>
#include "world.hpp"
#include <math.h>

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

Car::~Car(){}
Car::Car(): speed(0.0), steer(0.0), brake(false){}

void Car::dropDoors() {
  if (dJointIsEnabled(ph.leftDoor.joint)) {
    dJointDisable(ph.leftDoor.joint);
    dJointDisable(ph.rightDoor.joint);
  }
  else {
    dJointEnable(ph.leftDoor.joint);
    dJointEnable(ph.rightDoor.joint);
  }
}

void Car::init(const char *nodeName, Ogre::SceneNode *root){
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
  
  createNodesAndMeshes(nodeName, root);
  createCamNodes();

  //partie physique
  
  space=World::getSingletonPtr()->addSimpleSpace();
  dSpaceSetCleanup(space,0); //to avoid destroying geoms when  the space is destroyed
  //when is it destroyed?

  const dReal x=7.48 , y=0.60 , z=17.56 ;
  ph.geom = addBox ( x, y, z);

  dGeomSetData(ph.geom,(void*)&Car::type);

  dGeomSetData((dGeomID)space,(void*)&Car::type);

  dMassSetBox(&ph.mass, 1.0, 1.3 * x, 2*y, z);
  ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);
  dGeomSetPosition (ph.geom, Conf::Car::POS[0], Conf::Car::POS[1], Conf::Car::POS[2]);
  
  dGeomSetOffsetPosition(ph.geom, Conf::Car::POSOFFSET[0], Conf::Car::POSOFFSET[1], Conf::Car::POSOFFSET[2]);

  /*{//left door
    using namespace Conf::Car::Door;

    ph.leftDoor.geom = addBox ( BOX_BOUND[0], BOX_BOUND[1], BOX_BOUND[2]);
    
    dMassSetBox(&ph.leftDoor.mass, DENSITY, BOX_MASS[0], BOX_MASS[1], BOX_MASS[2]);
    ph.leftDoor.body = World::getSingletonPtr()->add(ph.leftDoor.geom, &ph.leftDoor.mass);
    dGeomSetData(ph.leftDoor.geom, (void*)&Car::type);
    
    //rotation 90
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, Conf::Math::PI);
        dBodySetRotation(ph.leftDoor.body, R);
    //translation
	{
	  const dReal xb = 1.183 * 0.35;
	  const dReal zb = 7.61 * 0.35;
	  const dReal yb = 0.38 * 0.35;

	  dGeomSetOffsetPosition(ph.leftDoor.geom, xb, -yb, -zb);
	}

	dBodySetPosition(ph.leftDoor.body, 
			 Conf::Car::POS[0] + Left::POS[0], 
			 Conf::Car::POS[1] + Left::POS[1] +2.5,
			 Conf::Car::POS[2] + Left::POS[2]
			 );

	//creation du joint
	ph.leftDoor.joint = World::getSingletonPtr()->addHinge(ph.leftDoor.body, ph.body, 0);
	dJointSetHingeParam (ph.leftDoor.joint, dParamLoStop, -Conf::Math::PI/3);
	dJointSetHingeParam (ph.leftDoor.joint, dParamHiStop, .0);
	//    dJointSetHingeParam (ph.leftDoor.joint, dParamStopERP, 1.0);
	//    dJointSetHingeParam (ph.leftDoor.joint, dParamStopCFM, 1.0);

	dJointSetHingeAxis(ph.leftDoor.joint,  Left::JOINT_AXIS[0], Left::JOINT_AXIS[1], Left::JOINT_AXIS[2]);
	dJointSetHingeAnchor(ph.leftDoor.joint,
			     Conf::Car::POS[0] + Left::JOINT_POS[0], 
			     Conf::Car::POS[1] + Left::JOINT_POS[1],
			     Conf::Car::POS[2] + Left::JOINT_POS[2]
			     );

	MyTools::byOdeToOgre(ph.leftDoor.body, cst.leftDoorNode);    
  }

  {//right door
    using namespace Conf::Car::Door;
    
    ph.rightDoor.geom = addBox ( BOX_BOUND[0], BOX_BOUND[1], BOX_BOUND[2]);
    dMassSetBox(&ph.rightDoor.mass, DENSITY, BOX_MASS[0], BOX_MASS[1], BOX_MASS[2]);
    ph.rightDoor.body = World::getSingletonPtr()->add(ph.rightDoor.geom, &ph.rightDoor.mass);
    dGeomSetData(ph.rightDoor.geom, (void*)&Car::type);
    
    //rotation 90
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, -Conf::Math::PI);
        dBodySetRotation(ph.rightDoor.body, R);
    //translation
	{
	  const dReal xb = 1.183 * 0.35;
	  const dReal zb = 7.61 * 0.35;
	  const dReal yb = 0.38 * 0.35;

	  dGeomSetOffsetPosition(ph.rightDoor.geom, -xb, -yb, -zb);
	}
	dBodySetPosition(ph.rightDoor.body, 
			 Conf::Car::POS[0] + Right::POS[0], 
			 Conf::Car::POS[1] + Right::POS[1] +2.5,
			 Conf::Car::POS[2] + Right::POS[2]
			 );

	//creation du joint
	ph.rightDoor.joint = World::getSingletonPtr()->addHinge(ph.rightDoor.body, ph.body, 0);
	dJointSetHingeParam (ph.rightDoor.joint, dParamLoStop, .0);
	dJointSetHingeParam (ph.rightDoor.joint, dParamHiStop, Conf::Math::PI/3);
	//    dJointSetHingeParam (ph.rightDoor.joint, dParamStopERP, 1.0);
	//    dJointSetHingeParam (ph.rightDoor.joint, dParamStopCFM, 1.0);

	dJointSetHingeAxis(ph.rightDoor.joint, Right::JOINT_AXIS[0], Right::JOINT_AXIS[1], Right::JOINT_AXIS[2]);
	dJointSetHingeAnchor(ph.rightDoor.joint, 			 
			     Conf::Car::POS[0] + Right::JOINT_POS[0], 
			     Conf::Car::POS[1] + Right::JOINT_POS[1],
			     Conf::Car::POS[2] + Right::JOINT_POS[2]
			     );

    MyTools::byOdeToOgre(ph.rightDoor.body, cst.rightDoorNode);    
    }*/

 
  MyTools::byOdeToOgre(ph.geom, cst.carNode);
  
  {
    Ogre::SceneNode* carNode=sceneMgr_->getRootSceneNode();
    for(int i=0; i<4; i++)
      ph.wheels[i].create(space, carNode);
  }
 
  //joints part
  for(int i = 0; i < 4; i++){
    ph.joints[i] = World::getSingletonPtr()->addHinge2(ph.body , ph.wheels[i].ph.body, 0);
    dJointSetHinge2Anchor(ph.joints[i],
			  Conf::Car::POS[0] + Conf::Wheel::POS[i][0], 
			  Conf::Car::POS[1] + Conf::Wheel::POS[i][1],
			  Conf::Car::POS[2] + Conf::Wheel::POS[i][2]
			  );
    dJointSetHinge2Axis1(ph.joints[i],0,1,0);
    dJointSetHinge2Axis2(ph.joints[i],1,0,0);

    /*
      ERP = h kp / (h kp + kd)
      CFM = 1 / (h kp + kd)
    */

    const dReal KP = 40.0;
    const dReal KD = 5.0;

    dJointSetHinge2Param(ph.joints[i], dParamSuspensionERP, 0.2 * KP / (0.2 * KP + KD));
    dJointSetHinge2Param(ph.joints[i], dParamSuspensionCFM, 1 / (0.2 * KP + KD));
  }

  //prismatic & rotoid
  //  dJointID c = dJointCreatePR( World::getSingletonPtr()->getWorld() ,0); wtf?
  // dJointAttach(c, w[0].getBody(), w[1].getBody());

  //forbid all Y rotation for j[3] j[4]
  for (int i = 0; i < 4; i++) {
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

void Car::reset() {
  dGeomSetPosition (ph.geom, Conf::Car::POS[0], Conf::Car::POS[1], Conf::Car::POS[2]);
  dMatrix3 R = {
    1.0, 2.0, 12.0, 
    3.0, 1.0, 6.0,
    4.0, 5.0, 1.0
  };
  
  dBodySetLinearVel(ph.body, 0., 0., 0.);
  dBodySetAngularVel(ph.body, 0., 0., 0.);

  dGeomSetRotation(ph.geom, R);
  rotateWheels(&R);
  dGeomSetRotation(ph.leftDoor.geom, R);
  dGeomSetRotation(ph.rightDoor.geom, R);
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
      dJointSetHinge2Param (ph.joints[i],dParamFMax,1.2); //
      dJointSetHinge2Param (ph.joints[i],dParamLoStop, -0.65);
      dJointSetHinge2Param (ph.joints[i],dParamHiStop, 0.65);
      dJointSetHinge2Param (ph.joints[i],dParamFudgeFactor,1.0);
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
    dJointSetHinge2Param(ph.joints[i], dParamVel2, sp);
    dJointSetHinge2Param(ph.joints[i], dParamFMax2, 4.0 * 10.0);    
  }

  /*if (sp < palier1) sp+=speed;
  else if(sp>max) sp=max;


 for (int i = 2; i < 4; i++) {
    dJointSetHinge2Param(ph.joints[i], dParamVel2, sp);

    if(sp<0)
      dJointSetHinge2Param(ph.joints[i], dParamFMax2, 1.5*10.5);    
    else if(sp>100*speed)
      dJointSetHinge2Param(ph.joints[i], dParamFMax2, 1.0*10.5);
    else if(sp>50*speed)
      dJointSetHinge2Param(ph.joints[i], dParamFMax2, 1.5*10.5);
    else
      dJointSetHinge2Param(ph.joints[i], dParamFMax2, 5.5*10.5);
      }*/
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



void Car::createLeftDoor() {
  Ogre::SceneNode *lnode = sceneMgr_->getRootSceneNode()->createChildSceneNode("left_door");
    
  cst.leftDoorNode = lnode;

  lnode->scale(0.35, 0.35, 0.35);

  createAndAttachEntity("left_door", "left_door.mesh", "Ford/LeftDoor", lnode);
  createAndAttachEntity("left_window", "left_window.mesh", "Ford/LeftWindow", lnode);
  createAndAttachEntity("left_little_window", "left_little_window.mesh", "Ford/LeftWindow", lnode);
}

void Car::createRightDoor() {
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

  createLeftDoor();
  createRightDoor();

}


void Car::createCamNodes() {
  Ogre::SceneNode *cam = cst.subCarNode->createChildSceneNode("cam_pos");
  //MUST BE CHANGE
  cam->translate(0.0, 9.0, -15.0);
  
  Ogre::SceneNode *camT = cst.subCarNode->createChildSceneNode("cam_target");
  //MUST BE CHANGE
  camT->translate(0.0, 4.0, 5.0);

  //don't what it used for
  cam->setAutoTracking (true, camT);
  cam->setFixedYawAxis (true); 
}

