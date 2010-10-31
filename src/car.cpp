#include "car.hpp"
#include "mytools.hpp"
#include <sstream>
#include "world.hpp"

Wheel::Wheel(){}

dContact Wheel::contact;
dContact Car::contact;

void Wheel::init(dSpaceID s){
  g = dCreateSphere(0, W_RADIUS);
  Wheel::contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  Wheel::contact.surface.mu = dInfinity;
  Wheel::contact.surface.bounce = 1.0;
  Wheel::contact.surface.bounce_vel = 0.1;
  Wheel::contact.surface.soft_cfm = 0.01;  
  Wheel::contact.surface.soft_erp = 0.3;  
  Wheel::contact.surface.slip1 = 1.5;
  Wheel::contact.surface.slip2 = 1.5;

  dGeomSetData(g,(void*)&Wheel::contact);

  dContact * c2 = (dContact*) dGeomGetData(g);
  if(c2==NULL)
    std::cout<<"wheel est null"<<std::endl;

  dSpaceAdd(s,g);
  dMassSetSphere (&m, W_DENSITY, W_RADIUS);
  b=World::getSingletonPtr()->add(g,&m);
}

void Wheel::update(){
  Ogre::SceneNode *n=_sceneMgr->getSceneNode(name.c_str());
  MyTools::byOdeToOgre(b, n );
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
  e = _sceneMgr->createEntity((name+"_tire").c_str(), "wheel_tire.mesh");
  e->setMaterialName("Wheels/Tire");
  e->setCastShadows(true);
  n->attachObject(e);
  e = _sceneMgr->createEntity((name+"_rim").c_str(), "wheel_rim.mesh");
  e->setMaterialName("Wheels/Rim");
  n->attachObject(e);
  //n->scale();
  //  n->yaw(Ogre::Degree(90));

  dReal d[3];
  getPositionFromCar(d);
  dBodySetPosition(b, d[0], d[1], d[2]);
  dMatrix3 R;
  const dReal PI=3.14159265;
  //ugly
  if(nb%2==1){
    //dRFromEulerAngles( R, 0.0, 0.0, -PI/2);
    dRFromAxisAndAngle(R, 0.0, 0.0, 1.0, -PI/2);//beware of the sgn of PI/2
    dBodySetRotation(b,R);
  }else {
    dRFromAxisAndAngle(R, 0.0, 0.0, 1.0, PI/2);
    dBodySetRotation(b,R);
  }
  MyTools::byOdeToOgre(b, n);

  //ugly
  if(nb>2)
    n->scale(2.45, 2.9, 2.45);
  else
    n->scale(2.45, 2.4, 2.45);
}

void Car::update(){

  //graphical
  MyTools::byOdeToOgre(g, _sceneMgr->getSceneNode(nodeName.c_str() ));
  
  for(int i=0; i<4; i++)
    w[i].update();

  //physical
  updateMotor();
  updateSteering();
}

Car::~Car(){}
Car::Car(): speed(0.0), steer(0.0), g(NULL), b(NULL) ,brake(false){}

/*Car::Car(std::string fileName){
  parseXml(fileName);
  } */ 


void Car::init(const char *n, Ogre::SceneNode *no){
  //si ogre n'est pas lancer...
  if(_sceneMgr==NULL){
    _log("Ogre isn't lunched before the car is set up");
    exit(0);
  }
  
  //si ode n'est pas lancer ...
  if(_glb.worldUp==false){
    _log("Ode isn't lunched before the car is set up");
    exit(0);
  }
  
  nodeName=n;
  
  Ogre::SceneNode *node;
  Ogre::Entity *e;  
  //  Ogre::Entity *b_car;    

  e = _sceneMgr->createEntity("subframe","subframe.mesh");
  e->setMaterialName("Car/Subframe");
  //  b_car = _sceneMgr->createEntity("body","body.mesh");
  //  b_car->setCastShadows(true);
  //  b_car->setMaterialName("Car/Body");
  node = no->createChildSceneNode(n);
  node->attachObject(e);
  //  node->attachObject(b_car);

  //partie physique
  
  space=World::getSingletonPtr()->addSimpleSpace();
  dSpaceSetCleanup(space,0); //pour eviter de detruire les geom quand le space est detruit
  //mais quand est t'il detruit?

  const dReal x=7.48 , y=0.72 , z=17.56 ;
  g= addBox ( x, y, z);
  Car::contact.surface.mode= dContactBounce | dContactSoftCFM
    | dContactSoftERP | dContactSlip1 | dContactSlip2;
  Car::contact.surface.mu = dInfinity;
  Car::contact.surface.bounce = 0.01;
  Car::contact.surface.bounce_vel = 0.7;
  Car::contact.surface.soft_cfm = 0.01;  
  Car::contact.surface.soft_erp = 0.3;  
  Car::contact.surface.slip1 = 0.07;
  Car::contact.surface.slip2 = 0.07;
  dGeomSetData(g,(void*)&Car::contact);

  dContact * c2 = (dContact*) dGeomGetData(g);
  if(c2==NULL)
    std::cout<<"car est null"<<std::endl;

  dGeomSetData((dGeomID)space,(void*)&Car::contact);

  c2 = (dContact*) dGeomGetData((dGeomID)space);
  if(c2==NULL)
    std::cout<<"space est null"<<std::endl;


  dMassSetBox(&m, 1.0, x, 2*y, z);
  b=World::getSingletonPtr()->add(g,&m);
  dGeomSetPosition (g, C_X, C_Y, C_Z);
  
  MyTools::byOdeToOgre(g, node);
  
  {
    Ogre::SceneNode* carNode=_sceneMgr->getRootSceneNode();
    struct Wheel::Position tmp[4]={
      {W_FR_X, W_FR_Y, W_FR_Z},
      {W_FL_X, W_FL_Y, W_FL_Z},
      {W_BR_X, W_BR_Y, W_BR_Z},
      {W_BL_X, W_BL_Y, W_BL_Z} };
    
    for(int i=0; i<4; i++)
      w[i].create(space, tmp[i], carNode);
  }
 
  //partie jointure
  for(int i=0;i<4; i++){
    j[i]=World::getSingletonPtr()->addHinge2(b,w[i].getBody(),0);
    const dReal *a=dBodyGetPosition(w[i].getBody());
    dJointSetHinge2Anchor(j[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1(j[i],0,1,0);
    dJointSetHinge2Axis2(j[i],1,0,0);
    
    dJointSetHinge2Param(j[i], dParamSuspensionERP, 0.8);
    dJointSetHinge2Param(j[i], dParamSuspensionCFM, 0.05);

  }

  //prismatic & rotoid
  //  dJointID c = dJointCreatePR( World::getSingletonPtr()->getWorld() ,0); wtf?
  // dJointAttach(c, w[0].getBody(), w[1].getBody());

  //empécher toute rotation selon y pour j[3] j[4]
  for (int i=0; i<4; i++) {
    dJointSetHinge2Param (j[i],dParamLoStop,0);
    dJointSetHinge2Param (j[i],dParamHiStop,0);

    if(i>1){
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

void Car::accelerate(){ speed+=10.5; }

void Car::slowDown(){ speed-=20.5; }

void Car::setSpeed(float s){ speed=s; }

void Car::turnRight(){ steer+=0.08; }

void Car::turnLeft(){ steer-=0.08; }

void Car::setSteer(float s){ steer=s; }

void Car::updateSteering(){
  static float st=0.0;
  if(steer==0){
    st=0;

    for(int i=0; i<2; i++){
      dJointSetHinge2Param (j[i],dParamLoStop,0);
      dJointSetHinge2Param (j[i],dParamHiStop,0);
    }

  }
  else{
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

void Car::setBrake(bool b){
  brake = b;
}

void Car::updateMotor(){
  const float max=1000.0;
  const float palier1=30.0;

  if(brake){
    for(int i = 2; i<4; i++){
      dJointSetHinge2Param(j[i], dParamVel2, 0);
      dJointSetHinge2Param(j[i], dParamFMax2, 10.5);        
    }
    return ;
  }

  static float sp=0.0;
  if(speed==0){
    sp=0;
    for(int i=2;i<4;i++)
    dJointSetHinge2Param(j[i], dParamFMax2, 0.01);
    return ;
  }
  else{
    float y= speed <0 ? -10.0 : 11.0;
    dBodyAddRelForceAtRelPos(b, 0.0, y, 0.0, 
			     0.0, 0.0, W_FR_Z ); //permet d'ajouter un effet decolage de la voiture
    sp+=speed;
  }

  if(sp<palier1) sp+=speed;
  else if(sp>max) sp=max;


 for(int i=2;i<4;i++){
    dJointSetHinge2Param(j[i], dParamVel2, sp);

    if(sp<0)
      dJointSetHinge2Param(j[i], dParamFMax2, 10.5);    
    else if(sp>100*speed)
      dJointSetHinge2Param(j[i], dParamFMax2, 1*10.5);
    else if(sp>50*speed)
      dJointSetHinge2Param(j[i], dParamFMax2, 1.5*10.5);
    else
      dJointSetHinge2Param(j[i], dParamFMax2, 2*10.5);
  }
}

Ogre::Vector3 Car::cam(){
    const dReal *pos = dBodyGetPosition (b);
    return Ogre::Vector3((Ogre::Real)pos[0],
			 (Ogre::Real)pos[1]+1.5, 
			 (Ogre::Real)pos[2]);
}

/*void Car::parseXml(std::string fileName){
  TiXmlDocument doc(fileName);
  if(!doc.LoadFile()){
    ParseXml::logAndDie("probleme dans la lecture de "+fileName);
  }
  TiXmlHandle docH( &doc );
  this->parseXml(docH);
}

void Car::parseXml(TiXmlHandle handle){ 
  handle=handle.FirstChild("car");
  if(!handle.ToElement())
    ParseXml::logAndDie("pas delemenr car");

  ParseXml::parseXml(handle,&m);
  //  ParseXml::parseXml(handle,&g,(Space)(*this));
  ParseXml::parseXml(handle,&b,&g,&m);

  {
    std::string s;
    if(!ParseXml::getValue(handle,&s,"name"))
      ParseXml::logAndDie("pas de balise name dans Car");
    nodeName=s;

    if(!ParseXml::getValue(handle,&s,"mesh"))
      ParseXml::logAndDie("pas de balise mesh dans Car");
    setMesh(s);

    if(ParseXml::getValue(handle,&s,"material"))
      setMaterial(s);
    
  }

  for(int i=0; i<4; i++)
    w[i].parseXml(handle,i);

  for(int i=0; i<4; i++)
    parseXml(handle, j[i], i,
	     b, w[i].getBody() );
  
}

void Wheel::parseXml(TiXmlHandle handle, unsigned int number){
  handle=handle.Child("wheel",number);
  if(!handle.ToElement())
    ParseXml::logAndDie("pas delemenr wheel");
  
  {
    std::string s;
    if(!ParseXml::parseXml(handle,&s,"name"))
      ParseXml::logAndDie("pas de balise name dans Car");
    name=s;

    if(!ParseXml::parseXml(handle,&s,"mesh"))
      ParseXml::logAndDie("pas de balise mesh dans Car");
    setMesh(s);

    if(ParseXml::parseXml(handle,&s,"material"))
      setMaterial(s);
    
  }
  
  ParseXml::parseXml(handle,&g);
  ParseXml::parseXml(handle,&b);
  ParseXml::parseXml(handle,&m);

}

void Car::setMesh(std::string name){
  Ogre::Entity* e=_sceneMgr->createEntity(nameNode);
  Ogre::SceneNode* n=_sceneMgr->getRootSceneNode()->createChild(nameNode);
  n->attachObject(e);
}

void Car::setMaterial(std::string name){
  Ogre::Entity* e=_sceneMgr->getEntity(nameNode);
  e->setMaterialName(name);
}
*/
