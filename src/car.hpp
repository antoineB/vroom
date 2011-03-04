#ifndef CAR_HPP
#define CAR_HPP

#include <ode/ode.h>

//#include "global.hpp"
#include "OgreFramework.hpp"
#include "parsexml.hpp"
#include "space.hpp"
#include "type.hpp"

//Car
#define C_X 0
#define C_Y 4
#define C_Z 0

//Wheel
#define W_RADIUS 1.20
#define W_DENSITY 1

#define W_FR_X 3.05
#define W_FR_Y +0.0
#define W_FR_Z -5.85

#define W_FL_X -3.05
#define W_FL_Y +0.0
#define W_FL_Z -5.85

#define W_BR_X 3.15
#define W_BR_Y -0.2
#define W_BR_Z 5.0

#define W_BL_X -3.15
#define W_BL_Y -0.2
#define W_BL_Z 5.0


class Wheel{
  
public:
  struct Position{
    dReal x;
    dReal y;
    dReal z;
  };
  
  ~Wheel();

  void create(dSpaceID s, Wheel::Position p, Ogre::SceneNode *node);
  void update();

  Wheel();  

  void parseXml(TiXmlHandle handle, unsigned int number);
  
  inline dBodyID getBody();

  static DContactType type;

  dGeomID getGeom(){
    return g;
  }


private:

  void getPositionFromCar(dReal *d){
    d[0] = C_X + pos.x;
    d[1] = C_Y + pos.y;
    d[2] = C_Z + pos.z;
  }


  struct Position pos;
  dBodyID b;
  dGeomID g;
  dMass m;
  std::string name;

  void init(dSpaceID s);
};


class Car: public Space{

public:
  Car();
  Car(std::string fileName);
  ~Car();
  void init(const char *n, Ogre::SceneNode *no);
  void update();

  void dropDoors();

  dReal getPunch();
 
  void accelerate();
  void slowDown();
  void setSpeed(float s);
  void turnRight();
  void turnLeft();
  void setSteer(float s);

  void reset();
  void lowRideFront();
  void lowRideBack();

  Ogre::Vector3 cam();
  Ogre::Vector3 getPosition();
  Ogre::Vector3 getDirection();
  Ogre::Quaternion getOrientation();


  void parseXml(std::string fileName);
  void parseXml(TiXmlHandle handle);

  bool isWheel(dGeomID gg){
    if(gg==w[0].getGeom())      return true;
    else if(gg==w[1].getGeom()) return true;
    else if(gg==w[2].getGeom()) return true;
    else if(gg==w[3].getGeom()) return true;
    return false;
  };

  void setBrake(bool b);

  void swayBars();

  dReal getFrontWheelsErp();
  dReal getBackWheelsErp();
  void setBackWheelsErp(dReal erp);
  void setFrontWheelsErp(dReal erp);
  dReal getFrontWheelsCfm();
  dReal getBackWheelsCfm();
  void setBackWheelsCfm(dReal cfm);
  void setFrontWheelsCfm(dReal cfm);


private:
  Wheel w[4];

  dBodyID b;
  dGeomID g;
  dMass m;
  static DContactType type;

  bool brake;

  dGeomID leftDoorGeom;
  dBodyID leftDoorBody;
  dJointID leftDoorJoint;
  dMass leftDoorMass;

  dGeomID rightDoorGeom;
  dBodyID rightDoorBody;
  dJointID rightDoorJoint;
  dMass rightDoorMass;


  dJointID j[4];
  
  float speed,steer;
  void updateMotor();
  void updateSteering();
  

  void setMesh(std::string name);
  void setMaterial(std::string name);

  void rotateWheels(dMatrix3 *R);
  void printRotationMatrix();

  void createAndAttachEntity(const std::string &name, const std::string &meshName, const std::string &MaterialName, Ogre::SceneNode *node) const ;
  void createLeftDoor();
  void createRightDoor();
  void createNodesAndMeshes(std::string nodeName, Ogre::SceneNode *parentNode);
  void createCamNodes();



  //should remain constant over the car object life
  struct Cst 
  {
    std::string nodeName;
    Ogre::SceneNode *carNode;
    Ogre::SceneNode *subCarNode;
    Ogre::SceneNode *leftDoorNode;
    Ogre::SceneNode *rightDoorNode;
  };

  struct Cst cst;

};
#endif
