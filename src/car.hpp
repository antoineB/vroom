#ifndef CAR_HPP
#define CAR_HPP

#include <ode/ode.h>

//#include "global.hpp"
#include "OgreFramework.hpp"
#include "space.hpp"
#include "wheel.hpp"
#include "type.hpp"
#include "conf.hpp"
#include "ground.hpp"
#include "obstacle.hpp"

class Car: public Space{

public:
  Car();
  Car(const char *fileName);
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

  const dReal* getSpeed();

  void setMass(dReal total, dReal x, dReal y, dReal z);

  void reset(Conf::Param &mod);
  void lowRideFront();
  void lowRideBack();

  Ogre::Vector3 cam();
  Ogre::Vector3 getPosition();
  Ogre::Vector3 getDirection();
  Ogre::Quaternion getOrientation();


  void parseXml(std::string fileName);
  void parseXml(TiXmlHandle handle);

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

  struct Ph {
    Wheel wheels[4];
    dJointID joints[4];
    dBodyID body;
    dGeomID geom;
    dMass mass;
  };

  struct Ph ph;

  static DContactType type;

private:

  bool brake;


  float speed,steer;
  void updateMotor();
  void updateSteering();
  

  void setMesh(std::string name);
  void setMaterial(std::string name);

  void rotateWheels(dMatrix3 *R);
  void printRotationMatrix();

  void createAndAttachEntity(const std::string &name, const std::string &meshName, const std::string &MaterialName, Ogre::SceneNode *node) const ;
  void createLeftDoorGraphic();
  void createRightDoorGraphic();
  void createNodesAndMeshes(std::string nodeName, Ogre::SceneNode *parentNode);
  void createCamNodes();

  void createSpace();
  void createPhysics();
  void createJoints();
  void disposePhysics();
  void disposeGeoms();
  void disposeJoints();

  void createPhysics(Conf::Car::Param &mod);
  void disposePhysics(Conf::Car::Param &mod);
  void disposeGeoms(Conf::Car::Param &mod);
  void disposeJoints(Conf::Car::Param &mod);
  void createJoints(Conf::Car::Param &mod);

  void createLeftDoorPhysic();
  void createRightDoorPhysic();

  static void fillContact();
  static void fillContact(Conf::Car::Param &mod);

  //should remain constant over the car object life
  struct Cst {
    std::string nodeName;
    Ogre::SceneNode *carNode;
    Ogre::SceneNode *subCarNode;
    Ogre::SceneNode *leftDoorNode;
    Ogre::SceneNode *rightDoorNode;
  };

  struct Cst cst;

};
#endif
