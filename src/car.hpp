#ifndef CAR_HPP
#define CAR_HPP

//#include "global.hpp"
#include "OgreFramework.hpp"
#include "space.hpp"
#include "wheel.hpp"
#include "type.hpp"
#include "conf.hpp"
#include "ground.hpp"
#include "obstacle.hpp"

#include <ode/ode.h>


class Car: public Space {
public:
  //should remain constant over the car object life
  struct Cst {
    std::string nodeName;
    Ogre::SceneNode *carNode;
    Ogre::SceneNode *subCarNode;
    float brakeForce;
    float gasForce;
    float steeringForce;
    float lowRiderForce;
  };

  struct Ph {
    dJointID joints[4];
    dBodyID body;
    dGeomID geom;
    dMass mass;
  };

  Car();
  ~Car();
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

  void lowRideFront();
  void lowRideBack();

  Ogre::Vector3 cam();
  Ogre::Vector3 getPosition();
  Ogre::Vector3 getDirection();
  Ogre::Quaternion getOrientation();

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

  void initXml(const char *xmlFile, Ogre::SceneNode *root);

  Wheel wheels[4];
  struct Ph ph;
  struct Cst cst;
  
  DContactType type;
  DContactType spaceType;

private:
  bool brake;
  float speed,steer;


  void createNodesAndMeshes(Utils::Xml &x);

  void updateMotor();
  void updateSteering();

  void rotateWheels(dMatrix3 *R);
  void printRotationMatrix();

  void createAndAttachEntity(const std::string &name, const std::string &meshName, const std::string &MaterialName, Ogre::SceneNode *node) const ;
  void createSpace();
  
  void createCamNodes(Utils::Xml &x);
  void createPhysics(Utils::Xml &x);
  void createJoints(Utils::Xml &x);
  void disposePhysics(Utils::Xml &x);
  void disposeGeoms(Utils::Xml &x);
  void disposeJoints(Utils::Xml &x);

  void setupContacs(Utils::Xml &x);
};
#endif
