#ifndef CAR_HPP
#define CAR_HPP

#include <ode/ode.h>

//#include "global.hpp"
#include "OgreFramework.hpp"
#include "parsexml.hpp"
#include "space.hpp"

//Car
#define C_X 0
#define C_Y 10
#define C_Z 0

//Wheel
#define W_RADIUS 1.20
#define W_DENSITY 1

#define W_FR_X 3.05
#define W_FR_Y 0
#define W_FR_Z -6.5

#define W_FL_X -3.05
#define W_FL_Y 0
#define W_FL_Z -6.5

#define W_BR_X 3.45
#define W_BR_Y 0
#define W_BR_Z 6.30

#define W_BL_X -3.45
#define W_BL_Y 0
#define W_BL_Z 6.30


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

  static dContact contact;

  dGeomID getGeom(){
    return g;
  }

private:

  void getPositionFromCar(dReal *d){
    //    dReal d[3];
    d[0] = C_X + pos.x;
    d[1] = C_Y + pos.y;
    d[2] = C_Z + pos.z;
    //    return d;
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
 
  void accelerate();
  void slowDown();
  void setSpeed(float s);
  void turnRight();
  void turnLeft();
  void setSteer(float s);

  Ogre::Vector3 cam();

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

private:
  Wheel w[4];

  dBodyID b;
  dGeomID g;
  dMass m;
  static dContact contact;

  bool brake;

  dJointID j[4];
  
  std::string nodeName;

  float speed,steer;
  void updateMotor();
  void updateSteering();
  

  void setMesh(std::string name);
  void setMaterial(std::string name);


};
#endif
