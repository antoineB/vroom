#ifndef WORLD_HPP
#define WORLD_HPP

#include "utils.hpp"

#include "global.hpp"
#include "mytools.hpp"

#include <ode/ode.h>
#include <OGRE/OgreSingleton.h>

#include "space.hpp"

void nearCallback (void *data, dGeomID o1, dGeomID o2);  

class World : public Space, public Ogre::Singleton<World> {
  
  struct Constant{
    dReal simulationPace;
    dReal gravity[3];
    dReal cfm;
    dReal erp;
  };

  struct Constant cst;

  dWorldID world;
  dJointGroupID contactGroup;

  World();
  World(const World&);
  World & operator=(const World&);

  void parseXml(std::string fileName);

  friend void nearCallback (void *data, dGeomID o1, dGeomID o2);  

public:
  ~World();
  World(std::string xmlFileName);
  dBodyID add( dGeomID g, dMass* m);
  void update();
  dJointID addHinge2(dBodyID b1, dBodyID b2, dJointGroupID jg=0);
  dJointID addHinge(dBodyID b1, dBodyID b2, dJointGroupID jg=0);

  dWorldID getWorld() const;
  dJointGroupID getContactGroup() const;

  static dReal atodr(const char *str);

  void beforeCollidingSpaces();
  void afterCollidingSpaces();
  void beforePhysicalStep();
  void afterPhysicalStep();

};

#endif
