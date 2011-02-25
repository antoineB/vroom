#ifndef GROUD_HPP
#define GROUD_HPP

#include <ode/ode.h>
#include <OGRE/OgreSingleton.h>

#include "world.hpp"

#include "global.hpp"
#include "type.hpp"

class Ground: public Ogre::Singleton<Ground>{
protected:
  static const std::string name;
  dGeomID plane;
  
  Ground(const Ground&);
  Ground & operator=(const Ground&);
  
  virtual void init(const char* material)=0;

public:
  Ground(){}
  dGeomID getPlane();
};

class FlatGround: public Ground{
  FlatGround();
  FlatGround(const FlatGround&);
  FlatGround & operator=(const FlatGround&);

  virtual void init(const char* material);

  static DContactType type;

public:
  FlatGround(const char* material);

};

#endif
