#ifndef GROUD_HPP
#define GROUD_HPP

#include <ode/ode.h>
#include <OgreSingleton.h>

#include "world.hpp"

#include "global.hpp"

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

  static dContact contact;

public:
  FlatGround(const char* material);

};

#endif
