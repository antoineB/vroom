#ifndef GEOM_HPP
#define GEOM_HPP

#include <ode/ode.h>

class Geom{
protected:
  dGeomID geom;
  
  Geom(const Geom&);
  Geom & operator=(const Geom&);
  
public:
  Geom():geom(NULL){}
  dGeomID getGeom();
  virtual void init(void* ptr)=0;
  virtual void update()=0;
};

#endif
