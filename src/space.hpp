#ifndef SPACE_HPP
#define SPACE_HPP

#include <ode/ode.h>

#include "global.hpp"

class Space{
  Space(const Space&);
  Space & operator=(const Space&);

protected:
  dSpaceID space;
  
public:
  Space(){}  
  ~Space();
  dGeomID addBox(dReal lx, dReal ly, dReal lz);
  dGeomID addSphere(dReal radius);
  dGeomID addCylinder(dReal radius, dReal length);
  dGeomID addPlane(dReal a, dReal b, dReal c, dReal d);
  dGeomID addTriMesh(dTriMeshDataID data/*, dTriCallback *Callback=NULL,
		     dTriArrayCallback *ArrayCallback=NULL,
		     dTriRayCallback *RayCallback=NULL*/);

  dGeomID add(dGeomID g);

  dSpaceID addSimpleSpace();
  dSpaceID addHashSpace();
  dSpaceID getSpace();

  Space(dSpaceID s){
    space =s;
  }
};

#endif
/*
custom cpp

*/
