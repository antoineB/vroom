#include "space.hpp"

Space::~Space(){ if(_glb.worldUp) dSpaceDestroy(space); }

dGeomID Space::add(dGeomID g){
    dSpaceAdd(space,g);
    return g;
}

dSpaceID Space::addSimpleSpace(){
  return dSimpleSpaceCreate(space);
}

dSpaceID Space::addHashSpace(){
  return dHashSpaceCreate(space);
}

dGeomID Space::addBox(dReal lx, dReal ly, dReal lz){
  return dCreateBox(space,  lx,  ly,  lz);
}

dGeomID Space::addCylinder(dReal radius, dReal length){
  return dCreateCylinder(space, radius, length);
}

dGeomID Space::addSphere(dReal radius){
  return dCreateSphere(space, radius);
}

dGeomID Space::addPlane(dReal a, dReal b, dReal c, dReal d){
  return dCreatePlane (space,  a,  b,  c,  d);
}

dGeomID Space::addTriMesh(dTriMeshDataID data/*,dTriCallback *Callback=NULL,
		   dTriArrayCallback *ArrayCallback=NULL,
		   dTriRayCallback *RayCallback=NULL*/){
  //  return dCreateTriMesh (space, data, Callback, ArrayCallback, RayCallback);
  return dCreateTriMesh (space, data, NULL, NULL, NULL);
}

dSpaceID Space::getSpace(){
  return space;
}
