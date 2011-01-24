#include <assert.h>

#include "type.hpp"
#include "global.hpp"
#include "world.hpp"

#define drawContactPoints_(BOOL)					\
  if (BOOL) {								\
  static int n = _glb.nbTurn;						\
  static std::vector<Ogre::Entity*> ents;				\
  if (n < _glb.nbTurn) {						\
  n = _glb.nbTurn;							\
  for (int i = ents.size()-1; i >= 0; i--) {				\
      sceneMgr_->destroySceneNode(ents[i]->getParentSceneNode());	\
      sceneMgr_->destroyEntity(ents[i]);				\
      ents.pop_back();							\
  }									\
  }									\
  Ogre::SceneNode* node;						\
  try{ node = sceneMgr_->getSceneNode("contact_node"); }		\
  catch (Ogre::Exception e) {						\
    node=sceneMgr_->getRootSceneNode()					\
      ->createChildSceneNode("contact_node");				\
  }									\
  for(int i=0; i < numc; i++) {						\
  Ogre::Entity* e = sceneMgr_->createEntity("collision_point.mesh");	\
  Ogre::SceneNode* n=node->createChildSceneNode(			\
    Ogre::Vector3( (Ogre::Real)contact[i].geom.pos[0],			\
		   (Ogre::Real)contact[i].geom.pos[1],			\
		   (Ogre::Real)contact[i].geom.pos[2]) );		\
  ents.push_back(e);							\
  n->attachObject(e);							\
  n->scale(0.35, 0.35, 0.35);						\
  }    									\
  }									\

Type::Type(TypeList typeList) : type(typeList) {}

bool Type::isPrecedence(dGeomID geom) const {
  return true;
}

const dContact* DContactProperty::getContact() const {
  return &contact;
}

DContactProperty::DContactProperty(Type::TypeList typeList) : Type(typeList) {}

void DContactProperty::dealWith(dGeomID me, dGeomID other) {
  assert(dGeomIsSpace(me) != true);
  assert(dGeomIsSpace(other) != true);
  
  DContactProperty *cProperty = (DContactProperty*)dGeomGetData(other);

  const int  MAX_CONTACT_POINTS = 16;
  dContact contact[MAX_CONTACT_POINTS];
  mixdContact(contact, getContact(), cProperty->getContact());
  for(int i=1; i<MAX_CONTACT_POINTS; i++)
    memcpy(contact+i, contact, sizeof(*contact));

  if (int numc = dCollide (me, other, MAX_CONTACT_POINTS,
			   &contact[0].geom,sizeof(dContact))) {

    drawContactPoints_(true);
  
    for(int i=0; i < numc; i++) {  
      dJointID c = dJointCreateContact (World::getSingletonPtr()->world, 
					World::getSingletonPtr()->contactGroup,
					contact+i);
      dJointAttach (c, dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}


void DContactProperty::mixdContact(dContact* res, const dContact* c1, const dContact* c2) {
  int m1 = c1->surface.mode, m2 = c2->surface.mode ;

  memset(&(res->surface.mode),0,sizeof(dSurfaceParameters));

  //set the mix
  if (m1 & dContactBounce) {
    res->surface.mode |= dContactBounce;
      res->surface.bounce += c1->surface.bounce;  
  }
  
  if (m2 & dContactBounce) {
    res->surface.mode |= dContactBounce;
    res->surface.bounce += c2->surface.bounce;
  }

  if (m1 & dContactSoftCFM) {
    res->surface.mode |= dContactSoftCFM;
    res->surface.soft_cfm += c1->surface.soft_cfm;
  }

  if (m2 & dContactSoftCFM) {
    res->surface.mode |= dContactSoftCFM;
    res->surface.soft_cfm += c2->surface.soft_cfm;
  }

  if (m1 & dContactSoftERP) {
    res->surface.mode |= dContactSoftERP;
    res->surface.soft_erp += c1->surface.soft_erp;
  }
  
  if (m2 & dContactSoftERP) {
    res->surface.mode |= dContactSoftERP;
    res->surface.soft_erp += c2->surface.soft_erp;
  }

  if (m1 & dContactSlip1) {
    res->surface.mode |= dContactSlip1;
    res->surface.slip1 += c1->surface.slip1;
    
  }
  
  if (m2 & dContactSlip1) {
    res->surface.mode |= dContactSlip1;
    res->surface.slip1 += c2->surface.slip1;
  }

  if (m1 & dContactSlip2) {
    res->surface.mode |= dContactSlip2;
    res->surface.slip2 += c1->surface.slip2;
  }

  if (m2 & dContactSlip2) {
    res->surface.mode |= dContactSlip2;
    res->surface.slip2 += c2->surface.slip2;
  }

  if (m1 & dContactMu2) {
    res->surface.mode |= dContactMu2;
    if (!res->surface.mu2 == dInfinity)//crappy
      res->surface.mu2 += c1->surface.mu2;
  }

  if (m2 & dContactMu2) {
    res->surface.mode |= dContactMu2;
    if (!res->surface.mu2 == dInfinity)
      res->surface.mu2 += c2->surface.mu2;
  }

  if (c2->surface.mu == dInfinity)
    res->surface.mu = dInfinity;
  else if (c1->surface.mu == dInfinity)
    res->surface.mu = dInfinity;
  else
    res->surface.mu = c2->surface.mu + c1->surface.mu ;

  res->surface.bounce_vel = 
    (c1->surface.bounce_vel < c2->surface.bounce_vel) ? c1->surface.bounce_vel : c2->surface.bounce_vel;

  //take care of possibles results
  if (res->surface.mu <0) res->surface.mu = 0;
  if (res->surface.bounce_vel < 0) res->surface.bounce_vel = 0;

  if (res->surface.slip1<0) res->surface.slip1 = 0;

  if (res->surface.slip2<0) res->surface.slip2 = 0;
  
}

