#ifndef TYPE_HPP
#define TYPE_HPP

#include <ode/ode.h>

#include <string>

/**
 * This class is used to be associated with a geom. So during the the
 * collision the geom type could be easily retrieved.
 */
class Type {
private:
  Type();
  Type(const Type&);
  Type & operator=(const Type&);

public:
  enum TypeList{
    NONE = 0,

    CAR,
      CAR_WHEEL,
	CAR_WHEEL_FRONT,
	  CAR_WHEEL_RIGHT_FRONT,
	  CAR_WHEEL_LEFT_FRONT,
	CAR_WHEEL_BACK,
	  CAR_WHEEL_RIGHT_BACK,
	  CAR_WHEEL_LEFT_BACK,
      CAR_SPACE,
      CAR_SUBFRAME,

    GLOBAL_SPACE,
    GROUND_SPACE,
    GROUND,
    OBSTACLE_SPACE,
    OBSTACLE,
    MOVABLE_OBSTACLE, //could resist to a certain amount of strike before being a MOVABLE_ELEMENT
    MOVABLE_ELEMENT,
    MOVABLE_ELEMENT_SPACE,

    UNDEFINED,
  };
 
  static TypeList stringToType(std::string & s) {
    if (s == "NONE")
      return NONE;
    if (s == "CAR")
      return CAR;
    if (s == "CAR_WHEEL")
      return CAR_WHEEL;
    if (s == "CAR_WHEEL_FRONT")
      return CAR_WHEEL_FRONT;
    if (s == "CAR_WHEEL_RIGHT_FRONT")
      return CAR_WHEEL_RIGHT_FRONT;
    if (s == "CAR_WHEEL_LEFT_FRONT")
      return CAR_WHEEL_LEFT_FRONT;
    if (s == "CAR_WHEEL_BACK")
      return CAR_WHEEL_BACK;
    if (s == "CAR_WHEEL_RIGHT_BACK")
      return CAR_WHEEL_RIGHT_BACK;
    if (s == "CAR_WHEEL_LEFT_BACK")
      return CAR_WHEEL_LEFT_BACK;
    if (s == "CAR_SPACE")
      return CAR_SPACE;
    if (s == "CAR_SUBFRAME")
      return CAR_SUBFRAME;
    
    if (s == "GLOBAL_SPACE")
      return GLOBAL_SPACE;
    if (s == "GROUND_SPACE")
      return GROUND_SPACE;
    if (s == "GROUND")
      return GROUND;
    if (s == "OBSTACLE_SPACE")
      return OBSTACLE_SPACE;
    if (s == "OBSTACLE")
      return OBSTACLE;
    if (s == "MOVABLE_OBSTACLE")
      return MOVABLE_OBSTACLE;
    if (s == "MOVABLE_ELEMENT")
      return MOVABLE_ELEMENT;
    if (s == "MOVABLE_ELEMENT_SPA")
      return MOVABLE_ELEMENT_SPACE;

    return UNDEFINED;
  }

  TypeList type;

  /**
   * Determine witch of the two Geoms will used its dealWith method on
   * the other.
   */
  bool isPrecedence(dGeomID geom) const;

  virtual  void dealWith(dGeomID me, dGeomID other) = 0;

  Type(TypeList type);
};



class DContactType : public Type {
private:
  DContactType();
  DContactType(const DContactType&);
  DContactType & operator=(const DContactType&);
  
public:
  dContact contact;

  static void mixdContact(dContact* res, const dContact* c1, const dContact* c2);

  DContactType(Type::TypeList type);
  virtual  void dealWith(dGeomID me, dGeomID other);
};

#endif
