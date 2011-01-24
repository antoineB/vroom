#ifndef TYPE_HPP
#define TYPE_HPP

#include <ode/ode.h>

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

    CAR_WHEEL,
      CAR_WHEEL_FRONT,
	CAR_WHEEL_RIGHT_FRONT,
	CAR_WHEEL_LEFT_FRONT,
      CAR_WHEEL_BACK,
	CAR_WHEEL_RIGHT_BACK,
	CAR_WHEEL_LEFT_BACK,
    
    CAR_SPACE,
    CAR_SUBFRAME,
    CAR_BONNET,
    CAR_TOP,
    CAR_LUGGAGE_COMPARTMENT,

    CAR_WINDSCREEN,
      CAR_FRONT_WINDSCREEN,
      CAR_BACK_WINDSCREEN,

    CAR_BUMPER,
      CAR_BACK_BUMPER,
      CAR_FRONT_BUMPER,

    CAR_FENDER,
      CAR_FRONT_FENDER,
	CAR_FRONT_LEFT_FENDER,
	CAR_FRONT_RIGHT_FENDER,
      CAR_BACK_FENDER,
	CAR_BACK_LEFT_FENDER,
	CAR_BACK_RIGHT_FENDER,

    CAR_DOOR,
      CAR_LEFT_DOOR,
      CAR_RIGHT_DOOR,
    CAR_WINDOW_DOOR,
      CAR_LEFT_WINDOW_DOOR,
      CAR_RIGHT_WINDOW_DOOR,

    GLOBAL_SPACE,
    GROUND_SPACE,
    GROUND,
    OBSTACLE_SPACE,
    OBSTACLE,

  };
 
  const TypeList type;

  /**
   * Determine witch of the two Geoms will used its dealWith method on
   * the other.
   */
  bool isPrecedence(dGeomID geom) const;

  virtual  void dealWith(dGeomID me, dGeomID other) = 0;

  Type(TypeList type);
};



class DContactProperty : public Type {
private:
  dContact contact;

  DContactProperty();
  DContactProperty(const DContactProperty&);
  DContactProperty & operator=(const DContactProperty&);
  void mixdContact(dContact* res, const dContact* c1, const dContact* c2);

public:
  const dContact* getContact() const;

  DContactProperty(Type::TypeList type);
  virtual  void dealWith(dGeomID me, dGeomID other);
};

#endif
