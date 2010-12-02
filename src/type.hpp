#ifndef TYPE_HPP
#define TYPE_HPP


/**
 * This class is used to be associated with a geom. So during the the
 * collision the geom type could be easily retrieved.
 */
class Type{

public:
  enum TypeList{
    NONE=0,

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

    GROUND,
    OBSTACLE,

  };
 

  const TypeList type;
  const void *dataPointer;

  Type(void *pointer, TypeList type = TypeList::NONE);
};

#endif
