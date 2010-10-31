#ifndef TYPE_HPP
#define TYPE_HPP

class Type;
enum Type::Type;

class Type{

  Type::Type type;

public:
  Type::Type getType();

  enum Type{
    NONE=0,
    WHEEL_FRONT,
    WHEEL_BACK,
    SUBFRAME,
    GROUND,
    OBSTACLE,

  }


};

#enndif
