#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "OgreFramework.hpp"
#include "global.hpp"
#include "geom.hpp"
#include "type.hpp"

class Obstacle: public Geom{
  std::string name;

  Obstacle();
  Obstacle(const Obstacle&);
  Obstacle & operator=(const Obstacle&);

  static void fillContact();

public:
  Obstacle(const char *n, const char* meshName, dReal x=.0, dReal y=.0, dReal z=.0);
  inline void initGeom(void *ptr);
  inline void update();
  void setMaterial(const char* name) const;
  static DContactType type;
};

#endif
