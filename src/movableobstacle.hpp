#ifndef MOVABLE_OBSTACLE_HPP
#define MOVABLE_OBSTACLE_HPP

#include "OgreFramework.hpp"
#include "global.hpp"
#include "geom.hpp"
#include "type.hpp"

class MovableObstacle: public Geom {
private:
  std::string name;
  dMass mass;

  static DContactType type;

  MovableObstacle();
  MovableObstacle(const MovableObstacle&);
  MovableObstacle & operator=(const MovableObstacle&);

public:
  dBodyID body;

  MovableObstacle(const char *n, const char* meshName, dReal x=.0, dReal y=.0, dReal z=.0);
  void update();
  void setMaterial(const char* nameMat) const;
};

#endif
