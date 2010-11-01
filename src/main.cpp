#include "demo.hpp"

#include "global_def.hpp"
#include "world.hpp"
#include "car.hpp"

//variables global

struct Global _glb;
 
//partie physique
std::vector<dGeomID> geoms;
std::vector<std::string> names;

Car car;

int main(int argc, char **argv){
  //set globals variables
  memset(&_glb,0,sizeof(_glb));

  //set up world
  new World("world.xml");

  try{
    demo de;
    de.startDemo();
   }
  catch(std::exception& e){
    fprintf(stderr, "An exception has occurred: %s\n", e.what());
  }

  
  World::getSingletonPtr()->printCst();

  delete World::getSingletonPtr();
  return 0;
 }

