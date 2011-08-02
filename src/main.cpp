#include "demo.hpp"

#include "utils.hpp"

#include "global_def.hpp"
#include "world.hpp"
#include "car.hpp"

//variables global
struct Global _glb;
 
//physical part
std::vector<dGeomID> geoms;
std::vector<std::string> names;

Car car;

unsigned long BitField::getCategorieStaticEnvironement() {
  return 0x00000001;
}

unsigned long BitField::getCollideStaticEnvironement() {
  return 0xFFFFFFFF ^ getCategorieStaticEnvironement();
}



int main(int argc, char **argv){
  //set globals variables
  memset(&_glb,0,sizeof(_glb));


  //set up world
  new World("../xml/world.xml");

  try{
    demo de;
    de.startDemo();
   }
  catch(std::exception& e){
    fprintf(stderr, "An exception has occurred: %s\n", e.what());
  }

  delete World::getSingletonPtr();
  return 0;
 }

