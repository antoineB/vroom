#include <ode/ode.h>
#include "OgreFramework.hpp"
#include "global.hpp"

namespace MyTools{
  using namespace Ogre;

  Vector3* getVerticesArray(const VertexData *vertex_data);
  unsigned int getVerticesNumber(const VertexData *vertex_data);
  unsigned int getIndicesNumber(IndexData *data);
  dTriIndex* getIndicesArray(IndexData *data, const unsigned int offset);

  void byOdeToOgre(dBodyID b, SceneNode* n);
  void byOdeToOgre(dGeomID g, SceneNode* n);
  dTriMeshDataID dTriMeshDataFromMesh(Ogre::Entity* e);
};

  /*{//creation de l'obstable
    
    Ogre::Entity* entity=_sceneMgr->getEntity("obs1");
    Ogre::SubMesh *sub_mesh = entity->getSubEntity(0)->getSubMesh();

    unsigned int vertex_number=MyTools::
      getVerticesNumber(sub_mesh->vertexData);
    
    unsigned int index_number=MyTools::
      getIndicesNumber(sub_mesh->indexData);

    Ogre::Vector3* tmp=MyTools::getVerticesArray(sub_mesh->vertexData);    
    dReal* vertices =new dReal[vertex_number*4];
    for(int i=0; i< vertex_number; i++){
      vertices[i*4 +0]=(dReal)tmp[i].x;
      vertices[i*4 +1]=(dReal)tmp[i].y;
      vertices[i*4 +2]=(dReal)tmp[i].z;
      vertices[i*4 +3]=0;
    }
    delete[] tmp;

    std::cout<<"ok"<<std::endl;

    dTriIndex* indices=MyTools::getIndicesArray( sub_mesh->indexData, 
					       0); //0 car un seul submesh
     
    bodys[1] = dBodyCreate (_world);

    dTriMeshDataID data = dGeomTriMeshDataCreate();
  
    std::cout<<"oaaa"<<std::endl;
    dGeomTriMeshDataBuildSimple(data,
				vertices, vertex_number,
				indices, index_number);
    
    geoms[1] = dCreateTriMesh(_space,data,0,0,0);
    
    std::cout<<"zzz"<<std::endl;
    // a supprimer
    dMassSetTrimesh (&(weights[1]), 1.0, geoms[1]);
    
    std::cout<<"eee"<<std::endl;
    //a supprimer
    dGeomSetBody (geoms[1],bodys[1]);

    std::cout<<"www"<<std::endl;
    std::cout<<weights[1].c[0]<<" "
	     <<weights[1].c[1]<<" "
	     <<weights[1].c[2]<<" "
	     <<std::endl;
    
    weights[1].c[0] = 0.0;
    weights[1].c[1] = 0.0;
    weights[1].c[2] = 0.0;
    
    //a supprimer
    //    dMassSetPosition(&weights[1], .0, .0, .0);
    dBodySetMass (bodys[1],&weights[1]);

    //ODE INTERNAL ERROR 2: The centre of mass must be at the origin. in dBodySetMass()
    std::cout<<"sss"<<std::endl;

    //http://pastebin.com/imSgSkHA

    std::cout<<"ggg"<<std::endl;
    //appliquer la position de l'object physique a l'object graphique
    //dBodySetPosition (bodys[1],0, 1.1,0);
    dGeomSetPosition (geoms[1], 5.0, 3.0,0);
    MyTools::byOdeToOgre(geoms[1],_sceneMgr->getSceneNode("obs1"));
    std::cout<<"qqq"<<std::endl;
    }*/
