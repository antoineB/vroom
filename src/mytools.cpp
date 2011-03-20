#include "mytools.hpp"


using namespace Ogre;

Vector3* MyTools::getVerticesArray(const VertexData *vertex_data){
  if (!vertex_data) {
    return NULL;
  }
  else{	
    //buffer of vertices
    Vector3* vertices =  new Ogre::Vector3[vertex_data->vertexCount];
    
    const Ogre::VertexElement* posElem = vertex_data->vertexDeclaration
      ->findElementBySemantic(Ogre::VES_POSITION);			
    
    Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding
      ->getBuffer(posElem->getSource());
    
    //size of a vertex, normaly 3*4 bytes
    const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
    
    unsigned char* vertex = static_cast<unsigned char*>
      (vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
    
    float* pReal;
    //Ogre::Vector3 * curVertices = &vertices[vertex_data->vertexCount];
    
    const unsigned int vertexCount = (unsigned int)vertex_data->vertexCount;

    for(unsigned int j = 0; j < vertexCount; ++j){
      posElem->baseVertexPointerToElement(vertex, &pReal);
      vertex += vSize;
      
      vertices[j].x = (*pReal++);
      vertices[j].y = (*pReal++);
      vertices[j].z = (*pReal++);
      //*curVertices = _transform * (*curVertices);
      //si on lui applique une transformation
      
      //	curVertices++;
    }
    vbuf->unlock();
    return vertices;
  }
}

unsigned int MyTools::getVerticesNumber(const VertexData *vertex_data){
  if (!vertex_data) 
    return 0;
  
  return (unsigned int)vertex_data->vertexCount;
}

unsigned int MyTools::getIndicesNumber(IndexData *data){
  return data->indexCount;
}

dTriIndex* MyTools::getIndicesArray(IndexData *data, const unsigned int offset){
  dTriIndex* tmp = new dTriIndex[data->indexCount];
  
  const unsigned int numTris = (unsigned int) data->indexCount / 3;
  
  HardwareIndexBufferSharedPtr ibuf = data->indexBuffer;	
  
  const bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);
  
  unsigned int index_offset = 0;
  
  if (use32bitindexes){
      const unsigned int* pInt = static_cast<unsigned int*>
  	(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
      
      for(unsigned int k = 0; k < numTris; ++k)
        {
  	  tmp[index_offset ++] = offset + *pInt++;
  	  tmp[index_offset ++] = offset + *pInt++;
  	  tmp[index_offset ++] = offset + *pInt++;
        }
      ibuf->unlock();
  }
    else 
      {
        const unsigned short* pShort = static_cast<unsigned short*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
  	for(unsigned int k = 0; k < numTris; ++k)
	  {
  	  tmp[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
  	  tmp[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
  	  tmp[index_offset ++] = offset + static_cast<unsigned int> (*pShort++);
	  }
        ibuf->unlock();
      }
  
  return tmp;
}

void MyTools::byOdeToOgre(dBodyID b, SceneNode* n){
  const dReal *pos = dBodyGetPosition (b);
  const dReal *quat = dBodyGetQuaternion(b);
  
  n->setPosition((Ogre::Real)pos[0],
		 (Ogre::Real)pos[1], 
		 (Ogre::Real)pos[2]);
  
  n->setOrientation( (Ogre::Real)quat[0], (Ogre::Real)quat[1], 
		     (Ogre::Real)quat[2], (Ogre::Real)quat[3] );
}

void MyTools::byOdeToOgre(dGeomID g, SceneNode* n){
  const dReal *pos = dGeomGetPosition (g);
  dQuaternion quat;
  dGeomGetQuaternion(g, quat );
  
  n->setPosition((Ogre::Real)pos[0],
		 (Ogre::Real)pos[1], 
		 (Ogre::Real)pos[2]);
  
  n->setOrientation( (Ogre::Real)quat[0], (Ogre::Real)quat[1], 
		     (Ogre::Real)quat[2], (Ogre::Real)quat[3] );
}

dTriMeshDataID MyTools::dTriMeshDataFromMesh(Ogre::Entity* e){
  Ogre::SubMesh *sub_mesh = e->getSubEntity(0)->getSubMesh();
  
  if(e->getNumSubEntities()>1){
    dbg_("MyTools:: "<<e->getNumSubEntities()<<" de sub Entity");
    return NULL;
  }
  if(e->getMesh()->sharedVertexData){
    dbg_("shared vertex found");
    return NULL;
  }
  
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
 
  dTriIndex* indices=MyTools::getIndicesArray( sub_mesh->indexData, 
					       0); //0 car un seul submesh
                                //le 0 peut être remplacer par le nombres index_number
     
  // for(int i=0; i<vertex_number; i++){
  //   std::cout<<vertices[i*4 +0]<<" "<<vertices[i*4 +1]<<" "
  // 	     <<vertices[i*4 +2];
  //   std::cout<<std::endl;
  // }
  // std::cout<<std::endl;
  // for(int i=0; i<index_number; i++){
  //   std::cout<<indices[i]<<" ";
  // }
  // std::cout<<std::endl;

  dTriMeshDataID data = dGeomTriMeshDataCreate();
  
  dGeomTriMeshDataBuildSimple(data,
			      vertices, vertex_number,
			      indices, index_number);
  
  return data;
}

