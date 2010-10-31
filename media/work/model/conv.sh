
if [ ! -e $1 ] ; then 
    echo "usage : myxml.xml mymesh.mesh"
    echo "pas de fichier valide spécifié" ; 
    exit -1 ; 
fi

if [ $2 = "" ] ; then
    echo "pas de fichier de sortie" ; 
    exit -1 ; 
fi

#name=`echo "$1" |cut -d'.' -f1,2` ;
name=`echo "$1" |sed s/.xml//` ;

OgreXMLConverter $1 ;
OgreMeshUpgrader $name ;
mv $name ../../meshs/$2 ;

rm OgreXMLConverter.log OgreMeshUpgrade.log $1 ;