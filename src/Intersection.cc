#include "Intersection.h"

namespace Geotree
{  
  Intersection::Intersection(const Mesh mesh1, const Mesh mesh2)
  {
    for (int i=0; mesh1.F.rows(); i++)
      {
	for (int j=0; mesh2.F.rows(); j++)
	  {
	    //Face face1 = mesh1.getFace(i);
	    //Face face2 = mesh2.getFace(j);

	    //addIntersections(face1, face2);
	  }
      }
  }
  
  // void Intersection::addIntersections(Face face1, Face face2)
  // {
    
  // }
}
