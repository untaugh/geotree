#include "FacesDivide.h"
#include <Eigen/Dense>

namespace Geotree
{

  FacesDivide::FacesDivide(Paths &paths)
    : paths(paths)
  {
    // add existing faces
    for (int face : this->paths.faces(MESH0))
      {
	int newindex = this->newMesh.add(paths.intersection.mesh0.getFace(face));
	FacePart facepart;
	facepart.faceindex = face;
	facepart.mesh = MESH0;
	facepart.newindex = newindex;
	faceparts.push_back(facepart);
      }
    
    for (int face : this->paths.faces(MESH1))
      {
	int newindex = this->newMesh.add(paths.intersection.mesh1.getFace(face));
	FacePart facepart;
	facepart.faceindex = face;
	facepart.mesh = MESH1;
	facepart.newindex = newindex;
	faceparts.push_back(facepart);
      }

    // for (FacePart facepart : this->faceparts)
    //   {
    // 	this->divide(facepart);
    //   }
  }

  void FacesDivide::divide(FacePart &facepart)
  {
    std::vector <std::set<Point>> facepaths = paths.getPaths(facepart.faceindex);
    
  }
}
