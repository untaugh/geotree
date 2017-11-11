#include "MeshIntersection.h"
#include "Calc.h"


namespace Geotree
{
  IntersectionFace::IntersectionFace(Mesh &_mesh, IntersectionPaths &_paths, Face _face)
    : paths(_paths), mesh(_mesh)
  {
    face = _face;
  }

  // void IntersectionFace::calculateSubpaths()
  // {
    
  // }
  
  void IntersectionFace::calculateEdgePoints()
  {
    Segments segments;

    Calc::getSegments(face.transpose(), segments);

    EdgePoint edgePoint;
    edgePoint.type = POINT;
    edgePoint.index = face[0];
    edgePoint.segment = segments.row(0);    
    edgePoints.insert(edgePoint);
    
    edgePoint.index = face[1];
    edgePoint.segment = segments.row(1);
    edgePoints.insert(edgePoint);
    
    edgePoint.index = face[2];
    edgePoint.segment = segments.row(2);
    edgePoints.insert(edgePoint);
     
    edgePoint.type = SEGMENT;

    // std::vector <FaceSet> getSubPaths(int face);

    // for (FaceSet subpath : subpaths)
    //   {
    // 	for (int i : subpath)
    // 	  {
    // 	    IntersectionPoint ip = points2.getPoint(i);
    // 	    SegmentIndex segment;

    // 	    if (ip.first.isSegment())
    // 	      {
    // 		segment = ip.first.getSegment();
    // 		std::cout << segment.transpose() << std::endl;
    // 		if (Calc::hasSegment(faceIndex, segment))
    // 		  {
    // 		    edgePoint.index = i;
    // 		    edgePoints.push_back(edgePoint);		    
    // 		  }
    // 	      }

    // 	    if (ip.second.isSegment())
    // 	      {
    // 		segment = ip.second.getSegment();
    // 		if (Calc::hasSegment(faceIndex, segment))
    // 		  {
    // 		    edgePoint.index = i;
    // 		    edgePoints.push_back(edgePoint);		    
    // 		  }
    // 	      }
    // 	  }
    //   }
  }
  
}
