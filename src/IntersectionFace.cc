#include "MeshIntersection.h"
#include "IntersectionFace.h"
#include "Calc.h"
#include <iostream>

namespace Geotree
{
  IntersectionFace::IntersectionFace(Mesh &_mesh, IntersectionPaths &_paths, Face _face)
    : paths(_paths), mesh(_mesh)
  {
    face = _face;
  }

  bool IntersectionFace2::isEdgePoint(int pathIndex, int pathStep)
  {
    for (EdgePoint2 point : edgePoints)
      {
	if (point.path == pathIndex && point.step == pathStep)
	  {
	    std::cout << "is edge point" << std::endl;
	    return true;
	  }
      }
  }

  EdgePoint2 IntersectionFace2::nextEdgePoint(PathT &path, int pathStep, bool trianglePoint, int triangleIndex)
  {
    bool returnNext = false;
    
    for (EdgePoint2 point : edgePoints)
      {
	if (returnNext)
	  {
	    return point;
	  }

          if (trianglePoint) {
              if (triangleIndex == point.segment)
              {
              returnNext = true;
          }
          }
	else if (paths[point.path] == path && point.step == pathStep)
	  {
	    returnNext = true;
	  }
      }

    if (returnNext)
      {
	return *edgePoints.begin();;
      }
  }
  
  PathT IntersectionFace2::getNextPoint(PathT &path, int &pathStep, bool &trianglePoint, int &triangleIndex)
  {
    if (trianglePoint || pathStep == 0 || pathStep == (path.size() - 1))
      {
	EdgePoint2 edgePoint = nextEdgePoint(path, pathStep, trianglePoint, triangleIndex);
          if (edgePoint.trianglePoint)
          {
              trianglePoint = true;
              triangleIndex = edgePoint.segment;
          } else {
              trianglePoint = false;
              //path = paths[edgePoint.path];
              pathStep = edgePoint.step;
          }
      }
    else
      {
	++pathStep;
      }


  }

  IntersectionFace2::IntersectionFace2(FaceT face)
    : FaceT(face)
  {
    
    std::cout << "face2" << std::endl;
    
    std::vector <int> indexPoints;

    indexPoints.push_back(mesh.F.row(index)(0));
    indexPoints.push_back(mesh.F.row(index)(1));
    indexPoints.push_back(mesh.F.row(index)(2));
    
    edgePoints.insert(EdgePoint2(0));
    edgePoints.insert(EdgePoint2(1));
    edgePoints.insert(EdgePoint2(2));

      faceEdgePoints.insert(FaceEdgePoint(mesh, mesh.F.row(index)(0), 0, 0.0));
      faceEdgePoints.insert(FaceEdgePoint(mesh, mesh.F.row(index)(1), 1, 0.0));
      faceEdgePoints.insert(FaceEdgePoint(mesh, mesh.F.row(index)(2), 2, 0.0));

      PathT path(mesh, indexPoints);
//    paths.push_back(path);
  }

//    std::vector <PathT> IntersectionFace2::completePath(PathT &path)
//    {
//
//    }

    FacePath IntersectionFace2::getPath(FaceEdgePoint edgepoint)
    {
        for (FacePath path : facepaths)
        {
            for (FacePoint point : path.points)
            {
                if (point.getIndex() == edgepoint.getIndex())
                {
                    return path;
                }
            }
        }
    }

    void IntersectionFace2::createEdgePaths()
    {
        std::set<FaceEdgePoint>::iterator it = faceEdgePoints.begin();

        while (it != faceEdgePoints.end() && it->position == 0.0 )
        {
            it++;
        }

        int startIndex = it->getIndex();

        FacePath path;

        path.points.push_back(*it++);

        do {

            if (it == faceEdgePoints.end())
            {
                it = faceEdgePoints.begin();
            }

            if (it->position > 0.0)
            {
                path.points.push_back(*it);
                facepaths.push_back(path);
                path.points.clear();
                path.points.push_back(*it);
            }
            else
            {
                path.points.push_back(*it);
            }
        } while (it++->getIndex() != startIndex);
    }

    void IntersectionFace2::completePaths2()
    {

    }


    void IntersectionFace2::completePaths2(std::set<FaceEdgePoint>::iterator point)
    {
        FacePath path;

        int pathStart = point->getIndex();

        std::vector<FaceEdgePoint>::iterator iter;

        do
        {
            path.points.push_back(*point++);

            if (point->position > 0.0)
            {

                FacePath cutPath = getPath(*point);

                if (cutPath.points.begin()->getIndex() == point->getIndex())
                {
                    //completePaths2(point);

                    for (iter = cutPath.points.begin(); iter != cutPath.points.end(); iter++)
                    {
                        path.points.push_back(*iter);
                    }
                }
                else if (cutPath.points.rbegin()->getIndex() == point->getIndex())
                {
                    for (std::vector<FaceEdgePoint>::reverse_iterator riter = cutPath.points.rbegin(); riter != cutPath.points.rend(); riter++)
                    {
                        path.points.push_back(*riter);
                    }
                }
                else
                {
                    throw;
                }

                point = faceEdgePoints.begin();

                while (point->getIndex() != path.points.back().getIndex())
                {
                    ++point;
                    std::cout << path.points.back().getIndex() << ":"<< point->getIndex() << std::endl;
                }

            }

            if (point == faceEdgePoints.end())
            {
                point = faceEdgePoints.begin();
            }

        } while(pathStart != point->getIndex());

        completeFacePaths.push_back(path);
    }

    PathT IntersectionFace2::nextPoint(const PathT path)


    PathT IntersectionFace2::completePathForward(const PathT path)
    {
        PathT newPath(mesh);

        Point startPoint = path.begin();

        Point currentPoint = path.back();
                //nextPoint(startPoint);

        PathT::iterator = pathIterator;

        do {
            newPath.add(currentPoint);

            pathIterator++;

        } while (startPoint != pathIterator);

        return newPath;
    }

    void IntersectionFace2::completePaths() {
        //std::set<FaceEdgePoint>::iterator point = faceEdgePoints.begin();

        //completePaths2(point);

        //return;

        int pathNum = 0;

        for (PathT &path : paths) {
            completePathForward(path);
        }
//            bool complete = false;
//
//            int startIndex = path.start();
//            int current = path.end();
//
//            int pathStep = path.size()-1;
//            int pathIndex = pathNum;
//            bool trianglePoint = false;
//            int triangleIndex;
//
//            while (! complete)
//            {
//                if (current == path.start())
//                {
//                    complete = true;
//                }
//                else
//                {
//                    getNextPoint(pathIndex, pathStep, trianglePoint, triangleIndex);
//                    if (trianglePoint) {
//                        std::cout << "triangle point " << triangleIndex << std::endl;
//
//                        current = mesh.F.row(index)(triangleIndex);
//                    } else {
//                        std::cout << "path point " << pathIndex << ", " << pathStep << std::endl;
//
//                        current = paths[pathIndex].index(pathStep);
//                    }
//                    if (current == path.start()) {
//                        complete = true;
//                    } else {
//                        path.addExisting(current);
//                    }
//                    std::cout << "adding point " << current << std::endl;
//                }
//            }
//
//            std::cout << path << std::endl;
//
//            ++pathNum;
//        }
    }

    void IntersectionFace2::cut2(std::vector <IntersectionPoint> points)
    {
        FacePath path;

        for (IntersectionPoint point: points)
        {
            if (point.first.type == SEGMENT)
            {
                Vector3d segmentStart = mesh.V.row(point.first.index);
                double distance = (point.getVector() - segmentStart).norm();
                int segment;

                if (point.first.index == mesh.F.row(index)(0))
                {
                    segment = 0;
                }
                else if (point.first.index == mesh.F.row(index)(1))
                {
                    segment = 1;
                }
                else if (point.first.index == mesh.F.row(index)(2))
                {
                    segment = 2;
                }
                else
                {
                    throw;
                }

                faceEdgePoints.insert(FaceEdgePoint(mesh, point.getIndex(), segment, distance));
            }

            path.points.push_back(FaceEdgePoint(mesh, point.getIndex()));
        }

        facepaths.push_back(path);
    }

  void IntersectionFace2::cut(std::vector <IntersectionPoint> points)
  {
    std::vector <int> indexPoints;

    for (IntersectionPoint point: points)
      {
	if (point.first.type == SEGMENT && point.first.getFaces(mesh).count(index))
	  {
	    Vector3d segmentStart = mesh.V.row(point.first.index);
	    double distance = (point.getVector() - segmentStart).norm();
	    int faceIndex;

	    if (point.first.index == mesh.F.row(index)(0))
	      {
		faceIndex = 0;
	      }
	    else if (point.first.index == mesh.F.row(index)(1))
	      {
		faceIndex = 1;
	      }
	    else if (point.first.index == mesh.F.row(index)(2))
	      {
		faceIndex = 2;
	      }
	    else
	      {
		throw;
	      }

	    EdgePoint2 edgePoint(paths.size(), indexPoints.size(), faceIndex, distance);
	    edgePoints.insert(edgePoint);
	  }

	indexPoints.push_back(point.getIndex());
      }

    PathT path(mesh, indexPoints);

    std::cout << path << std::endl;
    std::cout << mesh << std::endl;

    paths.push_back(path);
  }
  
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
    // 	    IntersectionPoint ip = points2.getVector(i);
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
