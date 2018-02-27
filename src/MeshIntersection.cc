#include "MeshIntersection.h"
#include "Calc.h"
#include "Segment.h"
#include "Face.h"
#include "Log.h"
#include "IntersectionFace.h"
#include <iostream>

namespace Geotree
{
  MeshIntersection::MeshIntersection(Mesh _mesh1, Mesh _mesh2)
    : mesh(_mesh1 + _mesh2),
      paths(points),
      secondMeshFOffset(_mesh1.facecount()),
      secondMeshVOffset(_mesh1.size()),
	  points(mesh)
  {
    calculatePoints();
    points.calculateConnectedPoints();
    paths.calculatePaths();
  }

  void MeshIntersection::calculatePoints()
  {
    Segments segments = mesh.getSegments();

    for (int faceIndex=0; faceIndex<mesh.F.rows(); faceIndex++)
      {
	for (int segmentIndex=0; segmentIndex<segments.rows(); segmentIndex++)
	  {
	    SegmentIndex segment = segments.row(segmentIndex);
	    
	    if ( (segment >= secondMeshVOffset ))
	      {
		if (faceIndex < secondMeshFOffset)
		  {
		    FaceT facet = mesh.getFaceT(faceIndex);
		    SegmentT segmentt = SegmentT(mesh.V, segment);
	    
		    if (facet.intersects(segmentt.getVectors()))
		      {
			IntersectionPoint point = facet.getIntersection(segmentt);
			points.addPoint(point);
		      }
		  }
	      }
	    else
	      {
		if (faceIndex >= secondMeshFOffset)
		  {
		    FaceT facet = mesh.getFaceT(faceIndex);
		    SegmentT segmentt = SegmentT(mesh.V, segments.row(segmentIndex));
		    
		    if (facet.intersects(segmentt.getVectors()))
		      {
			IntersectionPoint point = facet.getIntersection(segmentt);
			points.addPoint(point);
		      }
		  }
	      }
	  }
      }
  }



	Vector3i MeshIntersection::getFace(int face)
  {
    return mesh.F.row(face);
  }

//  std::vector <EdgePoint> MeshIntersection::getEdgePoints(int face, std::vector <FaceSet> &subpaths)
//  {
//    std::vector <EdgePoint> edgePoints;
//
//    Segments segments;
//	  Vector3i faceIndex = getFace(face);
//
//    Calc::getSegments(faceIndex.transpose(), segments);
//
//    EdgePoint edgePoint;
//    edgePoint.type = POINT;
//    edgePoint.index = faceIndex[0];
//    edgePoint.segment = segments.row(0);
//    edgePoints.push_back(edgePoint);
//
//    edgePoint.index = faceIndex[1];
//    edgePoint.segment = segments.row(1);
//    edgePoints.push_back(edgePoint);
//
//    edgePoint.index = faceIndex[2];
//    edgePoint.segment = segments.row(2);
//    edgePoints.push_back(edgePoint);
//
//    edgePoint.type = SEGMENT;
//    for (FaceSet subpath : subpaths)
//      {
//	for (int i : subpath)
//	  {
//	    IntersectionPoint ip = points.getPoint(i);
//	    SegmentIndex segment;
//
//	    if (ip.first.isSegment())
//	      {
//		segment = ip.first.getSegment();
//		//std::cout << segment.transpose() << std::endl;
//		if (Calc::hasSegment(faceIndex, segment))
//		  {
//		    edgePoint.index = i;
//		    edgePoints.push_back(edgePoint);
//		  }
//	      }
//
//	    if (ip.second.isSegment())
//	      {
//		segment = ip.second.getSegment();
//		if (Calc::hasSegment(faceIndex, segment))
//		  {
//		    edgePoint.index = i;
//		    edgePoints.push_back(edgePoint);
//		  }
//	      }
//	  }
//      }
//    return edgePoints;
//  }

//  bool MeshIntersection::operator ()(EdgePoint first, EdgePoint second)
//  {
//    SegmentIndex segmentFirst, segmentSecond;
//    int pointFirst, pointSecond;
//    Vector vectorBase,vectorFirst, vectorSecond;
//
//    if (first.type == SEGMENT)
//      {
//	IntersectionPoint point = points.getPoint(first.index);
//	//segmentFirst = point.first.index2.block(0,0,2,1);
//	segmentFirst= point.first.getSegment();
//	pointFirst = segmentFirst[0];
//	vectorFirst = point.getVector();
//      }
//    else
//      {
//	//pointFirst = points.getVector(first.index).first.index2[0];
//	pointFirst = points.getPoint(second.index).first.index;
//	vectorFirst = mesh.V.row(pointFirst);
//      }
//
//    if (second.type == SEGMENT)
//      {
//	IntersectionPoint point = points.getPoint(second.index);
//	//segmentSecond = point.first.index2.block(0,0,2,1);
//	segmentFirst= point.first.getSegment();
//	pointSecond = segmentSecond[0];
//	vectorSecond = point.getVector();
//      }
//    else
//      {
//	//pointSecond = points.getVector(second.index).first.index2[0];
//	pointSecond = points.getPoint(second.index).first.index;
//	vectorSecond = mesh.V.row(pointSecond);
//      }
//
//    //std::cout << "Compare " << segmentFirst.transpose() << ", " << segmentSecond.transpose() << std::endl;
//    if (pointFirst < pointSecond)
//      {
//	return true;
//      }
//    else if (pointFirst > pointSecond)
//      {
//	return false;
//      }
//    else
//      {
//	vectorBase = mesh.V.row(pointFirst);
//
//	double d1 = (vectorBase - vectorFirst).norm();
//	double d2 = (vectorBase - vectorSecond).norm();
//
//	if (d1 < d2)
//	  {
//	    return true;
//	  }
//	else
//	  {
//	    return false;
//	  }
//      }
//
//  }
  
//  void MeshIntersection::sortEdgePoints(int face, std::vector <EdgePoint> &edgePoints)
//  {
//
//	  Vector3i faceIndex = getFace(face);
//    Segments segments;
//
//    Calc::getSegments(faceIndex.transpose(), segments);
//
//    //std::cout << "segments " << segments << std::endl;
//
//    for (EdgePoint &ep : edgePoints)
//      {
//    	SegmentIndex segment;
//
//    	if (ep.type == SEGMENT)
//    	  {
//	    SegmentIndex seg;
//	      //SegmentIndex seg = points.getPoint(ep.index).getVector(face.mesh).index2.block(0,0,2,1);
//	    ep.segment = seg;
//
//	    //std::cout << "seg " << ep.segment.transpose() << std::endl;
//	    Vector start;// = getMesh(face.mesh).V.row(ep.segment[0]);
//
//	    ep.position = (start - points.getPoint(ep.index).getVector()).norm();
//    	  }
//	else
//	  {
//	    //std::cout << "segindex " << ep.index << std::endl;
//	    ep.position = 0.0;
//	  }
//      }
//
//    std::sort (edgePoints.begin(), edgePoints.end());
//  }

    void MeshIntersection::merge() {

		FaceSet intersectedFaces = points.getIntersectedFaces();

		for (unsigned faceIndex : intersectedFaces)
		{
			IntersectionFace face = mesh.getFaceT(faceIndex);
			std::vector <PathX<FacePoint>> facePaths = paths.getSubPaths2(faceIndex);

			for (auto path : facePaths) {
				face.cut(path);
			}

			face.completePaths();
			face.triangulate();
		}

    }

    // void MeshIntersection::printPoints(std::vector <EdgePoint> edgePoints)
  // {
  //   std::cout << "EdgePoints: ";	    
  //   for (EdgePoint ep : edgePoints)
  //     {
  // 	if (ep.type == POINT)
  // 	  {
  // 	    std::cout << "P";	    
  // 	  }
  // 	else
  // 	  {
  // 	    std::cout << "S";
  // 	  }
  // 	std::cout << "("<< ep.segment.transpose() << ",";
  // 	std::cout << ep.position << "), ";
  //     }
  //   std::cout << std::endl;
  // }
  
  // void MeshIntersection::printPoints()
  // {
    //int index = 0;
    // for (IntersectionPoint ip : points.getPoints())
    //   {
    // 	std::cout << index << " ";
    // 	if (ip.first.type == POINT)
    // 	  {
    // 	    std::cout << "P";
    // 	  }
    // 	else if (ip.first.type == SEGMENT)
    // 	  {
    // 	    std::cout << "S";
    // 	  }
    // 	else if (ip.first.type == FACE)
    // 	  {
    // 	    std::cout << "F";
    // 	  }

    // 	if (ip.second.type == POINT)
    // 	  {
    // 	    std::cout << "P";
    // 	  }
    // 	else if (ip.second.type == SEGMENT)
    // 	  {
    // 	    std::cout << "S";
    // 	  }
    // 	else if (ip.second.type == FACE)
    // 	  {
    // 	    std::cout << "F";
    // 	  }

	
    // 	std::cout << " [" << ip.first.index2.transpose() << "] ";
    // 	std::cout << " [" << ip.second.index2.transpose() << "] ";
	
    // 	std::cout << " (" << ip.point.transpose() << ")" << std::endl;
	
    // 	index++;
    //   }
  //}

  Path IntersectionPaths::calculatePath(int intersectionPointIndex)
  {
    std::vector <int> vectorpath;

    std::pair <int, int> points;
    
    FaceSet pointsset;
    
    int previouspoint = -1;
    int point = intersectionPointIndex;
    int count = 0;

    do
      {
	vectorpath.push_back(point);

	IntersectionPoint ip = this->intersectionPoints.getPoint(point);
	pointsset = ip.connectedPoints;
	//pointsset = this->intersectionPoints.getConnectedPoints(point);
	
	if (pointsset.size() > 2)
	  {
	    for (int i: pointsset)
	      {
		if (intersectionPoints.getPoint(i).connectedPoints.size() > 2)
		   {
		     pointsset.erase(i);
		   }
	      }
	  }

	if (pointsset.size() != 2)
	  {
	    std::cout << "Size is not 2: " << pointsset.size() <<std::endl;
	    return Path();
	  }

	points.first = *pointsset.begin();
	points.second = *pointsset.rbegin();
	//std::cout << point << " points: " << points.first << ", " <<points.second << std::endl;
	if (points.first != previouspoint)
	  {
	    previouspoint = point;
	    point = points.first;
	  }
	else if (points.second != previouspoint)
	  {
	    previouspoint = point;
	    point = points.second;
	  }
	else
	  {
	    return Path();
	  }
	count++;
      } while(point != intersectionPointIndex);

    Path path(vectorpath.size());

    int i = 0;
    for (int p : vectorpath)
      {
	path[i] = p;
	  i++;
      }
    return path;
  }
}
