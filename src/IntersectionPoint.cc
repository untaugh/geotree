#include "IntersectionPoint.h"
#include "MeshIntersection.h"
#include "Calc.h"
#include <iostream>
#include "Log.h"
#include <algorithm>
#include "Face.h"
#include "Segment.h"

namespace Geotree
{

  IntersectionPoint::IntersectionPoint(PointInfo firstType, PointInfo secondType, Vector _point)
    : first(firstType), second(secondType), point(_point) {};
  
  MeshIntersection::MeshIntersection(Mesh _mesh1, Mesh _mesh2)
    : mesh1(_mesh1), mesh2(_mesh2)
  {
    mesh = mesh1 + mesh2;
  }

  void MeshIntersection::addPoint(IntersectionPoint _point)
  {
    bool exists = false;
    for (IntersectionPoint ip : points)
      {	
	if (ip == _point)
	  {
	    exists = true;
	    break;
	  }
      }

    if (!exists)
      {
	Geotree::Log().Get(LOG_DEBUG)
	  << "Adding point: "  << _point.first.type << ", " << _point.second.type << ", " << _point.point.transpose();	
	points.push_back(_point);
      }
  }
  
  void MeshIntersection::calculatePoints()
  {
    Segments segments = mesh2.getSegments();
    
    for (int faceIndex=0; faceIndex<mesh1.F.rows(); faceIndex++)
      {
	for (int segmentIndex=0; segmentIndex<segments.rows(); segmentIndex++)
	  {
	    FaceT facet = mesh1.getFaceT(faceIndex);
	    //SegmentT segmentt = mesh2.getSegmentT(segmentIndex);

	    SegmentT segmentt = SegmentT(mesh2.V, segments.row(segmentIndex));
	    
	    if (facet.intersects(segmentt.getVectors()))
	      {
		IntersectionPoint point = facet.getIntersection(segmentt);
		addPoint(point);
	      }
	  }	  
      }

    segments = mesh1.getSegments();
    for (int faceIndex=0; faceIndex<mesh2.F.rows(); faceIndex++)
      {
	for (int segmentIndex=0; segmentIndex<segments.rows(); segmentIndex++)
	  {

	    FaceT facet = mesh2.getFaceT(faceIndex);
	    //SegmentT segmentt = mesh1.getSegmentT(segmentIndex);
	    SegmentT segmentt = SegmentT(mesh1.V, segments.row(segmentIndex));

	    if (facet.intersects(segmentt.getVectors()))
	      {
		IntersectionPoint point = facet.getIntersection(segmentt);
		
		PointInfo tmp = point.first;
		point.first = point.second;
		point.second = tmp;

		addPoint(point);
	      }
	  }	  
      }
  }
  
    
  FaceSet MeshIntersection::getIntersectionPoints(FaceSet facesFirst, FaceSet facesSecond)
  {
    FaceSet pointsFound;
    int count = 0;
    for (IntersectionPoint findPoint : points)
      {
	if (Calc::contains(facesFirst, findPoint.first.faces) && Calc::contains(facesSecond, findPoint.second.faces))
	  {
	    pointsFound.insert(count);
	  }
	count++;
      }
    
    return pointsFound;
  }

  IntersectionPoint MeshIntersection::getPoint(int pointIndex)
  {
    return points[pointIndex];
  }
  
  FaceSet MeshIntersection::getConnectedPoints(int pointIndex)
  {
    FaceSet intersectionPoints = getIntersectionPoints(getPoint(pointIndex).first.faces, getPoint(pointIndex).second.faces);

    intersectionPoints.erase(pointIndex);

    return intersectionPoints;
  }

  void MeshIntersection::calculatePointFaces()
  {
    for (int i=0; i<points.size(); i++)
      {
	points[i].first.getFaces(mesh1);
	points[i].second.getFaces(mesh2);
      }
  }
  
  void MeshIntersection::calculateConnectedPoints()
  {
    for (int i=0; i<points.size(); i++)
      {
	points[i].connectedPoints = getConnectedPoints(i);	
      }
  }

  std::pair <FaceSet, FaceSet> MeshIntersection::getIntersectedFaces()
  {
    std::pair <FaceSet, FaceSet> intersectedFaces;

    for (IntersectionPoint ip : points)
      {
	if (ip.first.type == PointInfo::FACE || ip.first.type == PointInfo::SEGMENT)
	  {
	    intersectedFaces.first.insert(ip.first.faces.begin(), ip.first.faces.end());
	  }

	if (ip.second.type == PointInfo::FACE || ip.second.type == PointInfo::SEGMENT)
	  {
	    intersectedFaces.second.insert(ip.second.faces.begin(), ip.second.faces.end());
	  }
      }
    return intersectedFaces;
  }  

  PointInfo IntersectionPoint::getPoint(const MeshIndex mesh)
  {
    if (mesh == FIRST)
      {
	return first;
      }
    else
      {
	return second;
      }
  }
  
  int MeshIntersection::getPathStartIndex(Path path, MeshFace face)
  {
    int i = 0;

    IntersectionPoint ip = points[path[i]];
    PointInfo point = ip.getPoint(face.mesh);

    while(point.faces.count(face.index))
      {
	if (++i >= path.size())
	  {
	    return 0;
	  }

	ip = points[path[i]];
	point = ip.getPoint(face.mesh);
      }
    return i;
  }

  std::vector <FaceSet> MeshIntersection::getSubPaths(MeshFace face)
  {
    std::vector <FaceSet> subpaths;
    
    for (Path path : paths)
      {
	int pathStart = getPathStartIndex(path, face);
	int i = 0;

	while(i < path.size())
	  {
	    FaceSet subpath;
	    int pathIndex = (pathStart + i) % path.size();
	    
	    while (i < path.size() && points[path[pathIndex]].getPoint(face.mesh).faces.count(face.index))
	      {
		subpath.insert(path[pathIndex]);
		++i;
		pathIndex = (pathStart + i) % path.size();
	      }
	    
	    if (subpath.size() > 0)
	      {
		subpaths.push_back(subpath);
		subpath.clear();
	      }
	    else
	      {
		i++;
	      }
	  }
      }
    return subpaths;
  }    

  FaceIndex MeshIntersection::getFace(MeshFace face)
  {
    if (face.mesh == FIRST)
      {
	return mesh1.F.row(face.index);
      }
    else
      {
	return mesh2.F.row(face.index);
      }    
  }

  std::vector <EdgePoint> MeshIntersection::getEdgePoints(MeshFace face, std::vector <FaceSet> &subpaths)
  {
    std::vector <EdgePoint> edgePoints;

    Segments segments;
    FaceIndex faceIndex = getFace(face);

    Calc::getSegments(faceIndex.transpose(), segments);
	
    EdgePoint edgePoint;
    edgePoint.type = POINT;
    edgePoint.index = faceIndex[0];
    edgePoint.segment = segments.row(0);    
    edgePoints.push_back(edgePoint);
    
    edgePoint.index = faceIndex[1];
    edgePoint.segment = segments.row(1);
    edgePoints.push_back(edgePoint);
    
    edgePoint.index = faceIndex[2];
    edgePoint.segment = segments.row(2);
    edgePoints.push_back(edgePoint);
     
    edgePoint.type = SEGMENT;
    for (FaceSet subpath : subpaths)
      {
	for (int i : subpath)
	  {
	    PointInfo point = points[i].getPoint(face.mesh);

	    if (point.type == PointInfo::SEGMENT)
	      {
		edgePoint.index = i;
		edgePoints.push_back(edgePoint);
	      }
	  }
      }
    return edgePoints;
  }

  bool MeshIntersection::operator ()(EdgePoint first, EdgePoint second)
  {
    SegmentIndex segmentFirst, segmentSecond;
    int pointFirst, pointSecond;
    Vector vectorBase,vectorFirst, vectorSecond;

    if (first.type == SEGMENT)
      {
	segmentFirst = points[first.index].first.index2.block(0,0,2,1);
	pointFirst = segmentFirst[0];
	vectorFirst = points[first.index].point;
      }
    else
      {
	pointFirst = points[first.index].first.index2[0];
	vectorFirst = mesh1.V.row(pointFirst);
      }

    if (second.type == SEGMENT)
      {
	segmentSecond = points[second.index].first.index2.block(0,0,2,1);
	pointSecond = segmentSecond[0];
	vectorSecond = points[second.index].point;
      }
    else
      {
	pointSecond = points[second.index].first.index2[0];
	vectorSecond = mesh1.V.row(pointSecond);
      }

    std::cout << "Compare " << segmentFirst.transpose() << ", " << segmentSecond.transpose() << std::endl;
    if (pointFirst < pointSecond)
      {
	return true;
      }
    else if (pointFirst > pointSecond)
      {
	return false;
      }
    else
      {
	vectorBase = mesh1.V.row(pointFirst);
	
	double d1 = (vectorBase - vectorFirst).norm();
	double d2 = (vectorBase - vectorSecond).norm();

	if (d1 < d2)
	  {
	    return true;
	  }
	else
	  {
	    return false;
	  }
      }
	  
  }
  
  void MeshIntersection::sortEdgePoints(MeshFace face, std::vector <EdgePoint> &edgePoints)
  {

    FaceIndex faceIndex = getFace(face);
    Segments segments;

    Calc::getSegments(faceIndex.transpose(), segments);

    //std::sort (edgePoints.begin(), edgePoints.end(), *this);

    std::cout << "segments " << segments << std::endl;
	    
    for (EdgePoint &ep : edgePoints)
      {
    	SegmentIndex segment;

    	if (ep.type == SEGMENT)
    	  {

	    SegmentIndex seg = points[ep.index].getPoint(face.mesh).index2.block(0,0,2,1);
	    ep.segment = seg;

	    std::cout << "seg " << ep.segment.transpose() << std::endl;
	    Vector start = getMesh(face.mesh).V.row(ep.segment[0]);

	    ep.position = (start - points[ep.index].point).norm();
    	  }
	else
	  {
	    std::cout << "segindex " << ep.index << std::endl;
	    ep.position = 0.0;
	  }
      }

    std::sort (edgePoints.begin(), edgePoints.end());
  }

  Mesh MeshIntersection::getMesh(MeshIndex mesh)
  {
    if (mesh == FIRST)
      {
	return mesh1;
      }
    else
      {
	return mesh2;
      }
  }
  
  Path MeshIntersection::calculatePath(int intersectionPointIndex)
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
	
	pointsset = getPoint(point).connectedPoints;
	
	if (pointsset.size() > 2)
	  {
	    for (int i: pointsset)
	      {
		if (getPoint(i).connectedPoints.size() > 2)
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

  int MeshIntersection::totalPathSize()
  {
    int totalSize = 0;
    for(Path path : paths)
      {
	totalSize += path.size();
      }
    return totalSize;
  }

  int MeshIntersection::getFreePoint()
  {
    for (int i=0; i<points.size(); i++)
      {
	bool freePoint = true;
	for (Path p: paths)
	  {
	    for (int j=0; j<p.size(); j++)
	      {
		if (p[j] == i)
		  {
		    freePoint = false;
		  }
	      }
	  }
	if (freePoint)
	  {
	    return i;
	  }
      }
    return -1;
  }
  
  void MeshIntersection::calculatePaths()
  {
    int intersectionPointIndex = 0;

    do {
      //std::cout << "Intersection point is " << intersectionPointIndex << std::endl;
      //std::cout << "Total path size is " << totalPathSize() << std::endl;
      //std::cout << "Points size is " << points.size() << std::endl;

      paths.push_back(calculatePath(intersectionPointIndex));

      int newIntersectionPointIndex = getFreePoint();
      if (intersectionPointIndex != newIntersectionPointIndex)
	  {
	    intersectionPointIndex = newIntersectionPointIndex;
	  }
      else
	{
	  return;
	}
    } while(totalPathSize() < points.size());
  }

  bool IntersectionPoint::operator==(const IntersectionPoint &ipoint)
  {
    
    if (Calc::vectorEqual(ipoint.point, this->point) && 
	ipoint.first.type == this->first.type &&
	ipoint.first.index2 == this->first.index2 &&
	ipoint.second.index2 == this->second.index2 &&
	ipoint.second.type == this->second.type )
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  void MeshIntersection::printPoints(std::vector <EdgePoint> edgePoints)
  {
    std::cout << "EdgePoints: ";	    
    for (EdgePoint ep : edgePoints)
      {
	if (ep.type == POINT)
	  {
	    std::cout << "P";	    
	  }
	else
	  {
	    std::cout << "S";
	  }
	std::cout << "("<< ep.segment.transpose() << ",";
	std::cout << ep.position << "), ";
      }
    std::cout << std::endl;
  }
  
  void MeshIntersection::printPoints()
  {
    int index = 0;
    for (IntersectionPoint ip : points)
      {
	std::cout << index << " ";
	if (ip.first.type == PointInfo::POINT)
	  {
	    std::cout << "P";
	  }
	else if (ip.first.type == PointInfo::SEGMENT)
	  {
	    std::cout << "S";
	  }
	else if (ip.first.type == PointInfo::FACE)
	  {
	    std::cout << "F";
	  }

	if (ip.second.type == PointInfo::POINT)
	  {
	    std::cout << "P";
	  }
	else if (ip.second.type == PointInfo::SEGMENT)
	  {
	    std::cout << "S";
	  }
	else if (ip.second.type == PointInfo::FACE)
	  {
	    std::cout << "F";
	  }

	std::cout << " (" << ip.point.transpose() << ")" << std::endl;
	
	index++;
      }
  }
}
