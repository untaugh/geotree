#pragma once
#include "Types.h"
#include "EdgePoint.h"
#include "Face.h"
#include "Path.h"
#include <iostream>

namespace Geotree
{
    class FaceEdgePoint : public FacePoint
    {
    public:
        FaceEdgePoint(FacePoint point, double distance) : FacePoint(point), segment(point.segment), position(distance) {};
        FaceEdgePoint(const Mesh &mesh, int index, int segment, double distance) : FacePoint(mesh, index), segment(segment), position(distance) {};
        FaceEdgePoint(const Mesh &mesh, int index) : FacePoint(mesh, index) {};
        int segment;

        double position;

        FaceEdgePoint operator=(const FaceEdgePoint& point)
        {
            this->segment = point.segment;
            this->position = point.position;
            this->index = point.index;

            return FaceEdgePoint(mesh, point.getIndex(), segment, position);
        }

        bool operator <(const FaceEdgePoint point)
        const {
            if (segment < point.segment) {
                return true;
            } else if (segment > point.segment) {
                return false;
            } else {
                if (this->position < point.position) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    };

  class IntersectionFace : public FaceT
  {
  public:
      IntersectionFace(FaceT face);

      void cut(PathX<FacePoint> path);

      void completePaths();

      void triangulate();

    int numberOfPaths() { return completeFacePaths.size() + holePaths.size(); };
    int numberOfEdgePoints() { return faceEdgePoints.size(); };
    int pathSize(int path) { return completeFacePaths[path].size(); };
    PathT getPath(int index) { return completeFacePaths[index]; };
      std::set <int> getArea(int index) { return areas[index]; };


  private:
      PathT completePathForward(const PathX<FacePoint> path);
      PathT completePathReverse(const PathX<FacePoint> path);

      std::vector <FaceEdgePoint> faceEdgePoints;
      std::vector <PathT> completeFacePaths;
      std::vector <PathT> holePaths;
      std::vector <std::set <int>> areas;

      PathX<FacePoint>::const_iterator nextPath(PathT& newPath, PathX<FacePoint>::const_iterator &iter);
      void nextPathReverse(PathT& newPath, PathX<FacePoint>::const_iterator &iter);

      std::vector <PathX<FacePoint>> paths;
  };
}
