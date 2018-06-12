#pragma once
#include <iostream>
#include <earcut.hpp>
#include "Face.h"

using namespace Eigen;

namespace Geotree
{
  struct Point2D
  {
    double x;
    double y;

    bool operator != (const Point2D &point) const
    {
      return this->x != point.x || this->y != point.y;
    }

    bool operator == (const Point2D &point) const
    {
      return this->x == point.x && this->y == point.y;
    }

    friend std::ostream& operator<< (std::ostream& stream, const Point2D& point)
    {
      stream << "(" << point.x << "," << point.y << ")";
      return stream;
    }
  };
  
  struct EdgePoint2D
  {
    int edge;
    double distance;
    Point2D *point;

    bool operator < (const EdgePoint2D &point) const
    {
      if (this->edge < point.edge)
      {
	return true;
      }

      if (this->edge > point.edge)
      {
	return false;
      }

      if (this->distance < point.distance)
      {
	return true;
      }
      else
      {
	return false;
      }
    }

    friend std::ostream& operator<< (std::ostream& stream, const EdgePoint2D& edgepoint)
    {
      stream << *edgepoint.point;
      return stream;
    }
  };
  
  struct Path2D
  {
    std::vector <Point2D> points;
    bool edgetoedge;
    EdgePoint2D begin;
    EdgePoint2D end;
  };

  struct Loop2D
  {
    std::vector <Point2D> points;
  };

  class Face2D
  {
  public:
    Face2D(Face face);
    Face2D() : dropaxis(Z) {};
    const Axis dropaxis;

    Point2D p0;
    Point2D p1;
    Point2D p2;

    //std::vector <std::vector<uint32_t>> split(Path2D path);
    std::vector <std::vector<uint32_t>> split(std::vector<Path2D> paths);
    
  private:
    void findpath(EdgePoint2D edgepoint, std::vector<Path2D> paths, int &path, bool &reverse);
    void createloops(int startpoint, std::vector <EdgePoint2D> edgepoints, std::vector<Path2D> paths, std::vector<Loop2D> &loops);
    std::vector<uint32_t> triangulate(Loop2D loop);
    //uint32_t getIndex(Point2D point, Path2D path);
    uint32_t getIndex(Point2D point, std::vector<Path2D> paths);
    std::vector <EdgePoint2D> getEdgepoints(std::vector<Path2D> paths);
  };
}

namespace mapbox {
  namespace util {

    template <>
    struct nth<0, Geotree::Point2D> {
      inline static auto get(const Geotree::Point2D &t) {
	return t.x;
      };
    };
    template <>
    struct nth<1, Geotree::Point2D> {
      inline static auto get(const Geotree::Point2D &t) {
	return t.y;
      };
    };
  }
}
