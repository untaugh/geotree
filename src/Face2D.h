#pragma once
#include <iostream>
#include <earcut.hpp>
#include "Face.h"
#include "Zero.h"
#include "Log.h"

using namespace Eigen;

namespace Geotree
{
  struct Point2D
  {
    double x;
    double y;

    uint32_t quadrant(Point2D &point)
    {
      if (point.y > this->y)
      {
	if (point.x > this->x)
	{
	  return 0;
	}
	else
	{
	  return 1;
	}
      }
      else
      {
	if (point.x > this->x)
	{
	  return 3;
	}
	else
	{
	  return 2;
	}
      }
    }

    bool operator != (const Point2D &point) const
    {
      return !(*this == point);
    }

    bool operator == (const Point2D &point) const
    {
      return NEAREQUAL(this->x, point.x) && NEAREQUAL(this->y, point.y);
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

    bool contains(Point2D &point)
    {
      int32_t wi = winding(point);	 
      return wi > 3 || wi <-2;
    }

    int32_t winding(Point2D &point)
    {
      int32_t winding = 0;
      uint32_t qa = points[0].quadrant(point);

      for (Point2D p : points)
      {
	uint32_t newqa = p.quadrant(point);

	winding += newqa - qa;
	Log().debug() << "Winding " << winding;
	qa = newqa;
      }

      //Log().debug() << "Winding " << winding;

      return winding;
    }
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

    std::vector <std::vector<uint32_t>> split(std::vector<Path2D> paths);
    
  private:
    void findpath(EdgePoint2D edgepoint, std::vector<Path2D> paths, int &path, bool &reverse);
    void createloops(int startpoint, std::vector <EdgePoint2D> edgepoints, std::vector<Path2D> paths, std::vector<Loop2D> &loops);
    void createholes(std::vector<Path2D> paths, std::vector<Loop2D> &loops);
    std::vector<uint32_t> triangulate(Loop2D loop, std::vector<Loop2D> &holes);
    std::vector<uint32_t> triangulate(Loop2D loop);
    uint32_t getIndex(Point2D point, std::vector<Path2D> paths);
    std::vector <EdgePoint2D> getEdgepoints(std::vector<Path2D> paths);
    bool contains(Loop2D loopA, Loop2D loopB);
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
