#include "Calc.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Log.h"
#include "Validate.h"
#include <math.h>

namespace Calc
{
  void getSegments(const Faces F, Segments &S)
  {
    std::set<SegmentIndex> segments;

    for (int i=0; i < F.rows(); i++)
      {
	for (int j=0; j<3; j++)
	  {
	    SegmentIndex segment;
	    int k = (j+1)%3;
	    
	    if (F.row(i)[j] < F.row(i)[k])
	      {
		segments.insert(SegmentIndex(F.row(i)[j], F.row(i)[k]));
	      }
	    else
	      {
		segments.insert(SegmentIndex(F.row(i)[k], F.row(i)[j]));
	      }
	  }
      }

    S = Segments(segments.size(),2);
    int i = 0;
    for (SegmentIndex si : segments)
      {
	S.row(i++) = si;
      }
  }

  unsigned int sharedSegments(Face F1, Face F2)
  {
    if ( (F1[0] == F2[0] && F1[1] == F2[1] && F1[2] == F2[2]) ||
	 (F1[0] == F2[1] && F1[1] == F2[0] && F1[2] == F2[2]) ||
	 (F1[0] == F2[2] && F1[1] == F2[0] && F1[2] == F2[1]) ||
	 (F1[0] == F2[0] && F1[1] == F2[2] && F1[2] == F2[1]) ||
	 (F1[0] == F2[1] && F1[1] == F2[2] && F1[2] == F2[0]) ||
	 (F1[0] == F2[2] && F1[1] == F2[1] && F1[2] == F2[0]) )
      {
	return 3;
      }
    
    if ( (F1[0] == F2[0] && F1[1] == F2[1]) ||
	 (F1[1] == F2[1] && F1[2] == F2[2]) ||
	 (F1[2] == F2[2] && F1[0] == F2[0]) ||
	 (F1[0] == F2[0] && F1[1] == F2[2]) ||
	 (F1[0] == F2[0] && F1[2] == F2[1]) ||
	 (F1[1] == F2[1] && F1[2] == F2[0]) ||
	 (F1[1] == F2[1] && F1[0] == F2[2]) ||
	 (F1[2] == F2[2] && F1[1] == F2[0]) ||
	 (F1[2] == F2[2] && F1[0] == F2[1]) ||
	 (F1[0] == F2[1] && F1[1] == F2[0]) ||
	 (F1[0] == F2[1] && F1[1] == F2[2]) ||
	 (F1[0] == F2[1] && F1[2] == F2[0]) ||
	 (F1[0] == F2[2] && F1[1] == F2[0]) ||
	 (F1[0] == F2[2] && F1[2] == F2[0]) ||
	 (F1[0] == F2[2] && F1[2] == F2[1]) ||
	 (F1[1] == F2[2] && F1[2] == F2[0]) ||
	 (F1[1] == F2[2] && F1[2] == F2[1]) ||
	 (F1[1] == F2[0] && F1[2] == F2[1]) )
      {
	return 1;
      }

    return 0;
  }

  SegmentIndex sharedSegment(Face face0, Face face1)
  {
    SegmentIndex segment;
    unsigned count = 0;
    
    for (int i=0; i<3; i++)
      {
	for (int j=0; j<3; j++)
	  {	
	    if (face0[i] == face1[j])
	      {
		segment[count++] = face0[i];
	      }
	  }
      }

    return segment;
  }

  void getFaceSegments(const Faces F, MatrixXi &Fo)
  {
    // number of segments
    unsigned int n;
    n = ((F.rows()+1)/2)*3;
    Fo = MatrixXi(n,2);

    //std::cout << "n " << n << std::endl;	  
    
    // segment counter
    unsigned int c;
    c = 0;
    
    for (int i=0; i < F.rows(); i++)
      {
	// j=i no duplicate segments
	for (int j=i; j < F.rows(); j++)
	  {
	    //std::cout << "F " << i << std::endl;	  
	    int num_segments = sharedSegments(F.row(i), F.row(j));

	    if (num_segments == 1)
	      {
		// save shared segment
		Fo.row(c++) << i,j;
	      }
	  }	
      }
  }

  void toSegment(Faces &F, unsigned int f1, unsigned int f2,
		 unsigned int &s1, unsigned int &s2)
  {
    int i;

    //std::cout << "F " << F.rows() << std::endl;
    //std::cout << "f1 " << f1 << std::endl;
    //std::cout << "f2 " << f2 << std::endl;
    //std::cout << "s1 " << s1 << std::endl;
    //std::cout << "s2 " << s2 << std::endl;
    
    for (i=0; i<3; i++)
      {
	if ( F.row(f1)[i] == F.row(f2)[0] ||
	     F.row(f1)[i] == F.row(f2)[1] ||
	     F.row(f1)[i] == F.row(f2)[2] )
	  {
	    // first point found
	    s1 = F.row(f1)[i];
	    break;
	  }	
      }

    // continue searching for second point
    for (i++; i<3; i++)
      {
	if ( F.row(f1)[i] == F.row(f2)[0] ||
	     F.row(f1)[i] == F.row(f2)[1] ||
	     F.row(f1)[i] == F.row(f2)[2] )
	  {
	    s2 = F.row(f1)[i];
	    return;
	  }
	
      }
  }

  bool intersectsFace(const Plane face, const Line segment)
  {
    Vertex normal = Calc::normal(face);

    double D = normal.dot( segment.row(1) - segment.row(0) );
    double N = normal.dot( segment.row(0) - face.row(0) );

    // is segment parallell to plane
    if (D == 0)
      {
	return false;
      }

    double sI = -N/D;
  
    // is segment outside plane
    if (sI < 0.0 || sI > 1.0)
      {
	return false;
      }

    Vertex point = intersection(face, segment);
    
    return inside(face, point);
  }

  bool abovePlane(const Plane plane, Vertex point)
  {
    Vertex normal = Calc::normal(plane);
    Vertex planePoint = plane.row(0);

    double dot = (normal).dot(point - planePoint);
	
    if (dot > 0.0)
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
  bool intersectsPlane(const Plane plane, const Line segment)
  {
    Vertex normal = Calc::normal(plane);

    double D = normal.dot( segment.row(1) - segment.row(0) );
    double N = normal.dot( segment.row(0) - plane.row(0) );

    // is segment parallell to plane
    if (D == 0)
      {
	return false;
      }

    double sI = -N/D;
  
    // is segment outside plane
    if (sI < 0.0 || sI > 1.0)
      {
	return false;
      }

    return true;    
  }

  Vertex intersection(const Plane face, const Line line)
  {
    Vertex normal = Calc::normal(face);

    Vertex lineStart = line.row(0);
    Vertex lineEnd = line.row(1);
    Vertex faceVertex0 = face.row(0);
    
    Vertex U = lineEnd - lineStart;
    Vertex W = lineStart - faceVertex0;

    double D = normal.dot(U);
    double N = normal.dot(W);
    
    Vertex P = lineStart - N/D * U;
	
    return P;
  }

  bool equal(Face f1, Face f2)
  {
    if ( (f1(0) == f2(0) && f1(1) == f2(1) && f1(2) == f2(2)) ||
  	 (f1(0) == f2(0) && f1(2) == f2(2) && f1(1) == f2(1)) ||
  	 (f1(0) == f2(1) && f1(1) == f2(0) && f1(2) == f2(2)) ||
  	 (f1(0) == f2(1) && f1(1) == f2(2) && f1(2) == f2(0)) ||
  	 (f1(0) == f2(2) && f1(1) == f2(0) && f1(2) == f2(1)) ||
  	 (f1(0) == f2(2) && f1(1) == f2(1) && f1(2) == f2(0)) )
      {
  	return true;
      }
    return false;  
  }

  bool triangulate(const Verticies V, const Path P, Faces &F)
  {
    //std::cout << "Triangulate:" << std::endl;

    // skip list
    std::set <int> skip;

    unsigned int pathIndex = P.size();

    Vertex up(0,0,0);

    //while( up == Vertex(0,0,0) && pathIndex > 1)
    while( (up == Vertex(0,0,0) || isnan(up[0]) || isnan(up[1]) || isnan(up[2]) )&& pathIndex > 1)
      {
	pathIndex--;
	Geotree::Log().Get(LOG_DEBUG)
	  << "Normal: "  << V.row(P[0]) << ", " << V.row(P[1]) << ", " << V.row( P[pathIndex] );

	up = normal(V.row(P[0]), V.row(P[1]), V.row( P[pathIndex] ));
      }

    if (pathIndex < 2)
      {
	Geotree::Log().Get(LOG_ERROR)
	  << "Unable to construct normal.";
	return false;
      }
    
    // min axis
    Axis minaxis = minAxis(V,P);
    Axis axis1, axis2;
    
    if (minaxis == AXIS_X)
      {
	axis1 = AXIS_Y;
	axis2 = AXIS_Z;
      }
    else if (minaxis == AXIS_Y)
      {
	axis1 = AXIS_X;
	axis2 = AXIS_Z;
      }
    else
      {
	axis1 = AXIS_X;
	axis2 = AXIS_Y;
      }

    // reference line
    Segment line;
    Point vp0, vp1, vp2, v0, v1;
    vp0 << V.row(P[0])[axis1], V.row(P[0])[axis2];
    vp1 << V.row(P[1])[axis1], V.row(P[1])[axis2];
    vp2 << V.row(P.tail(1)[0])[axis1], V.row(P[pathIndex])[axis2];

    v0 = vp1 - vp0;
    v1 = vp2 - vp0;

    double a2 = angle(v0,v1);

    // winding
    int wind = winding(V,P);

    Geotree::Log().Get(LOG_DEBUG)
      << "Triangulate: "  << "w:" << wind
      << ", a:" << a2
      << ", up:" << up.transpose();
    
    if ( wind <= 0 || a2 < 0 )
      {
	up = -up;
      }
    
    Faces tmp = Faces(P.size(), 3);

    int count = 0;

    int i = 0;
    int c = 0;
    bool done = false;

    int n1, n2, n3;
    Vertex p1,p2,p3;
    
    while (!done)
      {
	//std::cout << "while:" <<  count << std::endl;
	//std::cout << "skip:" <<  skip.size() << std::endl;

	//for (int s : skip)
	{
	    //std::cout << s << ", ";	    
	}
	//std::cout << std::endl;
	if (c++ > P.rows())
	  {
	    Geotree::Log().Get(LOG_DEBUG)
	  << "  failed.";
	    return false;
	  }
	
	if (! Calc::next(P, i, skip))
	  {
	    //std::cout << "1" << std::endl;
	    done = true;
	    //continue;
	  }
	p1 = V.row(P[i]);
	n1 = i++;
	
	if (! Calc::next(P, i, skip))
	  {
	    //std::cout << "2" << std::endl;
	    done = true;
	    //continue;
	  }
	p2 = V.row(P[i]);
	n2 = i++;
	
	if (! Calc::next(P, i, skip))
	  {
	    //std::cout << "3" << std::endl;
	    done = true;
	    //continue;
	  }
	p3 = V.row(P[i]);
	n3 = i;

	//std::cout << "n1 " << n1 << std::endl;
	//std::cout << "n2 " << n2 << std::endl;
	//std::cout << "n3 " << n3 << std::endl;

	if (n1 == n3)
	  {
	    //std::cout << "n1 == n3 " << std::endl;
	    done = true;
	    continue;
	  }
	
	// are any points inside?

	Plane face;
	face.row(0) = p1;
	face.row(1) = p2;
	face.row(2) = p3;

	Verticies selectedVerticies = select(V,P);

	if (pointsInside(face,selectedVerticies))
	{
	    //std::cout << "a point is inside" << std::endl;
	    i = n2;
	    continue;
	  }
	
	double a = angle(p1,p2,p3,up);
	//std::cout << "angle:" << a << std::endl;
	// is angle greater than 180?
	if ( a >= M_PI )
	  {
	    //std::cout << "angle is greater than 180" << std::endl;
	    i = n2;
	    continue;
	  }

	// create triangle
	tmp.row(count++) = Face(P[n1], P[n2], P[n3]);

	//	std::cout << "added:" <<  tmp.row(count-1)<< std::endl;
	    
	// skip point
	skip.insert(n2);

	// reset counter
	c = 0;

	Geotree::Log().Get(LOG_DEBUG)
	  << "  added:" << tmp.row(count-1);
      }

    //std::cout << "count:" << count << std::endl;
    F = tmp.block(0,0,count,3);

    return true;
  }
  
  Vertex normal(Vertex v1, Vertex v2, Vertex v3)
  {
    return (v2 - v1).cross(v3 - v1).normalized();
  }

  Vertex normal(const Plane plane)
  {
    Vertex v0 = plane.row(0);
    Vertex v1 = plane.row(1);
    Vertex v2 = plane.row(2);
    
    return (v1 - v0).cross(v2 - v0).normalized();
  }

  double distance(Vector point, Line segment)
  {
    return distance(point, segment.row(0).transpose(), segment.row(1).transpose());
  }
  
  // distance from point to segment
  double distance(Vertex point, Vertex seg0, Vertex seg1)    
  {
    // zero length
    if (seg0 == seg1)
      {
	return (point - seg0).norm();
      }

    Vertex v = seg1 - seg0;
    Vertex v_p0 = seg0 - point;
    Vertex v_p1 = seg1 - point;

    double d_seg0 = v_p0.dot(v)/v.dot(v);
    double d_seg1 = v_p1.dot(v)/v.dot(v);

    double d = ( v_p0 - (d_seg0 * v) ).norm(); // point to line

    // point to segment end points
    if ( (d_seg0 > 1.0 || d_seg0 < 0.0) &&
	 (d_seg1 > 1.0 || d_seg1 < 0.0) )
      {
	if ( v_p0.norm() < v_p1.norm())
	  {
	    d = v_p0.norm();
	  }
	else
	  {
	    d = v_p1.norm();
	  }
      }

    Geotree::Log().Get(LOG_DEBUG)
      << "distance: "
      << "point:(" << point.transpose() << "), seg0:(" << seg0.transpose()
      << "), seg1:(" << seg1.transpose() << "), d:" << d;
    
    return d;
  }

  double distance(Line segment0, Line segment1)
  {
    return distance(segment0.row(0), segment0.row(1), segment1.row(0), segment1.row(1));
  }
  
  double distance(Vertex v1a, Vertex v1b, Vertex v2a, Vertex v2b)
  {
    // direction of segments
    Vertex v1 = v1a - v1b;
    Vertex v2 = v2a - v2b;

    // vector perpendicular to both lines
    Vertex n = v1.cross(v2);

    // segments are parallell
    float dot  = v1.normalized().dot(v2.normalized());

    // parallel and zero length lines
    if (dot == 1 || dot == -1 || v1a == v1b || v2a == v2b)
      {
	double d, d_tmp;

	d = distance(v1a, v2a, v2b);

	d_tmp = distance(v1b, v2a, v2b);

	if (d_tmp < d)
	  {
	    d = d_tmp;
	  }

	d_tmp = distance(v2a, v1a, v1b);

	if (d_tmp < d)
	  {
	    d = d_tmp;
	  }

	d_tmp = distance(v2b, v1a, v1b);

	if (d_tmp < d)
	  {
	    d = d_tmp;
	  }

	return d;	 
      }
    
    // nearest points
    Vertex n1 = v1.cross(n);
    Vertex n2 = v2.cross(n);
    Vertex p1 =  v1a + ( (v2a - v1a).dot(n2) / v1.dot(n2) ) * v1;
    Vertex p2 =  v2a + ( (v1a - v2a).dot(n1) / v2.dot(n1) ) * v2;
    
    // std::cout << "closest point s1: " << ps1.transpose() << std::endl
    // 	      << "closest point s2: " << ps2.transpose() << std::endl;

    // std::cout << "v2a - ps2: " << (v2a - ps2).transpose() << std::endl
    // 	      << "v2: " << v2.transpose() << std::endl
    // 	      << "dot: " << v2.dot(v2a - ps2) << std::endl
    // 	      << "dot2: " << v2.dot(v2a - ps2) / v2.dot(v2) << std::endl;

    // position of nearest point on segment
    double a1 = v1.dot(v1a - p1) / v1.dot(v1);
    double a2 = v2.dot(v2a - p2) / v2.dot(v2);

    // std::cout << "a1: " << a1 << std::endl
    // 	      << "a2: " << a2 << std::endl;

    //Vertex p1 = ps1;
    //Vertex p2 = ps2;

    // v2a is closest point
    if (a2 < 0.0)
      {
	p2 = v2a;
      }

    // v2b is closest point
    if (a2 > 1.0)
      {
	p2 = v2b;
      }
    
    // v1a is closest point
    if (a1 < 0.0)
      {
	p1 = v1a;
      }

    // v1b is closest point
    if (a1 > 1.0)
      {
	p1 = v1b;
      }

    return (p1 - p2).norm();
    
    // distance
    //double D = abs(p.dot(n)) / sqrt(n.dot(n));

    //return D;
  }
  
  bool inside(const Verticies face, const Vertex point)
  {
    Vertex facePoint0 = face.row(0);
    Vertex facePoint1 = face.row(1);
    Vertex facePoint2 = face.row(2);

    Verticies V(4,3);
    V.row(0) = facePoint0;
    V.row(1) = facePoint1;
    V.row(2) = facePoint2;
    V.row(3) = point;

    if (! Geotree::Validate::planar(V))
      {
	return false;
      }
    
    // normal
    Vertex normal = Calc::normal(face);    

    // Test if point is inside triangle  
    Vertex e1 = facePoint1 - facePoint0;
    Vertex e2 = facePoint2 - facePoint1;
    Vertex e3 = facePoint0 - facePoint2;
    
    Vertex c1 = point - facePoint0;
    Vertex c2 = point - facePoint1;
    Vertex c3 = point - facePoint2;

    double r1 = normal.dot(e1.cross(c1));
    double r2 = normal.dot(e2.cross(c2));
    double r3 = normal.dot(e3.cross(c3));

    // is point outisde triangle
    if (r1 < 0.0 || r2 < 0.0 || r3 < 0.0)
      {
	return false;
      }
    
    return true;
  }

  Verticies select(Verticies verticies, Indicies points)
  {
    Verticies selectedVerticies(points.rows(), 3);

    for (int i=0; i<points.rows(); i++)
      {
	selectedVerticies.row(i) = verticies.row(points[i]);
      }

    return selectedVerticies;
  }
  
  bool pointsInside(Plane face, Verticies points)
  {
    for (int i=0; i<points.rows(); i++)
      {
	Vertex face0 = face.row(0);
	Vertex face1 = face.row(1);
	Vertex face2 = face.row(2);
	Vertex point = points.row(i);

	// don't test triangle points
	if (! (point == face0 || point == face1 || point == face2))
	  {
	    // test if point inside
	    if (inside(face,point))
	      {
		return true;
	      }	    
	  }
	
      }
    return false;    
  }

  double angle(Point p0, Point p1)
  {
    double a0 = atan2(p0[1], p0[0]);
    double a1 = atan2(p1[1], p1[0]);
    double a = a1 - a0;
    
    if (a < -M_PI)
      {
	a += M_PI;
	a = -a;
      }

    if (a > M_PI)
      {
	a -= M_PI;
	a = -a;
      }

    Geotree::Log().Get(LOG_VERBOSE)
      << "angle: a0:" << a0
      << ", a1:" << a1
      << ", a" << a;
    
    return a;
  }
  
  double angle(Vertex v1, Vertex v2, Vertex v3, Vertex up)
  {
    Vertex va = v2 - v1;
    Vertex vb = v2 - v3;

    Vertex n = va.cross(vb);

    double d = va.dot(vb);

    double angle = atan2(n.dot(up), d);

    if (angle < 0.0)
      {
	angle += M_PI * 2;
      }
    
    return angle;
  }

  bool next(VectorXi P, int &i, std::set<int> skip)
  {
    int count = P.size();
    int rows = P.size();

    i = i % rows;

    while (skip.count(i%rows))
      {
	i++;
	
	if (i >= rows)
	  {
	    i = 0;
	  }
	
	if (! count--)
	  {
	    return false;
	  }
      }

    return true;
  }

  // all connected to this point
  static std::set <unsigned> connectedFace(Faces F, std::set <unsigned> skip, Face f)
  {
    std::set <unsigned> list;

    for (int i=0; i<F.rows(); i++)
      {
	if ( !skip.count(i))
	  {
	    for (int j=0; j<3; j++)
	      {
		for (int k=0; k<3; k++)
		  {	       
		    if ( F.row(i)[j] == f[k])
		      {
			list.insert(i);
		      }
		  }
	      }
	  }
      }
    return list;
  }

  std::set <unsigned> connected(const MatrixXi F, std::set <unsigned> skip, const unsigned index)
  {
    std::set <unsigned> list;

    if (skip.count(index))
      {
	return list;
      }
    
    bool done = false;

    list.insert(index);

    while(!done)
      {
	done = true;
	for (unsigned i : list)
	  {
	    if (! skip.count(i))
	      {
		std::set <unsigned> ret;
		done = false;
		skip.insert(i);
		ret = connectedFace(F, skip, F.row(i));
		list.insert(ret.begin(), ret.end());
	      }    
	  }
      }

    return list;
  }

  bool hasPoint(Face face, unsigned index)
  {
    if (face[0] == index || face[1] == index || face[2] == index)
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
  unsigned sharedPoints(Face face0, Face face1)
  {
    unsigned count = 0;

    for (int i=0; i<3; i++)
      {
	if (hasPoint(face0, face1[i]))
	  {
	    count++;
	  }
      }
    return count;
  }
  
  bool connectedPoint(Faces F, FaceSet f1, FaceSet f2)
  {
    for (unsigned i : f1)
      {
	for (unsigned j : f2)
	  {
	    if (sharedPoints(F.row(i), F.row(j)) > 0)
	      return true;
	  }
      }
    return false;    
  }
  
  bool connected(Faces F, FaceSet f1, FaceSet f2)
  {
    std::cout << "f1: ";
    for (int f : f1)
      {
	std::cout << f << ", ";
      }
    std::cout << std::endl;;

    std::cout << "f2: ";
    for (int f : f2)
      {
	std::cout << f << ", ";
      }
    std::cout << std::endl;;
    
    for (unsigned i : f1)
      {
	for (unsigned j : f2)
	  {	    
	    int shared = sharedSegments(F.row(i), F.row(j));
	    
	    if ( shared == 1)
	      {
		std::cout << "Shared segments " << shared << std::endl;
		return true;
	      }
	  }
      }

    std::cout << "Not Connected" << std::endl;
    return false;
  }

  Verticies boundingBox(const Geometry G)
  {
    Vertex A, B;
    Path P(G.F.size());

    for (int i=0; i<G.F.size(); i++)
      {
	P[i] = G.F.row(i/3)[i%3];
      }
      
    boundingBox(G.V, P, A, B);

    Verticies Box(2,3);

    Box.row(0) = A;
    Box.row(1) = B;
    
    return Box;
  }
  
  //void boundingBox(Verticies V, Faces F, Vertex &B1, Vertex &B2)
  void boundingBox(Verticies V, Path F, Vertex &B1, Vertex &B2)
  {
    // initial values
    B1 = V.row(F.row(0)[0]);
    B2 = V.row(F.row(0)[0]);

    for (int i=0; i<F.rows(); i++)
      {
	// verticies in face
	for (int j=0; j<F.cols(); j++)
	  {
	    int p = F.row(i)[j];
	    
	    for (int k=0; k<3; k++)
	      {
		B1[k] = std::min(B1[k], V.row(p)[k]);
		B2[k] = std::max(B2[k], V.row(p)[k]);
	      }
	  }
      }
  }
  
  int winding(Verticies V, Path P)
  {
    // skip one axis
    Axis minaxis = minAxis(V,P);
    Axis axis1, axis2;
    
    if (minaxis == AXIS_X)
      {
	axis1 = AXIS_Y;
	axis2 = AXIS_Z;
      }
    else if (minaxis == AXIS_Y)
      {
	axis1 = AXIS_X;
	axis2 = AXIS_Z;
      }
    else
      {
	axis1 = AXIS_X;
	axis2 = AXIS_Y;
      }

    // reference line
    Segment line;

    line << V.row(P[0])[axis1], V.row(P[0])[axis2],
      V.row(P[1])[axis1], V.row(P[1])[axis2];
    
    // calculate winding
    Segment segment;
    int winding = 0;
    for (int i=1; i<(P.size() - 1); i++)
      {
	segment << V.row(P[i])[axis1], V.row(P[i])[axis2],
	  V.row(P[i+1])[axis1], V.row(P[i+1])[axis2];
	
	intersect(line, segment, winding);
      }
    return winding;
  }

  bool intersect(const Segment line, const Segment segment, int &winding)
  {    
    bool retval = false;
    Point v1, v1_r, v2;

    v1 = line.row(0) - line.row(1);
    
    v2 = segment.row(0) - segment.row(1);
    
    if ((v1[0] < 0))
      {
	v1_r << v1[1], -v1[0];
      }
    else
      {
	v1_r = v1;
	v1 << v1_r[1], -v1_r[0];
      }
    
    double y1 = line.row(0)[1] + v1[1]/v1[0] * (segment.row(0)[0] - line.row(0)[0]);
    double y2 = line.row(0)[1] + v1[1]/v1[0] * (segment.row(1)[0] - line.row(0)[0]);

    double x1 = line.row(0)[0] + v1_r[0]/v1_r[1] * (segment.row(0)[1] - line.row(0)[1]);
    double x2 = line.row(0)[0] + v1_r[0]/v1_r[1] * (segment.row(1)[1] - line.row(0)[1]);

    double x = segment.row(0)[0] + v2[0]/v2[1] * (line.row(0)[1] - segment.row(0)[1]);
    double y = segment.row(0)[1] + v2[1]/v2[0] * (line.row(0)[0] - segment.row(0)[0]);

    Geotree::Log().Get(LOG_DEBUG)
      << "Intersect: "
      << "y1:" << y1 << ", y2:" << y2
      << ", x1:" << x1 << ", x2:" << x2
      << ", x:" << x << ", y:" << y
      << ", w:" << winding;
    
    if (line.row(0) == segment.row(0))
      {
	return false;
      }    

    if ( (segment.row(0)[1] < y1) != (segment.row(1)[1] < y2) && (x > line.row(0)[0]) )
      {
    	if (segment.row(1)[1] < y2)
    	  {
    	    winding++;
    	  }
    	else
    	  {
    	    winding--;
    	  }
    	retval = true;
      }
    else if ( (segment.row(0)[1] > y1) != (segment.row(1)[1] > y2) && (x < line.row(0)[0]) )
      {
    	if (segment.row(1)[1] > y2)
    	  {
    	    winding++;
    	  }
    	else
    	  {
    	    winding--;
    	  }
    	retval = true;
      }

    if ( (segment.row(0)[0] < x1) != (segment.row(1)[0] < x2) && (y < line.row(0)[1]) )
      {
    	if (segment.row(1)[0] < x2)
    	  {
    	    winding++;
    	  }
    	else
    	  {
    	    winding--;
    	  }
    	retval = true;
      }
    else if ( (segment.row(0)[0] > x1) != (segment.row(1)[0] > x2) && (y > line.row(0)[1]) )
      {
    	if (segment.row(1)[0] > x2)
    	  {
    	    winding++;
    	  }
    	else
    	  {
    	    winding--;
    	  }
    	retval = true;
      }
        
    return retval;
  }

  Axis minAxis(const Verticies V, const Path P)
  {
    Vertex B1, B2;
    boundingBox(V, P, B1, B2);

    double size_x = abs(B1[0] - B2[0]);
    double size_y = abs(B1[1] - B2[1]);
    double size_z = abs(B1[2] - B2[2]);

    if (size_x < size_y)
      {
	if (size_x < size_z)
	  {
	    return AXIS_X;
	  }
	else
	  {
	    return AXIS_Z;
	  }
      }
    else
      {
	if (size_y < size_z)
	  {
	    return AXIS_Y;
	  }
	else
	  {
	    return AXIS_Z;
	  }
      }
  }

  bool inside(const Geometry G, const Vertex point)
  {
    // Get bounding box
    Verticies Box = boundingBox(G);

    Vertex box_diag = Box.row(0) - Box.row(1);
    
    int count = 0;

    Verticies S(2,3);
    S.row(0) = point;
    S.row(1) = point + box_diag;

    Vertex P_prev(-1,-1,-1);

    std::set <int> visitedFaces;
    
    // Test intersection from point to outside of box
    for (int i=0; i<G.F.rows(); i++)
      {
	Plane F;
	F.row(0) = G.V.row(G.F.row(i)[0]);
	F.row(1) = G.V.row(G.F.row(i)[1]);
	F.row(2) = G.V.row(G.F.row(i)[2]);

	if (inside(F, point ))
	  {
	    return true;
	  }

	if (intersectsFace(F,S))
	  {
	    Vertex P = intersection(F,S);

	    if (Calc::hasPoint(G.V, P))
	      {
		int pointIndex = Calc::pointIndex(G.V, P);
		if (intersect(G, pointIndex, point))
		  {
		    count++;
		  }		    
	      }
	    else if (atEdge(F, P))
	      {
		if (! visitedFaces.count(i))
		  {
		    SegmentIndex segment = getSegment(G, i, P);

		    Vector2i facesMatch = getFaces(G, segment);

		    Line line;

		    line.row(1) = P;
		    line.row(0) = point;
		    

		    visitedFaces.insert(facesMatch[0]);
		    visitedFaces.insert(facesMatch[1]);
				

		    if (intersect(G, facesMatch[0], facesMatch[1], line))
		      {
			count++;
		      }

		  }
	      }
	    else if (P != P_prev)
	      {
		count++;
	      }
	    else
	      {

	      }
	    P_prev = P;
	  }
      }

    Geotree::Log().Get(LOG_DEBUG)
      << "inside() point: " << point.transpose() << ", count: " << count;

    if (count % 2)
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  // get point that is not
  int getNotPoint(Face F, int notPoint)
  {
    if (F[0] != notPoint)
      {
	return F[0];
      }
    else if (F[1] != notPoint)
      {
	return F[1];
      }
    else if (F[2] != notPoint)
      {
	return F[2];
      }
    else
      {
	return -1;
      }
  }

  int getNotNotPoint(Face F, int notPoint0, int notPoint1)
  {
    if (F[0] != notPoint0 && F[0] != notPoint1)
      {	
	return F[0];
      }
    else if (F[1] != notPoint0 && F[1] != notPoint1)
      {
	return F[1];
      }
    else if (F[2] != notPoint0 && F[2] != notPoint1)
      {
	return F[2];
      }
    else
      {
	return -1;
      }
  }
  
  bool hasPoint(Face F, int index)
  {
    if (F[0] == index || F[1] == index || F[2] == index)
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
  // face with 
  int findPoint(const Faces F, const unsigned p0, const unsigned p1, const unsigned p2_not)
  {
    for (int i=0; i<F.rows(); i++)
      {
	if ( hasPoint(F.row(i), p0) &&
	     hasPoint(F.row(i), p1) &&
	     ! hasPoint(F.row(i), p2_not) )
	  {
	    return getNotNotPoint(F.row(i), p0, p1);
	  }
      }

    Geotree::Log().Get(LOG_DEBUG) << "findPoint: No Point found!";

    return -1;    
  }

  bool intersect(const Geometry geometry, const int face0, const int face1, const Line lineToSegment)
  {
    // vector from intersecting point parallell to face0
    Face f0 = geometry.F.row(face0);

    SegmentIndex segment = sharedSegment(geometry.F.row(face0), geometry.F.row(face1));

    int face0Point = getNotNotPoint(geometry.F.row(face0), segment[0], segment[1]);
    Vertex face0Vertex = geometry.V.row(face0Point) - lineToSegment.row(1);
    
    // vector from intersecting point parallell to face1
    int face1Point = getNotNotPoint(geometry.F.row(face1), segment[0], segment[1]);
    Vertex face1Vertex = geometry.V.row(face1Point) - lineToSegment.row(1);

    // vector from nitersecting point parallell to line
    Vertex lineVertex  = lineToSegment.row(0) - lineToSegment.row(1);

    Plane plane;
    plane.row(0) = geometry.V.row(segment[0]);
    plane.row(1) = geometry.V.row(face0Point);
    plane.row(2) = geometry.V.row(face1Point);

    
    Vertex normal = Calc::normal(plane);

    Vertex v0 = lineToSegment.row(1);
    Vertex v1 = lineToSegment.row(0);
    Vertex v2 = geometry.V.row(face0Point);
    Vertex v3 = geometry.V.row(face1Point);
    double a0 = angle(v2, v0, v1, normal);
    double a1 = angle(v2, v0, v3, normal);
    
    if (a0 < a1)
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
  bool intersect(const Geometry G, const unsigned point, const Vertex segment)
  {
    // Get faces with point
    std::vector <unsigned> F;

    for (int i=0; i<G.F.rows(); i++)
      {
	if (G.F.row(i)[0] == point ||
	    G.F.row(i)[1] == point ||
	    G.F.row(i)[2] == point)
	  {
	    F.push_back(i);
	  }
      }
   
    // Create plane parallell to segment
    int index = getNotPoint(G.F.row(F.front()), point);
    
    Plane plane;
    plane.row(0) = G.V.row(point);
    plane.row(1) = segment;
    plane.row(2) = G.V.row(index);
    
    Vertex normal = Calc::normal(plane);

    Plane plane_orth;
    plane_orth.row(0) = G.V.row(point);
    plane_orth.row(1) = segment;
    plane_orth.row(2) = G.V.row(point) + normal.transpose();

    int segmentIndex0, segmentIndex1, segmentIndex0_prev;

    segmentIndex0 = index;

    segmentIndex1 = getNotNotPoint(G.F.row(F.front()), point, index);

    int intersectionCountLeft = 0;
    
    for (int i=0; i<F.size(); i++)
      {
	Vertex P;

	Line segment;
	segment.row(1) = G.V.row(segmentIndex0);
	segment.row(0) = G.V.row(segmentIndex1);

	if (abovePlane(plane, G.V.row(segmentIndex0)) != abovePlane(plane, G.V.row(segmentIndex1)))
	  {
	    Vertex intersectionPoint = intersection(plane, segment);

	    if (abovePlane(plane_orth, intersectionPoint ))
	      {
		intersectionCountLeft++;
	      }
	  }
	
	segmentIndex0_prev = segmentIndex0;
	segmentIndex0 = segmentIndex1;
	segmentIndex1 = findPoint(G.F, point, segmentIndex1, segmentIndex0_prev);
      }

    Geotree::Log().Get(LOG_DEBUG) << "intersect() IntersectionCountLeft: " << intersectionCountLeft;

    if (intersectionCountLeft % 2)
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  bool hasPoint(Verticies verticies, Vertex point)
  {
    for (int i=0; i<verticies.rows(); i++)
      {
	Vertex current = verticies.row(i);
	if (current == point)
	  {
	    return true;
	  }
      }
    return false;
  }

  int pointIndex(Verticies verticies, Vertex point)
  {
    for (int i=0; i<verticies.rows(); i++)
      {
	Vertex current = verticies.row(i);
	if (current == point)
	  {
	    return i;
	  }
      }
    return -1;
  }

  bool atSegment(Line segment, Vertex point)
  {
    Vertex toSegment = segment.row(1).transpose() - segment.row(0).transpose();
    Vertex toPoint = point - segment.row(0).transpose();

    double dot = toSegment.normalized().dot(toPoint.normalized());

    // very close to 1.0
    if (dot >= 0.99999999999999)
      {
	if (toSegment.norm() >= toPoint.norm())
	  {
	    return true;
	  }
      }

    return false;
  }
  
  bool atEdge(Plane face, Vertex point)
  {
    Line segment0, segment1, segment2;
    
    segment0.row(0) = face.row(0);
    segment0.row(1) = face.row(1);
    segment1.row(0) = face.row(1);
    segment1.row(1) = face.row(2);
    segment2.row(0) = face.row(2);
    segment2.row(1) = face.row(0);
    
    if (atSegment(segment0, point))
      {
	return true;
      }
    if (atSegment(segment1, point))
      {
	return true;
      }
    if (atSegment(segment2, point))
      {
	return true;
      }    

    return false;

  }

  SegmentIndex getSegment(const Geometry geometry, const int faceIndex, const Vertex point)
  {
    SegmentIndex segment(0,0);

    Face face = geometry.F.row(faceIndex);

    Plane triangle;
    triangle.row(0) = geometry.V.row(face[0]);
    triangle.row(1) = geometry.V.row(face[1]);
    triangle.row(2) = geometry.V.row(face[2]);

    if (atEdge(triangle, point))
      {
	Line segment0, segment1, segment2;
	
	segment0.row(0) = triangle.row(0);
	segment0.row(1) = triangle.row(1);
	segment1.row(0) = triangle.row(1);
	segment1.row(1) = triangle.row(2);
	segment2.row(0) = triangle.row(2);
	segment2.row(1) = triangle.row(0);
    
	if (atSegment(segment0, point))
	  {
	    segment[0] = face[0];
	    segment[1] = face[1];	    
	  }
	else if (atSegment(segment1, point))
	  {
	    segment[0] = face[1];
	    segment[1] = face[2];	    
	  }
	else if (atSegment(segment2, point))
	  {
	    segment[0] = face[2];
	    segment[1] = face[0];
	  }	
      }

    return segment;    
  }

  bool hasSegment(Face face, SegmentIndex segment)
  {
    for (int i=0; i<3; i++)
      {
	for (int j=0; j<3; j++)
	  {
	    if (face[i] == segment[0] && face[j] == segment[1])
	      {
		return true;
	      }
	  }
      }
    return false;
  }
  
  Vector2i getFaces(const Geometry geometry, SegmentIndex segment)
  {
    Vector2i facesMatch;
    
    int count = 0;
    
    for (int i=0; i<geometry.F.rows(); i++)
      {	
	Face face = geometry.F.row(i);
	if (hasSegment(face, segment))
	  {
	    facesMatch[count++] = i;
	  }
      }

    return facesMatch;
  }

  bool closeToZero(double value)
  {
    return (value < 1.0e-14 && value > -1.0e-14);
  }

  bool isEndPoint(Line segment, Vector point)
  {
    if (point == segment.row(0).transpose() ||
	point == segment.row(1).transpose())
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  bool isEndPoint(Plane face, Vector point)
  {
    if (Calc::closeToZero((point - face.row(0).transpose()).norm()) ||
    	Calc::closeToZero((point - face.row(1).transpose()).norm()) ||
    	Calc::closeToZero((point - face.row(2).transpose()).norm()) )
      {
    	return true;
      }
    else
      {
	return false;
      }    
  }

  bool equal (double d1, double d2)
  {
    long lv1 = *(long*) &d1;
    long lv2 = *(long*) &d2;

    if (lv1 == (lv2 + 1) || lv1 == (lv2 - 1) || lv1 == lv2 || lv1 == (lv2 + 2) || lv1 == (lv2 - 2))
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
  bool vectorEqual(Vector v1, Vector v2)
  {
    if (equal(v1(0), v2(0)) && equal(v1(1), v2(1)) && equal(v1(2), v2(2)))
      {
	return true;
      }
    else
      {
	return false;
      }
  }

    bool contains(FaceSet first, FaceSet second)
    {
      for (int i: first)
	{
	  if (second.count(i))
	    {
	      return true;
	    }
	}
      return false;
    }
}

