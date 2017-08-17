#include "Calc.h"
#include <iostream>
#include <Eigen/Dense>
#include "Log.h"

namespace Calc
{
  void getSegments(Faces &F, Faces &S)
  {
    S = Faces(F.rows()*3,2);
    
    for (int i=0; i < F.rows(); i++)
      {
	S.row(0+i*3)[0] = F.row(i)[0];
	S.row(0+i*3)[1] = F.row(i)[1];
	S.row(1+i*3)[0] = F.row(i)[1];
	S.row(1+i*3)[1] = F.row(i)[2];
	S.row(2+i*3)[0] = F.row(i)[2];
	S.row(2+i*3)[1] = F.row(i)[0];
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
  
  void getFaceSegments(Faces &F, Faces &Fo)
  {
    // number of segments
    unsigned int n;
    n = ((F.rows()+1)/2)*3;
    Fo = Faces(n,2);

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

  bool getIntersection(Verticies F, Verticies segment, Vertex &point)
  {
    // verticies of face
    Vertex p1 = F.row(0);
    Vertex p2 = F.row(1);
    Vertex p3 = F.row(2);

    // verticies of segment
    Vertex s1 = segment.row(0);
    Vertex s2 = segment.row(1);
    
    // normal
    Vertex n = Calc::normal(p1, p2, p3);

    // calculate u and w
    Vertex u = s2 - s1;
    Vertex w = s1 - p1;

    // calculate D and N
    float D = n.dot(u);
    float N = -n.dot(w);

    // is segment parallell to plane
    if (D == 0)
      {
	return false;
      }

    float sI = N/D;
    
    //std::cout << "sI " << sI << std::endl;
  
    // is segment outside plane
    if (sI < 0.0 || sI > 1.0)
      {
	return false;
      }

    // calulate point of inersection
    point = s1 + sI * u;
    
    // is point outisde triangle
    if(inside(p1, p2, p3, point))
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
  bool getIntersection(Verticies &V1, Faces &F1,
		       Verticies &V2, Faces &F2,
		       unsigned int f1,
		       unsigned int f2a, unsigned int f2b,
		       Vertex &p)
  {
    Face f = F1.row(f1);
    Vertex p1 = V1.row(f[0]);
    Vertex p2 = V1.row(f[1]);
    Vertex p3 = V1.row(f[2]);

    // face indicies to segment verticies
    unsigned int n1,n2;
    toSegment(F2, f2a, f2b, n1, n2);
    Vertex s1 = V2.row(n1);
    Vertex s2 = V2.row(n2);

    // normal
    Vertex n = Calc::normal(p1,p2,p3);

    // calculate u and w
    Vertex u = s2 - s1;
    Vertex w = s1 - p1;

    // calculate D and N
    float D = n.dot(u);
    float N = -n.dot(w);

    // is segment parallell to plane
    if (D == 0)
      {
	return false;
      }
  
    float sI = N/D;
    
    //std::cout << "sI " << sI << std::endl;
  
    // is segment outside plane
    if (sI < 0.0 || sI > 1.0)
      {
	return false;
      }

    // calulate point of inersection
    p = s1 + sI * u;
    
    // is point outisde triangle
    if(inside(p1, p2, p3, p))
      {
	return true;
      }
    else
      {
	return false;
      }
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

    Vertex up = normal(V.row(P[0]), V.row(P[1]), V.row( P.tail(1)[0] ));

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
    vp2 << V.row(P.tail(1)[0])[axis1], V.row(P.tail(1)[0])[axis2];

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
	if ( inside(p1, p2, p3, V, P) )
	  {
	    //std::cout << "a point is inside" << std::endl;
	    i = n2;
	    continue;
	  }
	
	double a = angle(p1,p2,p3,up);
	std::cout << "angle:" << a << std::endl;
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
    Vertex va = v2 - v1;
    Vertex vb = v3 - v1;

    return va.cross(vb).normalized();

    //std::cout << n.transpose() << std::endl;
	
    //return n;
  }

  double distance(Vertex v1a, Vertex v1b, Vertex v2a, Vertex v2b)
  { 
    // direction of segments
    Vertex v1 = v1a - v1b;
    Vertex v2 = v2a - v2b;

    // vector perpendicular to both lines
    Vertex n = v1.cross(v2);

    // segments are parallell
    if (n.norm() == 0.0)
      {
	Vertex v3 = v1a - v2a; // line 1 to 2
	Vertex v4 = v1.normalized() * v3.dot(v1.normalized()); // project v3 to line
	//Vertex v5 = v1a + v4;
	//std::cout << v3 << ", " << v4 << std::endl;
	if (v1.norm() < v4.norm())
	  {
	    return 2.0;
	  }
	
	return (v4 - v2a).norm();
	//return -1;
      }
    
    // nearest points
    Vertex n2 = v2.cross(n);
    Vertex n1 = v1.cross(n);
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
  
  bool inside(Vertex v1, Vertex v2, Vertex v3, Vertex p)
  {
    // normal
    Vertex n = Calc::normal(v1,v2,v3);

    // Test if point is inside triangle  
    Vertex e1 = v2 - v1;
    Vertex e2 = v3 - v2;
    Vertex e3 = v1 - v3;
    
    Vertex c1 = p - v1;
    Vertex c2 = p - v2;
    Vertex c3 = p - v3;

    double r1 = n.dot(e1.cross(c1));
    double r2 = n.dot(e2.cross(c2));
    double r3 = n.dot(e3.cross(c3));

    // is point outisde triangle
    if (r1 < 0.0 || r2 < 0.0 || r3 < 0.0)
      {
	return false;
      }
    
    return true;
  }

  bool inside(Vertex v1, Vertex v2, Vertex v3, Verticies V, VectorXi P)
  {
    Verticies V2(P.rows(), 3);

    for (int i=0; i<P.rows(); i++)
      {
	V2.row(i) = V.row(P[i]);
      }

    return inside(v1, v2, v3, V2);
  }
  
  bool inside(Vertex v1, Vertex v2, Vertex v3, Verticies P)
  {
    // iterate points
    for (int i=0; i<P.rows(); i++)
      {
	Vertex p = P.row(i);

	// don't test triangle points
	if (! (p == v1 || p == v2 || p == v3))
	  {
	    // test if point inside
	    if (inside(v1,v2,v3,p))
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

    Geotree::Log().Get(LOG_DEBUG)
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
  static std::set <int> connectedFace(Faces F, std::set <int> skip, Face f)
  {
    std::set <int> list;

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
      
  std::set <int> connected(Faces F, std::set <int> skip, int index)
  {
    std::set <int> list, ret;

    bool done = false;
	    
    list.insert(index);

    while(!done)
      {
	done = true;
	for (int i : list)
	  {
	    if (! skip.count(i))
	      {
		done = false;
		skip.insert(i);
		ret = connectedFace(F, skip, F.row(i));
		list.insert(ret.begin(), ret.end());
	      }    
	  }
      }

    return list;
  }
  
  void boundingBox(Verticies V, Faces F, Vertex &B1, Vertex &B2)
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

  Axis minAxis(const Verticies V, const Faces P)
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
}
