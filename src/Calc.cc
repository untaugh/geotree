#include "Calc.h"
#include <iostream>

namespace Calc
{
  void getSegments(MatrixXi &F, MatrixXi &S)
  {
    S = MatrixXi(F.rows()*3,2);
    
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

  unsigned int sharedSegments(Vector3i F1, Vector3i F2)
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
  
  void getFaceSegments(MatrixXi &F, MatrixXi &Fo)
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

  void toSegment(MatrixXi &F, unsigned int f1, unsigned int f2,
		 unsigned int &s1, unsigned int &s2)
  {
    int i;
    
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

  bool getIntersection(MatrixXd &V1, MatrixXi &F1,
		       MatrixXd &V2, MatrixXi &F2,
		       unsigned int f1,
		       unsigned int f2a, unsigned int f2b,
		       Vector3d &p)
  {
    Vector3i f = F1.row(f1);
    Vector3d p1 = V1.row(f[0]);
    Vector3d p2 = V1.row(f[1]);
    Vector3d p3 = V1.row(f[2]);

    // face indicies to segment verticies
    unsigned int n1,n2;
    toSegment(F2, f2a, f2b, n1, n2);
    Vector3d s1 = V2.row(n1);
    Vector3d s2 = V2.row(n2);

    // normal
    Vector3d n = Calc::normal(p1,p2,p3);

    // calculate u and w
    Vector3d u = s2 - s1;
    Vector3d w = s1 - p1;

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

  bool equal(Vector3i f1, Vector3i f2)
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

  void triangulate(const MatrixXd P, MatrixXi &F)
  {

      for (int i=0; i<P.rows()-2; i++)
	{
	  Eigen::Vector3d v1 = P.row(i) - P.row(i+1);
	  Eigen::Vector3d v2 = P.row(i+2) - P.row(i+1);
	  float d = v1.dot(v2);
	  std::cout << "v1: " << v1.transpose()
		    << " v2: " << v2.transpose()
		    << " dot: " << d << std::endl;
	}
      
      F = Eigen::MatrixXi(2,3); 
  }
  
  Vector3d normal(Vector3d v1, Vector3d v2, Vector3d v3)
  {
    Vector3d va = v2 - v1;
    Vector3d vb = v3 - v1;

    Vector3d n = va.cross(vb);

    n.normalize();

    //std::cout << n.transpose() << std::endl;
	
    return n;
  }

  double distance(Vector3d v1a, Vector3d v1b, Vector3d v2a, Vector3d v2b)
  {
    // direction of segments
    Vector3d v1 = v1a - v1b;
    Vector3d v2 = v2a - v2b;

    // vector perpendicular to both lines
    Vector3d n = v1.cross(v2);

    // nearest points
    Vector3d n2 = v2.cross(n);
    Vector3d n1 = v1.cross(n);
    Vector3d p1 =  v1a + ( (v2a - v1a).dot(n2) / v1.dot(n2) ) * v1;
    Vector3d p2 =  v2a + ( (v1a - v2a).dot(n1) / v2.dot(n1) ) * v2;
    
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

    //Vector3d p1 = ps1;
    //Vector3d p2 = ps2;

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

  
  bool inside(Vector3d v1, Vector3d v2, Vector3d v3, Vector3d p)
  {
    // normal
    Vector3d n = Calc::normal(v1,v2,v3);

    // Test if point is inside triangle  
    Vector3d e1 = v2 - v1;
    Vector3d e2 = v3 - v2;
    Vector3d e3 = v1 - v3;
    
    Vector3d c1 = p - v1;
    Vector3d c2 = p - v2;
    Vector3d c3 = p - v3;

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

  double angle(Vector3d v1, Vector3d v2, Vector3d v3, Vector3d up)
  {
    Vector3d va = v2 - v1;
    Vector3d vb = v2 - v3;

    Vector3d n = va.cross(vb);

    double d = va.dot(vb);

    double angle = atan2(n.dot(up), d);

    if (angle < 0.0)
      {
	angle += M_PI * 2;
      }
    
    return angle;
  }
}
