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
    if ( F1[0] == F2[0] && F1[1] == F2[1] && F1[2] == F2[2] ||
	 F1[0] == F2[1] && F1[1] == F2[0] && F1[2] == F2[2] ||
	 F1[0] == F2[2] && F1[1] == F2[0] && F1[2] == F2[1] ||
	 F1[0] == F2[0] && F1[1] == F2[2] && F1[2] == F2[1] ||
	 F1[0] == F2[1] && F1[1] == F2[2] && F1[2] == F2[0] ||
	 F1[0] == F2[2] && F1[1] == F2[1] && F1[2] == F2[0] )
      {
	return 3;
      }
    
    if ( F1[0] == F2[0] && F1[1] == F2[1] ||
	 F1[1] == F2[1] && F1[2] == F2[2] ||
	 F1[2] == F2[2] && F1[0] == F2[0] ||
	 F1[0] == F2[0] && F1[1] == F2[2] ||
	 F1[0] == F2[0] && F1[2] == F2[1] ||
	 F1[1] == F2[1] && F1[2] == F2[0] ||
	 F1[1] == F2[1] && F1[0] == F2[2] ||
	 F1[2] == F2[2] && F1[1] == F2[0] ||
	 F1[2] == F2[2] && F1[0] == F2[1] ||
	 F1[0] == F2[1] && F1[1] == F2[0] ||
	 F1[0] == F2[1] && F1[1] == F2[2] ||
	 F1[0] == F2[1] && F1[2] == F2[0] ||
	 F1[0] == F2[2] && F1[1] == F2[0] ||
	 F1[0] == F2[2] && F1[2] == F2[0] ||
	 F1[0] == F2[2] && F1[2] == F2[1] ||
	 F1[1] == F2[2] && F1[2] == F2[0] ||
	 F1[1] == F2[2] && F1[2] == F2[1] ||
	 F1[1] == F2[0] && F1[2] == F2[1] )
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

    std::cout << "n " << n << std::endl;	  
    
    // segment counter
    unsigned int c;
    c = 0;
    
    for (int i=0; i < F.rows(); i++)
      {
	// j=i no duplicate segments
	for (int j=i; j < F.rows(); j++)
	  {
	    std::cout << "F " << i << std::endl;	  
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

    unsigned int n1,n2;
    toSegment(F2, f2a, f2b, n1, n2);

    Vector3d s1 = V2.row(n1);
    Vector3d s2 = V2.row(n2);
    
    // Plane and segment intersection
    Vector3d v1 = p1 - p2;
    Vector3d v2 = p1 - p3;

    // cross product to find normal
    Vector3d n = v1.cross(v2);
    n.normalize();
  
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
  
    // Test if point is inside triangle
  
    Vector3d e1 = p2 - p1;
    Vector3d e2 = p3 - p2;
    Vector3d e3 = p1 - p3;
    
    Vector3d c1 = p - p1;
    Vector3d c2 = p - p2;
    Vector3d c3 = p - p3;

    float r1 = n.dot(e1.cross(c1));
    float r2 = n.dot(e2.cross(c2));
    float r3 = n.dot(e3.cross(c3));
  
    // is point outisde triangle
    if (r1 < 0.0 || r2 < 0.0 || r3 < 0.0)
      {
	//std::cout << "r1: " << r1 << " r2: " << r2 << " r3: " << r3 << std::endl;
	return false;
      }
    
    return true;
  }
}
