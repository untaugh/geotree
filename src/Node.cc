#include <Node.h>
#include <iostream>

/* * * 
 * Add a child node.
 */
void Node::add(Node * node)
{
  this->children.push_back(node);
}

/* * * 
 * Traverse and build children nodes.
 */
void Node::build()
{
  for (Node * n : this->children)
    {
      if (TranslateNode * t = dynamic_cast<TranslateNode*>(n))
      	{
      	  //std::cout << "This is a translate node"<< std::endl;
      	}
      else if (UnionNode * t = dynamic_cast<UnionNode*>(n))
      	{
      	  //std::cout << "This is a union node"<< std::endl;
      	}

      else
	{
      	  //std::cout << "This is a node"<< std::endl;
	}
      n->build();
    }
}

/* * * 
 * Intersection between faces.
 */
typedef struct ISMAP{
  int face1;
  int face2;
  int s1;
  int s2;
  Vector3d p;
} ISMAP;

/* * * 
 * Combine children geometries into one.
 */
void UnionNode::build()
{
  Node::build();

  // Calculate intersections

  if (this->children.size() > 1)
    {
      MatrixXd Vnew = MatrixXd(100,3);
      MatrixXi Fnew = MatrixXi(100,3);
      
      Node * n1 = this->children[0];
      Node * n2 = this->children[1];
      Vector3d v;
      
      std::vector<ISMAP> intersections;
      std::vector<ISMAP> intersections2;
      
      int f1, f2;
      for (f1=0; f1 < n1->g->F.rows(); f1++)
	{
	  int vF1[3] = { n1->g->F.row(f1)[0], n1->g->F.row(f1)[1], n1->g->F.row(f1)[2]};
	  MatrixXd F1 = Matrix<double,3,3>();
	  F1 << n1->g->V.row(n1->g->F.row(f1)[0]), n1->g->V.row(n1->g->F.row(f1)[1]), n1->g->V.row(n1->g->F.row(f1)[2]);
	  
	  for (f2=0; f2 < n2->g->F.rows(); f2++)
	    {
	      int vF2[3] = { n2->g->F.row(f2)[0], n2->g->F.row(f2)[1], n2->g->F.row(f2)[2]};
	      
	      MatrixXd F2 = Matrix<double,3,3>();
	      F2 << n2->g->V.row(vF2[0]), n2->g->V.row(vF2[1]), n2->g->V.row(vF2[2]);


	      for (int i=0; i<3; i++)
		{		  
		  Vector3d v1 = F2.row(i);
		  Vector3d v2 = F2.row((i+1)%3);
		  
		  if (n2->g->intersect(&F1, &v1, &v2, &v))
		    {
		      ISMAP map = {f1, f2, vF2[i], vF2[(i+1)%3], v};
		      intersections.push_back(map);
		    }
		}
	      
	      for (int i=0; i<3; i++)
		{		  
		  Vector3d v1 = F1.row(i);
		  Vector3d v2 = F1.row((i+1)%3);
		  
		  if (n2->g->intersect(&F2, &v1, &v2, &v))
		    {
		      ISMAP map = {f2, f1, vF1[i], vF1[(i+1)%3], v};
		      intersections2.push_back(map);
		    }
		}
	      
	    }
	}

      for (ISMAP m : intersections)
	{
	  std::cout << "face1: " << m.face1
		    << ", face2(segment): " << m.face2
		    << ", s1: " << m.s1
		    << ", s2: " << m.s2
		    << ", p: " << m.p.transpose()
		    << std::endl;
	}
      for (ISMAP m : intersections2)
	{
	  std::cout << "face2: " << m.face1
		    << ", face1(segment): " << m.face2
		    << ", s1: " << m.s1
		    << ", s2: " << m.s2
		    << ", p: " << m.p.transpose()
		    << std::endl;
	} 
    }
  
  int Vsize=0;
  int Fsize=0;
  
  for (Node * n : this->children)
    {
      Vsize += n->g->V.rows();
      Fsize += n->g->F.rows();
    }

  Eigen::MatrixXd V(Vsize,3);
  Eigen::MatrixXi F(Fsize,3);
  
  Vsize=0;
  Fsize=0;
  for (Node * n : this->children)
    {
      V.block(Vsize,0,n->g->V.rows(),3) = n->g->V;
      F.block(Fsize,0,n->g->F.rows(),3) = n->g->F;
      Vsize += n->g->V.rows();
      Fsize += n->g->F.rows();
      delete n->g;
    }
  
  this->g = new Geometry(V,F);
}

/* * * 
 * Do the translation. 
 */
void TranslateNode::build()
{
  // first combine geometries into one
  UnionNode::build();

  // need vector size 4 for matrix multiplication
  Eigen::Vector4d v(0.0, 0.0, 0.0, 1.0);

  // traverse rows
  for (int i; i < this->g->V.rows(); i++)
    {
      v.segment(0,3) = this->g->V.row(i);
      v = t.matrix() * v;
      this->g->V.row(i) = v.segment(0,3);
    }
}

/* * * 
 * A cube with triangle faces. 
 */
CubeNode::CubeNode(double x, double y, double z)
{
  Eigen::MatrixXd V(8,3);
  Eigen::MatrixXi F(12,3);

  V << 0.0, 0.0, 0.0,
    0.0, y, 0.0,
    x, y, 0.0,
    x, 0.0, 0.0,
    0.0, 0.0, z,
    0.0, y, z,
    x, y, z,
    x, 0.0, z;

  F << 0,1,2,
    0,2,3,
    0,1,4,
    1,4,5,
    1,2,5,
    2,5,6,
    2,3,6,
    3,6,7,
    3,0,7,
    0,7,4,
    4,5,6,
    4,6,7;

  this->g = new Geometry(V,F);  
}

/* * * 
 * Translate in three dimensions. 
 */
TranslateNode::TranslateNode(double x, double y, double z) :
  t(Eigen::Translation3d(x,y,z))
{
}
