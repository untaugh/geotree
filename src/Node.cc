#include "Node.h"
#include "Calc.h"
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
	  t = NULL;
      	  //std::cout << "This is a translate node"<< std::endl;
      	}
      else if (UnionNode * t = dynamic_cast<UnionNode*>(n))
      	{
	  t = NULL;
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
 * Do the translation. 
 */
void TranslateNode::build()
{
  // first combine geometries into one
  UnionNode::build();

  // need vector size 4 for matrix multiplication
  Eigen::Vector4d v(0.0, 0.0, 0.0, 1.0);

  // traverse rows
  for (int i; i < this->g.V.rows(); i++)
    {
      v.segment(0,3) = this->g.V.row(i);
      v = t.matrix() * v;
      this->g.V.row(i) = v.segment(0,3);
    }
}

void UnionNode::build()
{
  Node::build();

  int Vsize=0;
  int Fsize=0;
  
  for (Node * n : this->children)
    {
      Vsize += n->g.V.rows();
      Fsize += n->g.F.rows();
    }

  Eigen::MatrixXd V(Vsize,3);
  Eigen::MatrixXi F(Fsize,3);
  
  Vsize=0;
  Fsize=0;
  for (Node * n : this->children)
    {
      V.block(Vsize,0,n->g.V.rows(),3) = n->g.V;
      F.block(Fsize,0,n->g.F.rows(),3) = n->g.F;
      Vsize += n->g.V.rows();
      Fsize += n->g.F.rows();
    }

  Geometry g(V,F);
  this->g = g;

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

  Geometry g(V,F);
  this->g = g;
}

/* * * 
 * Translate in three dimensions. 
 */
TranslateNode::TranslateNode(double x, double y, double z) :
  t(Eigen::Translation3d(x,y,z))
{
}
