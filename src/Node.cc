#include "Node.h"
#include <iostream>

namespace Geotree
{
void Node::add(Node * node)
{
  this->children.push_back(node);
}

void Node::build()
{
  for (Node * n : this->children)
    {
      if (TranslateNode * t = dynamic_cast<TranslateNode*>(n))
      	{
	  t = NULL;
      	}
      else if (UnionNode * t = dynamic_cast<UnionNode*>(n))
      	{
	  t = NULL;
      	}
      else
      	{
      	}
      n->build();
    }
}

void TranslateNode::build()
{
  UnionNode::build();
}

void UnionNode::build()
{
  Node::build();
}

CubeNode::CubeNode(double x, double y, double z)
{
}

TranslateNode::TranslateNode(double x, double y, double z) :
  t(Eigen::Translation3d(x,y,z))
{
}
}
