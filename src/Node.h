#pragma once
#include <vector>
#include "Geometry.h"
#include <Eigen/Geometry>
class Node {
 public:
  std::vector<Node*> children;
  Geometry * g;
  virtual void build();
  void add(Node * node);
};

class UnionNode : public Node
{
 public:
  UnionNode(){};
  void build();
};

class GeometryNode : public Node
{
 public:
  void build(){};
  GeometryNode(){};
  GeometryNode(Geometry *g_){ this->g = g_; };
};

class CubeNode : public GeometryNode
{
 public:
  CubeNode(double x, double y, double z);
};

class TranslateNode : public UnionNode
{
 public:
  void build();
  TranslateNode(double x, double y, double z);
 private:
  Eigen::Affine3d t;
  
};
