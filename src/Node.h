#pragma once
#include <vector>
#include <Eigen/Geometry>
#include "Mesh.h"

namespace Geotree
{
class Node {
 public:
  std::vector<Node*> children;
  virtual void build();
  void add(Node * node);
};

class UnionNode : public Node
{
 public:
  UnionNode(){};
  void build();
};

class MeshNode : public Node
{
 public:
  void build(){};
  MeshNode(){};
  Mesh mesh;
};

class CubeNode : public MeshNode
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
}
