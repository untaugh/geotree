#include "gtest/gtest.h"
#include <Eigen/Core>

#include "Validate.h"
#include "Geometry.h"

namespace {
  class ValidateTest : public ::testing::Test {
  };

  // test verticies form a valid triangular face
  TEST_F(ValidateTest, TriangularFace)
  {
    Eigen::Matrix3d face;    
    face << 1,0,0,2,0,0,3,0,0;

    EXPECT_TRUE(Geotree::Validate::face(face));
  }

  TEST_F(ValidateTest, Verticies)
  {
    const int num = 4;
    Eigen::Matrix3d f[num];

    // dupliate verticies
    f[0] << 0,0,0,0,0,0,0,0,0;
    f[1] << 1,2,3,1,2,3,4,5,6;
    f[2] << 0,0,1,32,75,10,32,75,10;
    f[3] << -34,21,19,-34,19,21,-34,21,19;
    
    for (int i=0; i<num; i++)
      {
	EXPECT_FALSE(Geotree::Validate::face(f[i]));	
      }
  }

  TEST_F(ValidateTest, Indicies)
  {
    Eigen::Vector3i f;

    f << 0,1,2;
    EXPECT_TRUE(Geotree::Validate::face(f));

    f << 0,0,0;
    EXPECT_FALSE(Geotree::Validate::face(f));
    
    f << 0,1,1;
    EXPECT_FALSE(Geotree::Validate::face(f));

    f << 4,0,4;
    EXPECT_FALSE(Geotree::Validate::face(f));
    
    f << 0,-1,1;
    EXPECT_FALSE(Geotree::Validate::face(f));
  }
  
  TEST_F(ValidateTest, GeometryVerticies)
  {
    Eigen::MatrixXd V = Eigen::Matrix3d();
    Eigen::MatrixXi F = Eigen::MatrixXi(1,3);
    Geometry g = Geometry(V,F);

    g.V << 1,2,0,2,3,0,3,4,0;
    g.F << 0,1,2;
    EXPECT_TRUE(Geotree::Validate::geometry(g));

    // duplicate faces
    g.V << 0,0,0,0,0,0,0,0,0;
    EXPECT_FALSE(Geotree::Validate::geometry(g));

    // duplicate face
    g.V << 1,2,0,2,3,0,1,2,0;
    EXPECT_FALSE(Geotree::Validate::geometry(g));

    g.V = Eigen::MatrixXd(9,3);
    g.V << 1,0,0,2,0,0,3,0,0,4,0,0,5,0,0,6,0,0,7,0,0,8,0,0,9,0,0;
    EXPECT_TRUE(Geotree::Validate::geometry(g));

    // duplicate face
    g.V << 1,0,0,2,0,0,3,0,0,4,0,0,5,0,0,6,0,0,7,0,0,8,0,0,8,0,0;
    EXPECT_FALSE(Geotree::Validate::geometry(g));
  }

  TEST_F(ValidateTest, GeometryIndicies)
  {
    Eigen::MatrixXd V = Eigen::Matrix3d();
    Eigen::MatrixXi F = Eigen::MatrixXi(1,3);
    Geometry g = Geometry(V,F);

    g.V << 1,0,0,2,0,0,3,0,0;
    g.F << 2,1,0;
    EXPECT_TRUE(Geotree::Validate::geometry(g));

    // index overflow
    g.V << 1,0,0,2,0,0,3,0,0;
    g.F << 0,1,3;
    EXPECT_FALSE(Geotree::Validate::geometry(g));

    // negative index
    g.F << 0,-1,3;
    EXPECT_FALSE(Geotree::Validate::geometry(g));
    
    // not valid face
    g.F << 0,1,1;
    EXPECT_FALSE(Geotree::Validate::geometry(g));
  }

  // index matrix size
  TEST_F(ValidateTest, GeometryIndexSize)
  {
    Eigen::MatrixXd V = Eigen::Matrix3d();
    Eigen::MatrixXi F = Eigen::Vector3i();
    Geometry g = Geometry(V,F);

    g.V << 1,0,0,2,0,0,3,0,0;
    g.F << 0,1,2;
    EXPECT_FALSE(Geotree::Validate::geometry(g));
  }

  // duplicate faces in face index
  TEST_F(ValidateTest, GeometryIndiciesDuplicate)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(6,3);
    Eigen::MatrixXi F = Eigen::MatrixXi(3,3);
    Geometry g = Geometry(V,F);

    g.V << 1,0,0,2,0,0,3,0,0,4,0,0,5,0,0,6,0,0;
    g.F << 0,1,2,0,1,3,0,1,4;
    EXPECT_TRUE(Geotree::Validate::geometry(g));

    g.F << 0,1,2,0,1,3,0,1,2;
    EXPECT_FALSE(Geotree::Validate::geometry(g));

    g.F << 0,1,2,0,1,3,2,1,0;
    EXPECT_FALSE(Geotree::Validate::geometry(g));
  }

  // test points of path on same plane
  TEST_F(ValidateTest, PathPlanar)
  {
    Eigen::MatrixXd P = Eigen::MatrixXd(4,3);

    P << 0.0,0.0,0.2, 1.0,0.0,0.2, 0.0,1.0,0.2, 1.0,1.0,0.2;
    EXPECT_TRUE(Geotree::Validate::planar(P));

    P << 0.0,0.0,0.2, 1.0,0.0,0.2, 0.0,1.0,0.2, -1.0,-0.1,0.2;
    EXPECT_TRUE(Geotree::Validate::planar(P));

    P = Eigen::MatrixXd(6,3);

    P << 0.0,0.0,0.2, 1.0,0.0,0.2, 0.0,1.0,0.2, 1.0,1.0,0.2, 0.1,0.2,0.2,
      -10.0,100.2,0.2;
    EXPECT_TRUE(Geotree::Validate::planar(P));

    P << 0.0,0.0,0.2, 1.0,0.0,0.2, 0.0,1.0,0.2, 1.0,1.0,0.2, 0.1,0.2,0.2,
      -10.0,100.2,0.1;
    EXPECT_FALSE(Geotree::Validate::planar(P));

    P << 1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0, -2.0,-1.0,4.0, 5.2,4.3,-8.5,
      -10.0,100.0,-89.0;
    EXPECT_TRUE(Geotree::Validate::planar(P));
  }
  
  // unreferenced verticies
  TEST_F(ValidateTest, DISABLED_GeometryUnreferenced)
  {
    
  }
}
