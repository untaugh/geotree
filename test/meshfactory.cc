#include "gtest/gtest.h"
#include <Eigen/Core>
#include "MeshFactory.h"

namespace {

  using namespace Geotree;
  
  class MeshFactoryTest : public ::testing::Test {
  protected:
    MeshFactoryTest()
    {
    }
    ~MeshFactoryTest()
    {
    }
    MeshFactory factory;
  };

  void verifyCube(Mesh cube, double x, double y, double z)
  {
    EXPECT_EQ(12, cube.size());
    EXPECT_EQ(12, cube.F.rows());
    EXPECT_EQ(8, cube.V.rows());

    EXPECT_EQ(4*(x+y+z), cube.V.sum());

    EXPECT_TRUE( (cube.V.array() == x ||
		  cube.V.array() == y ||
		  cube.V.array() == z ||
		  cube.V.array() == 0.0).all());

    EXPECT_GE((cube.V.array() == x).count(), 4);
    EXPECT_GE((cube.V.array() == y).count(), 4);
    EXPECT_GE((cube.V.array() == z).count(), 4);
    EXPECT_EQ(12, (cube.V.array() == 0.0).count());

    EXPECT_EQ(1, (cube.V.rowwise().sum().array() == 0.0).count());
    EXPECT_GE((cube.V.rowwise().sum().array() == x).count(), 1);
    EXPECT_GE((cube.V.rowwise().sum().array() == y).count(), 1);
    EXPECT_GE((cube.V.rowwise().sum().array() == z).count(), 1);
    EXPECT_GE((cube.V.rowwise().sum().array() == x+y).count(), 1);
    EXPECT_GE((cube.V.rowwise().sum().array() == x+z).count(), 1);
    EXPECT_GE((cube.V.rowwise().sum().array() == y+z).count(), 1);
    EXPECT_GE((cube.V.rowwise().sum().array() == x+y+z).count(), 1);

    EXPECT_EQ(x*4, cube.V.col(0).sum());
    EXPECT_EQ(y*4, cube.V.col(1).sum());
    EXPECT_EQ(z*4, cube.V.col(2).sum());
  }

  void nullCube(Mesh cube)
  {
    EXPECT_EQ(0, cube.size());
    EXPECT_EQ(0, cube.V.rows());
    EXPECT_EQ(0, cube.F.rows());
  }

  TEST_F(MeshFactoryTest, cube)
  {
    Mesh cube = factory.makeCube(10,10,10);
    
    EXPECT_EQ(12, cube.size());
    EXPECT_EQ(8, cube.V.rows());

    EXPECT_EQ(120.0, cube.V.sum());
	
    EXPECT_EQ(12,(cube.V.array() == 10.0).count() );

    EXPECT_EQ(1, (cube.V.rowwise().sum().array() == 30.0).count());
    EXPECT_EQ(3, (cube.V.rowwise().sum().array() == 20.0).count());
    EXPECT_EQ(3, (cube.V.rowwise().sum().array() == 10.0).count());

    EXPECT_EQ(40.0, cube.V.col(0).sum());
    EXPECT_EQ(40.0, cube.V.col(1).sum());
    EXPECT_EQ(40.0, cube.V.col(2).sum());
  }

  TEST_F(MeshFactoryTest, cubesVerify)
  {
    verifyCube(factory.makeCube(10, 10, 10), 10.0, 10.0, 10.0);
    verifyCube(factory.makeCube(1.0, 2.0, 3.0), 1.0, 2.0, 3.0);
    verifyCube(factory.makeCube(1.0, 1.0, 2.0), 1.0, 1.0, 2.0);
    verifyCube(factory.makeCube(1.0, 2.0, 1.0), 1.0, 2.0, 1.0);
    verifyCube(factory.makeCube(0.0001, 0.567, 100.3), 0.0001, 0.567, 100.3);
  }

  TEST_F(MeshFactoryTest, cubeInvalid)
  {
    nullCube(factory.makeCube(0,0,0));
    nullCube(factory.makeCube(0.0, 10, 10));
    nullCube(factory.makeCube(10, 0.0, 10));
    nullCube(factory.makeCube(10, 10, 0.0));
    nullCube(factory.makeCube(-1.0, 10, 10));
    nullCube(factory.makeCube(10, -1.0, 10));
    nullCube(factory.makeCube(10, 10, -1.0));
  }

  TEST_F(MeshFactoryTest, DISABLED_sphere)
  {
    Mesh sphere = factory.makeSphere(10);

    EXPECT_EQ(20, sphere.size());
  }

  TEST_F(MeshFactoryTest, DISABLED_cylinder)
  {
    Mesh cylinder = factory.makeCylinder(10, 10);

    EXPECT_EQ(20, cylinder.size());
  }
}
