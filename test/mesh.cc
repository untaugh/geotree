#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Mesh.h"
#include "MeshFactory.h"

namespace {

  using namespace Geotree;
  
  class MeshTest : public ::testing::Test {
  protected:
    MeshTest()
    {
    }
    ~MeshTest()
    {
    }
    Mesh m;
    MeshFactory mf;
  };

  void verifyTranslate(Mesh before, Mesh after, double x, double y, double z)
  {
    EXPECT_EQ(before.V.size(), after.V.size());
    EXPECT_EQ(before.F.size(), after.F.size());

    EXPECT_TRUE((before.F.array() == after.F.array()).all());

    for (int i=0; i<before.V.rows(); i++ )
      {
	EXPECT_EQ(before.V.row(i)[0] + x, after.V.row(i)[0]);
	EXPECT_EQ(before.V.row(i)[1] + y, after.V.row(i)[1]);
	EXPECT_EQ(before.V.row(i)[2] + z, after.V.row(i)[2]);
      }
  }

  TEST_F(MeshTest, first)
  {
    EXPECT_EQ(0, this->m.V.size());
    EXPECT_EQ(0, this->m.F.size());
  }

  TEST_F(MeshTest, size)
  {
    EXPECT_EQ(0, this->m.size());

    this->m.F = Faces(1,3);
    EXPECT_EQ(1, this->m.size());

    this->m.F = Faces((1<<28),3);
    EXPECT_EQ((1<<28), this->m.size());
    
    this->m.V = Verticies((1<<10),3);
    EXPECT_EQ((1<<10), this->m.V.rows());
  }

  TEST_F(MeshTest, translate)
  {
    Mesh cube = mf.makeCube(10,10,10);
    Mesh cubeTranslate;
    
    cubeTranslate = cube;
    cubeTranslate.translate(Vertex(10,0,0));
    verifyTranslate(cube, cubeTranslate, 10,0,0);

    cubeTranslate = cube;
    cubeTranslate.translate(Vertex(0,0,0));
    verifyTranslate(cube, cubeTranslate, 0,0,0);

    cubeTranslate = cube;
    cubeTranslate.translate(Vertex(-10.1, -55.2, -3.2));
    verifyTranslate(cube, cubeTranslate, -10.1, -55.2, -3.2);
  }

  TEST_F(MeshTest, add)
  {
    Mesh cube1 = mf.makeCube(1,1,1);
    Mesh cube2 = mf.makeCube(1,1,1);

    cube2.translate(Vertex(2,2,2));

    Mesh cube3 = cube1 + cube2;

    EXPECT_EQ(24, cube3.size());
    EXPECT_EQ(24, cube3.F.rows());
    EXPECT_EQ(16, cube3.V.rows());    
  }
}
