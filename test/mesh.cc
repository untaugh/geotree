#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Mesh.h"
#include "MeshFactory.h"

namespace {
    using namespace Geotree;

    class MeshTest : public ::testing::Test {
    protected:
    };

  TEST_F(MeshTest, getSegments)
  {
    MeshFactory factory;
    Mesh tetra = factory.tetra(1);

    std::vector<Segment> segments;
    tetra.getSegments(segments);

    EXPECT_EQ(segments.size(), 6);
  }

  TEST_F(MeshTest, boundingBox)
  {
    MeshFactory factory;
    Mesh tetra = factory.tetra(1);

    Cube box = tetra.boundingBox();

    EXPECT_EQ(box.p0, Vector3d(0.0, 0.0, 0.0));
    EXPECT_EQ(box.p1, Vector3d(1.0, 1.0, 1.0));
  }

  TEST_F(MeshTest, boundingBoxNegative)
  {
    MeshFactory factory;
    Mesh tetra = factory.tetra(2);

    tetra.translate(Vector3d(-10, -20, -30));
    Cube box = tetra.boundingBox();

    EXPECT_EQ(box.p0, Vector3d(-10.0, -20.0, -30.0));
    EXPECT_EQ(box.p1, Vector3d(-8.0, -18.0, -28.0));
  }

  TEST_F(MeshTest, boundingBoxZero)
  {
    Mesh zero;

    Cube box = zero.boundingBox();
    
    EXPECT_EQ(box.p0, Vector3d(0.0, 0.0, 0.0));
    EXPECT_EQ(box.p1, Vector3d(0.0, 0.0, 0.0));
  }
}
