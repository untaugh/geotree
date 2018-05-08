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
}
