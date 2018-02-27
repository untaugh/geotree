#include "gtest/gtest.h"
#include "Intersection.h"
#include "MeshFactory.h"
#include <Eigen/Core>

using namespace Eigen;

namespace {
    using namespace Geotree;

    class IntersectionTest : public ::testing::Test {
    };

    TEST_F(IntersectionTest, basic)
    {
        MeshFactory factory;
        Mesh tetra1 = factory.makeTetra(1);
        Mesh tetra2 = factory.makeTetra(1);

        tetra2.translate(Vector(0.1, 0.1, 0.1));

        Intersection is(tetra1, tetra2);

        EXPECT_EQ(1, is.pathcount());

    }
}
