#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "MeshIntersection.h"
#include "IntersectionPoints.h"
#include "MeshFactory.h"

namespace {
    using namespace Geotree;

    class IntersectionPointsTest : public ::testing::Test {
    };

    TEST_F(IntersectionPointsTest, First) {
        Verticies V(3,3);
        Faces F(1,3);
        V << 0,0,0, 1,0,0, 0,1,0;
        F << 0,1,2;
        Mesh mesh(V, F);

        IntersectionPoints ips(mesh);
        PointInfo A,B,C;
        A.type = SEGMENT;
        A.index = 0;
        A.index2 = 1;
        B.type = SEGMENT;
        B.index = 1;
        B.index2 = 2;
        C.type = FACE;
        C.index = 3;
        C.index2 = 4;

        IntersectionPoint ip1(mesh, A, C, Vector3d(0.5,0,0));
        IntersectionPoint ip2(mesh, B, C, Vector3d(0,0.5,0));

        ips.addPoint(ip1);
        ips.addPoint(ip2);

        std::vector<PathX<FacePoint>> paths = ips.getFacePath(0);

        EXPECT_EQ(paths.size(), 1);
        EXPECT_EQ(paths[0].size(), 2);

    }
}