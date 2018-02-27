#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "MeshIntersection.h"
#include "IntersectionPoint.h"
#include "MeshFactory.h"

namespace {
    using namespace Geotree;

    class MeshIntersectionTest : public ::testing::Test {
    };

    int countPointsAdjascent(IntersectionPoints ip)
    {
        std::pair <int, int> points;
        FaceSet pointsset;

        int previouspoint = -1;
        int point = 0;
        int count = 0;
        do
        {
            pointsset = ip.getPoint(point).connectedPoints;

            if (pointsset.size() == 3)
            {
                for (int i: pointsset)
                {
                    if (ip.getPoint(i).connectedPoints.size() == 3)
                    {
                        pointsset.erase(i);
                    }
                }
            }

            if (pointsset.size() != 2)
            {
                std::cout << "Size is not 2: " << pointsset.size() <<std::endl;
                return -1;
            }

            points.first = *pointsset.begin();
            points.second = *pointsset.rbegin();
            //std::cout << point << " points: " << points.first << ", " <<points.second << std::endl;
            if (points.first != previouspoint)
            {
                previouspoint = point;
                point = points.first;
            }
            else if (points.second != previouspoint)
            {
                previouspoint = point;
                point = points.second;
            }
            else
            {
                return -1;
            }
            count++;
        } while(point != 0);

        return count;
    }

    TEST_F(MeshIntersectionTest, DISABLED_TetrahedronHole) {

        Verticies Vm(4, 3);
        Faces Fm(4, 3);
        Vm << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Fm << 0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3;

        Verticies Vn(4, 3);
        Faces Fn(4, 3);
        Vn << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Fn << 0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3;

        Mesh m(Vm, Fm);
        Mesh n(Vn, Fn);

        n.translate(Vector(0.1, 0.1, 0.1));


        MeshIntersection mi(m, n);

        mi.merge();

        EXPECT_EQ(mi.mesh.size(), 8);

        EXPECT_EQ(mi.points.size(), 3);

        for (int i = 0; i < mi.points.size(); i++) {
            IntersectionPoint p = mi.points.getPoint(i);
            EXPECT_EQ(p.first.type, FACE);
            EXPECT_EQ(p.second.type, SEGMENT);
        }

        EXPECT_EQ(countPointsAdjascent(mi.points), 3);

        std::vector<FaceSet> subpaths;
        int face;
        std::vector<EdgePoint> edgePoints;
        FaceSet faces = mi.points.getIntersectedFaces();

        EXPECT_EQ(faces.size(), 3 + 1);

        face = *faces.begin();
        subpaths = mi.paths.getSubPaths(face);
        ASSERT_EQ(subpaths.size(), 1);
        EXPECT_EQ(subpaths[0].size(), 3);
    }

}