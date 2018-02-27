#include "gtest/gtest.h"
#include <Eigen/Core>

#include "Path.h"
#include "Mesh.h"
#include "IntersectionFace.h"

namespace {
    using namespace Geotree;

    class PathTest : public ::testing::Test {
    };

    TEST_F(PathTest, first) {
        Mesh mesh;
        PathT path(mesh);

        path.add(Vector3d(0, 0, 0));

        EXPECT_EQ(path.size(), 1);

        EXPECT_EQ(path.get(0), Vector3d(0, 0, 0));
    }

    TEST_F(PathTest, addPoints) {
        Mesh mesh;
        PathT path(mesh);

        path.add(Vector3d(1, 0, 0));
        path.add(Vector3d(0, 2, 0));
        path.add(Vector3d(0, 0, 3));

        EXPECT_EQ(path.size(), 3);

        EXPECT_EQ(path.get(0), Vector3d(1, 0, 0));
        EXPECT_EQ(path.get(1), Vector3d(0, 2, 0));
        EXPECT_EQ(path.get(2), Vector3d(0, 0, 3));
    }

    TEST_F(PathTest, addOperator) {
        Mesh mesh;
        PathT path(mesh);

        path.add(Vector3d(1, 4, 0));
        path += Vector3d(0, 2, 5);
        path += Vector3d(6, 0, 3);

        EXPECT_EQ(path.size(), 3);

        EXPECT_EQ(path.get(0), Vector3d(1, 4, 0));
        EXPECT_EQ(path.get(1), Vector3d(0, 2, 5));
        EXPECT_EQ(path.get(2), Vector3d(6, 0, 3));
    }

    TEST_F(PathTest, eqOperatorFlip) {
        Mesh mesh;
        PathT path(mesh);
        PathT path2(mesh);
        PathT path3(mesh);
        PathT path4(mesh);

        path.add(Vector3d(1, 0, 0));
        path.add(Vector3d(0, 2, 0));
        path.add(Vector3d(0, 0, 3));

        path2.addExisting(0);
        path2.addExisting(1);
        path2.addExisting(2);

        path3.addExisting(2);
        path3.addExisting(1);
        path3.addExisting(0);

        path4.addExisting(0);
        path4.addExisting(1);

        EXPECT_TRUE(path == path2);
        EXPECT_TRUE(path == path3);

        EXPECT_FALSE(path == path4);

    }

    TEST_F(PathTest, eqOperatorStartPoint) {
        Mesh mesh;
        PathT path1(mesh);
        PathT path2(mesh);
        PathT path3(mesh);
        PathT path4(mesh);
        PathT path5(mesh);
        PathT path6(mesh);

        path1.add(Vector3d(1, 0, 0));
        path1.add(Vector3d(2, 0, 0));
        path1.add(Vector3d(3, 0, 0));
        path1.add(Vector3d(4, 0, 0));
        path1.add(Vector3d(5, 0, 0));

        path2.addExisting(1);
        path2.addExisting(2);
        path2.addExisting(3);
        path2.addExisting(4);
        path2.addExisting(0);

        path3.addExisting(4);
        path3.addExisting(0);
        path3.addExisting(1);
        path3.addExisting(2);
        path3.addExisting(3);

        path4.addExisting(3);
        path4.addExisting(2);
        path4.addExisting(1);
        path4.addExisting(0);
        path4.addExisting(4);

        path5.addExisting(0);
        path5.addExisting(4);
        path5.addExisting(3);
        path5.addExisting(2);
        path5.addExisting(1);

        path6.addExisting(3);
        path6.addExisting(0);
        path6.addExisting(1);
        path6.addExisting(2);
        path6.addExisting(4);

        EXPECT_EQ(path1, path2);
        EXPECT_EQ(path1, path3);
        EXPECT_EQ(path1, path4);
        EXPECT_EQ(path1, path5);
        EXPECT_NE(path1, path6);
    }

    TEST_F(PathTest, eqOperatorNoSharedStartPoint) {
        Mesh mesh;

        PathT path1(mesh);
        PathT path2(mesh);
        PathT path3(mesh);
        PathT path4(mesh);
        PathT path5(mesh);
        PathT path6(mesh);

        path1.add(Vector3d(1, 0, 0));
        path1.add(Vector3d(2, 0, 0));
        path1.add(Vector3d(3, 0, 0));
        path1.add(Vector3d(4, 0, 0));
        path1.add(Vector3d(5, 0, 0));
        path1.add(Vector3d(5, 0, 0));

        path2.addExisting(0);
        path2.addExisting(1);
        path2.addExisting(2);
        path2.addExisting(3);
        path2.addExisting(4);

        path3.addExisting(1);
        path3.addExisting(2);
        path3.addExisting(3);
        path3.addExisting(4);
        path3.addExisting(5);

        path4.addExisting(1);
        path4.addExisting(2);
        path4.addExisting(3);
        path4.addExisting(0);
        path4.addExisting(5);

        EXPECT_NE(path2, path3);
        EXPECT_NE(path2, path4);

    }

        TEST_F(PathTest, getPoint) {
        Mesh mesh;
        PathT path1(mesh);

        path1.add(Vector3d(1, 0, 0));
        path1.add(Vector3d(2, 0, 0));
        path1.add(Vector3d(3, 0, 0));
        path1.add(Vector3d(4, 0, 0));
        path1.add(Vector3d(5, 0, 0));

        Point point = path1.begin();
        EXPECT_EQ(point.getIndex(), 0);
        EXPECT_EQ(point.getVector(), Vector3d(1, 0, 0));

        Point point2 = path1.back();
        EXPECT_EQ(point2.getIndex(), 4);
        EXPECT_EQ(point2.getVector(), Vector3d(5, 0, 0));

    }

    TEST_F(PathTest, addPoint) {
        Mesh mesh;
        PathT path(mesh);

        Point point(mesh, 0);

        path.add(point);

        EXPECT_EQ(path.size(), 1);
    }

    TEST_F(PathTest, EdgeToEdge)
    {

        Mesh mesh;

        PathX<FacePoint> pathSegment(mesh);
        pathSegment.add(FacePoint(mesh, Vector(0.5, 0, 0), SEGMENT, 0));
        pathSegment.add(FacePoint(mesh, Vector(0, 0.5, 0), SEGMENT, 2));

        EXPECT_TRUE(pathSegment.edgeToEdge());

        PathX<FacePoint> pathFace(mesh);

        pathFace.add(FacePoint(mesh, Vector(0.1, 0.1, 0), FACE, 0));
        pathFace.add(FacePoint(mesh, Vector(0.5, 0.1, 0), FACE, 0));
        pathFace.add(FacePoint(mesh, Vector(0.1, 0.5, 0), FACE, 0));

        EXPECT_FALSE(pathFace.edgeToEdge());
    }

    TEST_F(PathTest, Triangulate)
    {
        Mesh mesh;

        PathX<Point> path(mesh);

        path.add(Point(mesh, mesh.add(Vector3d(0,0,0))));
        path.add(Point(mesh, mesh.add(Vector3d(1,0,0))));
        path.add(Point(mesh, mesh.add(Vector3d(1,1,0))));
        path.add(Point(mesh, mesh.add(Vector3d(0,1,0))));

        std::vector <PathX<Point>> holes;

        MatrixX3i triangles = path.triangulate(holes);

        MatrixX3i expected(2,3);
        expected << 0,1,2, 1,2,3;

        EXPECT_EQ(triangles, expected);

        std::cout << triangles << std::endl;
    }

    TEST_F(PathTest, TriangulateHole)
    {
        Mesh mesh;

        PathX<Point> path(mesh);
        path.add(Point(mesh, mesh.add(Vector3d(0,0,0))));
        path.add(Point(mesh, mesh.add(Vector3d(1,0,0))));
        path.add(Point(mesh, mesh.add(Vector3d(1,1,0))));
        path.add(Point(mesh, mesh.add(Vector3d(0,1,0))));

        PathX<Point> hole(mesh);
        hole.add(Point(mesh, mesh.add(Vector3d(0.1,0.1,0))));
        hole.add(Point(mesh, mesh.add(Vector3d(0.9,0.1,0))));
        hole.add(Point(mesh, mesh.add(Vector3d(0.9,0.9,0))));
        hole.add(Point(mesh, mesh.add(Vector3d(0.1,0.9,0))));

        std::vector <PathX<Point>> holes;

        holes.push_back(hole);

        MatrixX3i triangles = path.triangulate(holes);

        MatrixX3i expected(8,3);
        //expected << 0,1,2, 1,2,3;

        EXPECT_EQ(triangles, expected);

        std::cout << triangles << std::endl;
    }

}