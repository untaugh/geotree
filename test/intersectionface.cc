#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "MeshIntersection.h"
#include "IntersectionFace.h"
#include "MeshFactory.h"
#include "Face.h"

namespace {
    using namespace Geotree;

    class IntersectionFaceTest : public ::testing::Test {
    };

    TEST_F(IntersectionFaceTest, basic)
    {
        //Mesh mesh;

        Verticies V(3,3);
        Faces F(1,3);

        V << 0,0,0, 1,0,0, 0,1,0;
        F << 0,1,2;

        //mesh.V = V;
        //mesh.F = F;

        Mesh mesh(V,F);

        IntersectionFace face = mesh.getFaceT(0);

        PathX<FacePoint> path(mesh);
        path.add(FacePoint(mesh, Vector(0.5, 0, 0), SEGMENT, 0));
        path.add(FacePoint(mesh, Vector(0, 0.5, 0), SEGMENT, 2));
        face.cut(path);

        EXPECT_EQ(face.numberOfPaths(), 0);
        EXPECT_EQ(face.numberOfEdgePoints(), 5);

        face.completePaths();

        EXPECT_EQ(face.numberOfPaths(), 2);

        EXPECT_TRUE(face.pathSize(0) == 4 || face.pathSize(0) == 3);
        EXPECT_TRUE(face.pathSize(1) == 4 || face.pathSize(1) == 3);

        PathT p1(mesh);
        p1.addExisting(3);
        p1.addExisting(4);
        p1.addExisting(0);

        PathT p2(mesh);
        p2.addExisting(3);
        p2.addExisting(4);
        p2.addExisting(2);
        p2.addExisting(1);

        std::cout << face.getPath(0) << std::endl;
        std::cout << face.getPath(1) << std::endl;

        EXPECT_TRUE(face.getPath(0) == p1 || face.getPath(0) == p2);
        EXPECT_TRUE(face.getPath(1) == p1 || face.getPath(1) == p2);

        face.triangulate();

        for (int j=0; j<2; j++) {
            std::set<int> area = face.getArea(j);

            std::cout << "area: ";
            for (int i : area) {
                std::cout << i << ", ";
            }
            std::cout << std::endl;
        }

    }

    TEST_F(IntersectionFaceTest, twoCuts)
    {
        //Mesh mesh;

        Verticies V(3,3);
        Faces F(1,3);

        V << 0,0,0, 1,0,0, 0,1,0;
        F << 0,1,2;

        Mesh mesh(V,F);
        //mesh.V = V;
        //mesh.F = F;

        IntersectionFace face = mesh.getFaceT(0);

        PathX<FacePoint> path(mesh);
        PathX<FacePoint> path2(mesh);

        path.add(FacePoint(mesh, Vector(0.5, 0, 0), SEGMENT, 0));
        path2.add(FacePoint(mesh, Vector(0.7, 0, 0), SEGMENT, 0));

        path.add(FacePoint(mesh, Vector(0, 0.5, 0), SEGMENT, 2));
        path2.add(FacePoint(mesh, Vector(0, 0.7, 0), SEGMENT, 2));

        face.cut(path);
        face.cut(path2);

        EXPECT_EQ(face.numberOfPaths(), 0);
        EXPECT_EQ(face.numberOfEdgePoints(), 7);

        face.completePaths();

        EXPECT_EQ(face.numberOfPaths(), 3);

        std::cout << face.getPath(0) << std::endl;
        std::cout << face.getPath(1) << std::endl;


        EXPECT_TRUE(face.pathSize(0) == 4 || face.pathSize(0) == 3);
        EXPECT_TRUE(face.pathSize(1) == 4 || face.pathSize(1) == 3);
        EXPECT_TRUE(face.pathSize(2) == 4 || face.pathSize(2) == 3);

        PathT p1(mesh);
        p1.addExisting(3);
        p1.addExisting(5);
        p1.addExisting(0);

        PathT p2(mesh);
        p2.addExisting(3);
        p2.addExisting(5);
        p2.addExisting(6);
        p2.addExisting(4);

        EXPECT_TRUE(face.getPath(0) == p1 || face.getPath(0) == p2);
        EXPECT_TRUE(face.getPath(1) == p1 || face.getPath(1) == p2);

        face.triangulate();

        for (int j=0; j<3; j++) {
            std::set<int> area = face.getArea(j);

            std::cout << "area: ";
            for (int i : area) {
                std::cout << i << ", ";
            }
            std::cout << std::endl;
        }
    }

    TEST_F(IntersectionFaceTest, longer)
    {
        Verticies V(3,3);
        Faces F(1,3);

        V << 0,0,0, 1,0,0, 0,1,0;
        F << 0,1,2;

        Mesh mesh(V,F);
        IntersectionFace face = mesh.getFaceT(0);

        PathX<FacePoint> path(mesh);

        path.add(FacePoint(mesh, Vector(0.5, 0, 0), SEGMENT, 0));
        path.add(FacePoint(mesh, Vector(0.5, 0.1, 0), FACE, 0));
        path.add(FacePoint(mesh, Vector(0.5, 0.2, 0), FACE, 0));
        path.add(FacePoint(mesh, Vector(0.5, 0.3, 0), FACE, 0));
        path.add(FacePoint(mesh, Vector(0.0, 0.5, 0), SEGMENT, 2));

        face.cut(path);

        EXPECT_EQ(face.numberOfPaths(), 0);
        EXPECT_EQ(face.numberOfEdgePoints(), 5);

        face.completePaths();

        EXPECT_EQ(face.numberOfPaths(), 2);

        std::cout << face.getPath(0) << std::endl;
        std::cout << face.getPath(1) << std::endl;


        EXPECT_TRUE(face.pathSize(0) == 6 || face.pathSize(0) == 7);
        EXPECT_TRUE(face.pathSize(1) == 6 || face.pathSize(1) == 7);

        face.triangulate();

        for (int j=0; j<2; j++) {
            std::set<int> area = face.getArea(j);

            std::cout << "area: ";
            for (int i : area) {
                std::cout << i << ", ";
            }
            std::cout << std::endl;
        }
    }

    TEST_F(IntersectionFaceTest, noEdge) {

        Verticies V(3, 3);
        Faces F(1, 3);

        V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
        F << 0, 1, 2;

        Mesh mesh(V,F);

        IntersectionFace face = mesh.getFaceT(0);

        PathX<FacePoint> path(mesh);

        path.add(FacePoint(mesh, Vector(0.1, 0.1, 0), FACE, 0));
        path.add(FacePoint(mesh, Vector(0.5, 0.1, 0), FACE, 0));
        path.add(FacePoint(mesh, Vector(0.1, 0.5, 0), FACE, 0));

        face.cut(path);

        EXPECT_EQ(face.numberOfPaths(), 0);
        EXPECT_EQ(face.numberOfEdgePoints(), 3);

        face.completePaths();

        EXPECT_EQ(face.numberOfPaths(), 2);

        face.triangulate();

        for (int j=0; j<2; j++) {
            std::set<int> area = face.getArea(j);

            std::cout << "area: ";
            for (int i : area) {
                std::cout << i << ", ";
            }
            std::cout << std::endl;
        }
    }

    }
