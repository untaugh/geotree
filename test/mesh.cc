#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Mesh.h"
#include "MeshFactory.h"

namespace {

  using namespace Geotree;
  
  class MeshTest : public ::testing::Test {
  protected:
    MeshFactory mf;
  };

//  void verifyTranslate(Mesh before, Mesh after, double x, double y, double z)
//  {
//    EXPECT_EQ(before.V.size(), after.V.size());
//    EXPECT_EQ(before.F.size(), after.F.size());
//
//    EXPECT_TRUE((before.F.array() == after.F.array()).all());
//
//    for (int i=0; i<before.V.rows(); i++ )
//      {
//	EXPECT_EQ(before.V.row(i)[0] + x, after.V.row(i)[0]);
//	EXPECT_EQ(before.V.row(i)[1] + y, after.V.row(i)[1]);
//	EXPECT_EQ(before.V.row(i)[2] + z, after.V.row(i)[2]);
//      }
//  }    TEST_F(MeshTest, addOperatorBasic)
    //{

    //}

  TEST_F(MeshTest, first)
  {
    Verticies V(0,3);
    Faces F(0,3);

    Mesh mesh(V,F);

    EXPECT_EQ(0, mesh.vectorcount());
    EXPECT_EQ(0, mesh.facecount());
  }

    TEST_F(MeshTest, oneFace)
    {
        Verticies V(3,3);
        Faces F(1,3);

        V << 0,0,0, 1,0,0, 0,1,0;
        F << 0,1,2;

        Mesh mesh(V,F);

        EXPECT_EQ(3, mesh.vectorcount());
        EXPECT_EQ(1, mesh.facecount());

        FaceT face = mesh.getFaceT(0);

        FaceT face_eq0(mesh, 0);
        FaceT face_eq1(mesh, 1);

        EXPECT_EQ(face_eq0, face);
        EXPECT_NE(face_eq1, face);

        Point point0 = face.getPoint(0);
        Point point1 = face.getPoint(1);

        Point point_eq0(mesh, 0);
        Point point_eq1(mesh, 1);
        Point point_eq2(mesh, 2);

        EXPECT_EQ(point0, point_eq0);
        EXPECT_NE(point0, point_eq1);
        EXPECT_NE(point0, point_eq2);
        EXPECT_EQ(point1, point_eq1);
    }

    Mesh meshWithSize(uint32_t vectors, uint32_t faces)
    {
        Verticies V(vectors,3);
        Faces F(faces,3);

        return Mesh(V,F);
    }

    TEST_F(MeshTest, sizes)
    {
        EXPECT_EQ(0, meshWithSize(0,0).facecount());

        EXPECT_EQ(1, meshWithSize(0,1).facecount());

        EXPECT_EQ((1<<24), meshWithSize(0,1<<24).facecount());

        EXPECT_EQ((1<<10), meshWithSize(1<<10,1<<10).vectorcount());
    }

    TEST_F(MeshTest, addOperatorBasic)
    {
        Verticies Vm(4,3);
        Faces Fm(4,3);
        Vm << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
        Fm << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

        Verticies Vn(4,3);
        Faces Fn(4,3);
        Vn << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
        Fn << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

        Mesh m(Vm, Fm);
        Mesh n(Vn, Fn);

        Mesh l = m + n;

        EXPECT_EQ(l.vectorcount(), 8);
        EXPECT_EQ(l.facecount(), 8);
    }



//  // TEST_F(MeshTest, SegmentIndex)
//  // {
//  //   Mesh mesh;
//  //   mesh.S = Segments(5,2);
//  //   mesh.S << 0,1, 1,2, 4,1, 40,32, 4,3;
//
//  //   EXPECT_EQ(mesh.getSegmentIndex(SegmentIndex(0,1)),0);
//  //   EXPECT_EQ(mesh.getSegmentIndex(SegmentIndex(1,0)),0);
//  //   EXPECT_EQ(mesh.getSegmentIndex(SegmentIndex(40,32)),3);
//  //   EXPECT_EQ(mesh.getSegmentIndex(SegmentIndex(32,40)),3);
//  //   EXPECT_EQ(mesh.getSegmentIndex(SegmentIndex(3,4)),4);
//  // }
//
//    TEST_F(MeshTest, rowCount)
//    {
//
//    }
}
