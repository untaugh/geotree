#pragma once
#include <vector>
#include "Types.h"
#include "Mesh.h"
#include <iterator>
#include <memory>
#include <iostream>
#define MPE_POLY2TRI_IMPLEMENTATION
#define MPE_POLY2TRI_USE_DOUBLE
#include "MPE_fastpoly2tri.h"

namespace Geotree
{
    template <typename T>
    class PathX
    {
    public:
        PathX(Mesh &_mesh) : mesh(_mesh) {};
        PathX(Mesh &_mesh, std::vector <T> _points) : mesh(_mesh), points(_points) { };
        void add(const Vector3d point)
        {
            //mesh.V.conservativeResize(mesh.V.rows() + 1, NoChange);
            //points.push_back(T(mesh, (mesh.V.rows() - 1)));
            //mesh.V.row(mesh.V.rows() - 1) = point;
        }
        void add(T point) { points.push_back(point); };
        void addExisting(const int index) { points.push_back(T(mesh,index)); };
        void addExisting(const T point) { points.push_back(point); };

        int size() const
        {
            return points.size();
        }
        Vector3d get(const int index);
        int index(int i) { return points[i].getIndex(); };
        std::vector <T> getPoints() const { return points; };

        bool edgeToEdge() const;

        bool hasPoint(double x, double y)
        {
            std::cout << "has:" << x << ", " << y << std::endl;

            for (auto point : getPoints())
            {
                Vector3d pointVector = point.getVector();

                std::cout << pointVector.x() - x << ", ";
                std::cout << pointVector.y() - y << std::endl;

                if (abs(pointVector.x() - x) < 0.000000001 && abs(pointVector.y() - y) < 0.000000001)
                {
                    return true;
                }
            }

            return false;
        }

        T findPoint(double x, double y)
        {
            for (auto point : getPoints())
            {
                Vector3d pointVector = point.getVector();
                std::cout << "find: " << pointVector.x()  << ", " << pointVector.y() << std::endl;

                if (abs(pointVector.x() - x) < 0.000000001 && abs(pointVector.y() - y) < 0.000000001)
                {
                    return point;
                }
            }
            std::cout << mesh << std::endl;
            throw;
        }

        bool contains(PathX<T> hole)
        {
            return true;
        }

        MatrixX3i triangulate()
        {
            std::vector <PathX<T>> holes;
            return triangulate(holes);
        }

        MatrixX3i triangulate(std::vector <PathX<T>> holes)
        {
            uint32_t MaxPointCount = 10000;

            size_t MemoryRequired = MPE_PolyMemoryRequired(MaxPointCount);

            void* Memory = calloc(MemoryRequired, 1);

            MPEPolyContext PolyContext;

            if (MPE_PolyInitContext(&PolyContext, Memory, MaxPointCount)) {
                for (auto point : points) {
                    MPEPolyPoint *Point = MPE_PolyPushPoint(&PolyContext);

                    Vector3d pointVector = point.getVector();
                    Point->X = pointVector[0];
                    Point->Y = pointVector[1];
                }
            }

            MPE_PolyAddEdge(&PolyContext);

            for (auto hole : holes)
            {
                for (auto point : hole.getPoints())
                {
                    MPEPolyPoint *Point = MPE_PolyPushPoint(&PolyContext);

                    Vector3d pointVector = point.getVector();
                    Point->X = pointVector[0];
                    Point->Y = pointVector[1];
                }

                MPE_PolyAddHole(&PolyContext);
            }

            MPE_PolyTriangulate(&PolyContext);

            std::cout << "size: " << PolyContext.TriangleCount << std::endl;

            MatrixX3i triangles(PolyContext.TriangleCount, 3);

            for (uxx TriangleIndex = 0; TriangleIndex < PolyContext.TriangleCount; ++TriangleIndex)
            {
                MPEPolyTriangle* Triangle = PolyContext.Triangles[TriangleIndex];
                MPEPolyPoint* PointA = Triangle->Points[0];
                MPEPolyPoint* PointB = Triangle->Points[1];
                MPEPolyPoint* PointC = Triangle->Points[2];

                for (int i=0; i<3; i++)
                {
                    std::cout << "tri: " << TriangleIndex << " " << i << std::endl;

                    if (hasPoint(Triangle->Points[i]->X, Triangle->Points[i]->Y))
                    {
                        triangles.row(TriangleIndex)[i] = findPoint(Triangle->Points[i]->X, Triangle->Points[i]->Y).getIndex();
                        std::cout << "res: " << triangles.row(TriangleIndex)[i] << std::endl;

                    }
                    else
                    {
                        for (auto hole : holes)
                        {
                            if (hole.hasPoint(Triangle->Points[i]->X, Triangle->Points[i]->Y))
                            {
                                triangles.row(TriangleIndex)[i] = hole.findPoint(Triangle->Points[i]->X, Triangle->Points[i]->Y).getIndex();
                                std::cout << "hole: " << i << " " << triangles.row(TriangleIndex)[i] << std::endl;
                            }
                        }
                    }

                }
            }

            return triangles;
        }

        T begin() const { return points.front(); };
        T back() const { return points.back(); };

        void operator +=(const Vector3d point)
        {
            add(point);
        }

        bool operator == (const PathX &path) const;
        bool operator != (const PathX &path) const;

        class const_iterator
        {
        public:
            const_iterator(const PathX &path, int i) : index(i), path(&path) { };
            const_iterator(const PathX &path, int i, bool reverse) : index(i), reverse(reverse), path(&path) { };

            bool last() const { if (reverse){ return index == 0; }else{ return index == (path->size() - 1); }};

            void operator++(int i) { if (reverse){index--;} else{index++;} };
            void operator--(int i);
            bool operator==(const const_iterator& iter) const { return this->index == iter.index; };
            bool operator!=(const const_iterator& iter) const { return path->points[this->index].getIndex() != iter.path->points[iter.index].getIndex(); };
            T operator*() const { return path->points[index]; };

            const_iterator& operator=( const const_iterator &iter) {
                this->reverse = iter.reverse; this->path = iter.path; this->index = iter.index; return *this; };
            int index;
            bool reverse = false;

        private:
            const PathX* path;
        };

        const_iterator begin2() const { return const_iterator(*this, 0); };
        const_iterator end2() const {
            return const_iterator(*this, this->size() - 1, true); };

    private:
        Mesh &mesh;
        std::vector <T> points;
    };

    typedef PathX<Point> PathT;

    template <typename T>
    std::ostream& operator<< (std::ostream& stream, const PathX<T>& path)
    {
        stream << "Path:";

        for (Point point : path.getPoints())
        {
            stream << point.getIndex() << ",";
        }

        return stream;
    }
}
