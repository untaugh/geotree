#include "MeshIntersection.h"
#include "IntersectionFace.h"
#include "Calc.h"
#include "Log.h"
#include <iostream>
#include <memory>

namespace Geotree
{
  IntersectionFace::IntersectionFace(FaceT face)
    : FaceT(face) {

      //faceEdgePoints.push_back(FaceEdgePoint(mesh, mesh.F.row(index)(0), 0, 0.0));
      //faceEdgePoints.push_back(FaceEdgePoint(mesh, mesh.F.row(index)(1), 1, 0.0));
      //faceEdgePoints.push_back(FaceEdgePoint(mesh, mesh.F.row(index)(2), 2, 0.0));

  }

    void IntersectionFace::nextPathReverse(PathT& newPath, PathX<FacePoint>::const_iterator &iter)
    {
        bool returnNextPoint = false;

        int i = 0;
        while(true)
        {
            if (--i < 0 )
            {
                i = faceEdgePoints.size() - 1;
            }

            FaceEdgePoint point = faceEdgePoints[i];

            if (returnNextPoint)
            {
                if (point.position > 0.0)
                {
                    for (auto &path : paths)
                    {
                        if (path.index(0) == point.getIndex())
                        {
                            iter = path.begin2();
                            return;
                        }
                        else if (path.index(path.size() - 1) == point.getIndex())
                        {
                            iter = path.end2();
                            return;
                        }
                    }
                    throw;
                }
                else
                {
                    newPath.addExisting(point.getIndex());
                }
            }

            Point iterpoint = *iter;
            if (iterpoint.getIndex() == point.getIndex())
            {
                returnNextPoint = true;
            }
        }

        throw;
    }

    PathX<FacePoint>::const_iterator IntersectionFace::nextPath(PathT& newPath, PathX<FacePoint>::const_iterator &iter)
    {
        bool returnNextPoint = false;

        unsigned int i = 0;
        while(true)
        {
            if (++i >= faceEdgePoints.size())
            {
                i = 0;
            }
            FaceEdgePoint point = faceEdgePoints[i];

            if (returnNextPoint)
            {
                if (point.position > 0.0)
                {
                    for (auto &path : paths)
                    {
                        if (path.index(0) == point.getIndex())
                        {
                            iter = path.begin2();
                            return path.begin2();
                        }
                        else if (path.index(path.size() - 1) == point.getIndex())
                        {
                            iter = path.end2();
                            return path.end2();
                        }
                    }
                    throw;
                }
                else
                {
                    newPath.addExisting(point.getIndex());
                }
            }

            Point iterpoint = *iter;
            if (iterpoint.getIndex() == point.getIndex())
            {
                returnNextPoint = true;
            }
        }

        throw;
    }

    PathT IntersectionFace::completePathReverse(const PathX<FacePoint> path)
    {
        PathT newPath(mesh);

        PathX<FacePoint>::const_iterator path_iter = path.begin2();

        do {
            newPath.addExisting(*path_iter);

            if (path_iter.last() && (*path_iter).type == SEGMENT)
            {
                nextPathReverse(newPath, path_iter);
            }
            else if (path_iter.last() && (*path_iter).type == FACE)
            {
                path_iter = path.begin2();
            }
            else
            {
                path_iter++;
            }
        } while (path_iter != path.begin2());

        return newPath;
    }

    PathT IntersectionFace::completePathForward(const PathX<FacePoint> path)
    {
        PathT newPath(mesh);

        PathX<FacePoint>::const_iterator path_iter = path.begin2();

        do {
            newPath.addExisting(*path_iter);

            if (path_iter.last() && (*path_iter).type == SEGMENT) {
                path_iter = nextPath(newPath, path_iter);
            }
            else if (path_iter.last() && (*path_iter).type == FACE)
            {
                path_iter = path.begin2();
            }
            else
            {
                path_iter++;
            }
        } while (path_iter != path.begin2());

        return newPath;
    }

    void IntersectionFace::completePaths() {

        bool hasEdgePath = false;

        for (PathX<FacePoint> &path : paths)
        {
            if (path.edgeToEdge()) {
                hasEdgePath = true;

                PathT newPath = completePathForward(path);

                if (completeFacePaths.size()) {
                    std::vector<PathT>::iterator iter = std::find(completeFacePaths.begin(), completeFacePaths.end(),
                                                                  newPath);

                    if (iter == completeFacePaths.end()) {
                        completeFacePaths.push_back(newPath);
                    }
                } else {
                    completeFacePaths.push_back(newPath);
                }

                PathT newPath2 = completePathReverse(path);

                if (std::find(completeFacePaths.begin(), completeFacePaths.end(), newPath2) ==
                    completeFacePaths.end()) {
                    completeFacePaths.push_back(newPath2);
                }
            }
            else
            {
                PathT holePath(mesh);

                for (auto point : path.getPoints())
                {
                    holePath.addExisting(point.getIndex());
                }

                holePaths.push_back(holePath);
            }
        }

        if (!hasEdgePath)
        {
            PathT facePath(mesh);
            //facePath.addExisting(getMeshIndex(0));
            //facePath.addExisting(getMeshIndex(1));
            //facePath.addExisting(getMeshIndex(2));
            completeFacePaths.push_back(facePath);
        }

        for (auto path: completeFacePaths)
        {
            Geotree::Log().Get(LOG_DEBUG) << "complete path: " << path;
        }

        for (auto path: holePaths)
        {
            Geotree::Log().Get(LOG_DEBUG) << "hole path: " << path;
        }
    }

    void IntersectionFace::cut(PathX<FacePoint> path)
    {
        Geotree::Log().Get(LOG_DEBUG) << "cut: " << path;

        double distance;

        if (path.edgeToEdge()) {
                distance = path.begin().distance(mesh.getPoint(path.begin().segment));
                faceEdgePoints.push_back(FaceEdgePoint(path.begin(), distance));


                distance = path.back().distance(mesh.getPoint(path.back().segment));
                faceEdgePoints.push_back(FaceEdgePoint(path.back(), distance));


            std::sort(faceEdgePoints.begin(), faceEdgePoints.end());

            std::cout << "mesh: " << mesh << std::endl;
        }

        paths.push_back(path);
    }

    void IntersectionFace::triangulate() {

        Geotree::Log().Get(LOG_DEBUG) << "triangulate: " << mesh;

        for (auto path : completeFacePaths)
        {
            Geotree::Log().Get(LOG_DEBUG) << " " << path;

            std::vector <PathX<Point>> holes;

            for (auto hole : holePaths)
            {
                if (path.contains(hole))
                {
                    Geotree::Log().Get(LOG_DEBUG) << " with hole:" << hole;

                    holes.push_back(hole);
                }
            }

            std::set <int> area;
            MatrixX3i triangles = path.triangulate(holes);

            for (int i=0; i<triangles.rows(); i++)
            {
                int triangleIndex = mesh.addFace(triangles.row(i));
                area.insert(triangleIndex);
            }

            areas.push_back(area);
        }


        for (auto path : holePaths)
        {
            Geotree::Log().Get(LOG_DEBUG) << " hole:" << path;

            std::set <int> area;
            MatrixX3i triangles = path.triangulate();

            for (int i=0; i<triangles.rows(); i++)
            {
                int triangleIndex = mesh.addFace(triangles.row(i));
                area.insert(triangleIndex);
            }

            areas.push_back(area);
        }
    }
}
