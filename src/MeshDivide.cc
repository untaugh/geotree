#include "MeshDivide.h"
#include <set>
#include "Paths.h"
#include <string>

namespace Geotree
{
  MeshDivide::MeshDivide(const Paths &paths)
    : mesh0(paths.intersection.mesh0),
      mesh1(paths.intersection.mesh1),
      paths(paths)
  {
    divide(MESH0);
    divide(MESH1);

    overlap();
  }

  bool MeshDivide::contains(int face, std::set<int> faces)
  {
    return faces.find(face) != faces.end();
  }

  std::set<int> MeshDivide::connectedfaces(MeshID meshid, int face)
  {
    std::vector <Face> meshfaces;
    std::set<int> pathfacesindex = paths.faces(meshid);
    std::vector <Face> pathfaces;
    const Mesh *mesh;
    
    if (meshid == MESH0)
      {
	mesh = &mesh0;
      }
    else
      {
	mesh = &mesh1;
      }

    mesh->getFaces(meshfaces);

    for (int face : pathfacesindex)
      {
	pathfaces.push_back(mesh->getFace(face));
      }

    std::set<int> group;

    if (contains(face, pathfacesindex))
      {
	return group;
      }
	
    group.insert(face);

    bool addednew = true;
    
    while(addednew)
      {
	addednew = false;

	for (int groupfaceindex : group)
	  {
	    Face groupface = mesh->getFace(groupfaceindex);
	      
	    for(Face meshface : meshfaces)
	      {
		if (contains(meshface.index, group))
		  {
		    continue;
		  }

		if (contains(meshface.index, pathfacesindex))
		  {
		    continue;
		  }
		
		if (groupface.connected(meshface))
		  {
		    group.insert(meshface.index);
		    addednew = true;
		  }
	      }
	  }
      }
    return group;
  }

  bool MeshDivide::inside(MeshID meshid, std::set<int> faces, Cube box)
  {
    const Mesh *mesh;

    if (meshid == MESH0)
      {
	mesh = &mesh0;
      }
    else
      {
	mesh = &mesh1;
      }

    bool inside;
    bool initial = true;

    for (int faceindex : faces)
      {
	Face face = mesh->getFace(faceindex);

	Point p0(face.point0);
	Point p1(face.point1);
	Point p2(face.point2);

	//std::string strinside = inside ? " inside " : " not inside ";
	//std::cout << p0 << p1 << p2 << std::endl << strinside << box << std::endl;
	
	if (initial)
	  {
	    std::cout << "initial" << std::endl;
	    inside = p0.inside(box);
	    initial = false;
	  }
	else if (inside != p0.inside(box))
	  {
	    std::cout << "inside mismatch0" << std::endl;
	  }

	if (inside != p1.inside(box))
	  {
	    std::cout << "inside mismatch1" << std::endl;
	  }

	if (inside != p2.inside(box))
	  {
	    std::cout << "inside mismatch2" << std::endl;
	  }
      }

    return inside;
  }
  
  bool MeshDivide::overlap()
  {
    Cube box0A = mesh0.boundingBox(facesMesh0A);
    Cube box0B = mesh0.boundingBox(facesMesh0B);
    Cube box1A = mesh1.boundingBox(facesMesh1A);
    Cube box1B = mesh1.boundingBox(facesMesh1B);

    bool inside0A1A = inside(MESH0, facesMesh0A, box1A);
    bool inside0A1B = inside(MESH0, facesMesh0A, box1B);
    bool inside0B1A = inside(MESH0, facesMesh0B, box1A);
    bool inside0B1B = inside(MESH0, facesMesh0B, box1B);

    bool inside1A0A = inside(MESH1, facesMesh1A, box0A);
    bool inside1A0B = inside(MESH1, facesMesh1A, box0B);
    bool inside1B0A = inside(MESH1, facesMesh1B, box0A);
    bool inside1B0B = inside(MESH1, facesMesh1B, box0B);

    std::cout << "0A " << facesMesh0A.size() << std::endl; 
    std::cout << "0B " << facesMesh0B.size() << std::endl;
    std::cout << "1A " << facesMesh1A.size() << std::endl;
    std::cout << "1B " << facesMesh1B.size() << std::endl; 

    std::cout << "0A inside 1A " << inside0A1A << std::endl; 
    std::cout << "0A inside 1B " << inside0A1B << std::endl;
    std::cout << "0B inside 1A " << inside0B1A << std::endl;
    std::cout << "0B inside 1B " << inside0B1B << std::endl; 

    std::cout << "1A inside 0A " << inside1A0A << std::endl; 
    std::cout << "1A inside 0B " << inside1A0B << std::endl;
    std::cout << "1B inside 0A " << inside1B0A << std::endl;
    std::cout << "1B inside 0B " << inside1B0B << std::endl;    

    if (inside0A1A && inside0A1B) throw;
    if (inside0B1A && inside0B1B) throw;
    if (inside1A0A && inside1A0B) throw;
    if (inside1B0A && inside1B0B) throw;
    
    inside0A = inside0A1A || inside0A1B;
    inside0B = inside0B1A || inside0B1B;
    inside1A = inside1A0A || inside1A0B;
    inside1B = inside1B0A || inside1B0B;

    return true;
  }
  
  void MeshDivide::divide(MeshID mesh)
  {
    std::vector <std::set <int>> facegroups;

    std::set<int> pathfacesindex = paths.faces(mesh);

    std::vector <Face> meshfaces;

    std::set <int> *facesMeshA;
    std::set <int> *facesMeshB;
    
    if (mesh == MESH0)
      {
	mesh0.getFaces(meshfaces);
	facesMeshA = &facesMesh0A;
	facesMeshB = &facesMesh0B;
      }
    else
      {
	mesh1.getFaces(meshfaces);
	facesMeshA = &facesMesh1A;
	facesMeshB = &facesMesh1B;
      }

    for (Face face : meshfaces)
      {
	bool hasface = false;

	if (contains(face.index, pathfacesindex))
	  {
	    continue;
	  }
	
	for (std::set <int> group : facegroups)
	  {
	    if (contains(face.index, group))
	      {
		hasface = true;
		break;
	      }
	  }

	if (hasface)
	  {
	    continue;
	  }

	std::set <int> facegroup = connectedfaces(mesh, face.index);
	//std::cout << mesh << " "<< face.index << "size: " << facegroup.size() << std::endl;

	if (facegroup.size() > 0)
	  {
	    facegroups.push_back(facegroup);
	  }
      }

    if (facegroups.size() > 0)
      {
	*facesMeshA = facegroups[0];
      }

    if (facegroups.size() > 1)
      {
	*facesMeshB = facegroups[1];
      }
  }
}
