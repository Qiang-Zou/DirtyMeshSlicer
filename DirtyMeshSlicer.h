#ifndef MESH_SLICER_H
#define MESH_SLICER_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Handles.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/System/config.h>
#include <OpenMesh/Core/Mesh/DefaultTriMesh.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cstdlib>

#include <tetwild/tetwild.h>

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readDMAT.h>
#include <igl/writeOBJ.h>
#include <igl/writeDMAT.h>
#include <igl/per_vertex_normals.h>
// #include <igl/marching_cubes.h>

// marching cube
#include <MarchingCube/MC33.h>
#include <MarchingCube/MC33_LookUpTable.h>

using namespace std;
using namespace OpenMesh;

#define INF 1e8

typedef OpenMesh::PolyMesh_ArrayKernelT<>  MyMesh;

struct faceCompare {
    OpenMesh::FaceHandle fh;
    float min_z;
    float max_z;
};

class DirtyMeshSlicer
{
public:
    DirtyMeshSlicer():
    surfaceRepairFlag(true),
    left(1e8 * Eigen::Vector3f::Ones()),
    right(-1e8 * Eigen::Vector3f::Ones()){}

    ~DirtyMeshSlicer();

    bool surfaceRepairFlag;
    int chooseSurfaceRepairFlag;
    string inputF;
    MyMesh mesh;
    std::vector<faceCompare> faces;
    Eigen::Vector3f left;
    Eigen::Vector3f right;
    std::vector<double> layerHeights;
    std::vector<std::vector<std::vector<Eigen::Vector3d> > > layers;
    float layerLevel;
    float layerThick;

    // choose surface repair methods
    void chooseSurfaceRepair(const char*, char*, std::string, std::string);

    // bounding box
    void findBoundingBox(MyMesh& mesh, Eigen::Vector3f& left, Eigen::Vector3f& right, std::vector<faceCompare>& faces);

    // sort
    void meshFaceSort();

    // tetwild
    void tetWildRecon(string inputFileName, string sliceFileName);

    // poisson surface reconstruction
    void poissonRec(string inputFileName, string midFile, string sliceFileName);

    // winding number based reconstruction
    void windingNumberRecon(string inputFileName, string sliceFileName);
    
    // extract the faces in one layer
    void extractFaces(int* cursor, vector<OpenMesh::FaceHandle>& layerFaces);

    // slicing in one layer
    void meshSlicing(MyMesh& mesh, float layerLevel, vector<OpenMesh::FaceHandle>& layerFaces, std::vector<vector<float> >& t,
    std::vector<std::vector<OpenMesh::HalfedgeHandle> >& edgesPerLayer, OpenMesh::FProp<bool>& visited);

    // calculate contour
    void calculateContour(std::vector<vector<float> >& t, std::vector<std::vector<OpenMesh::HalfedgeHandle> >& edgesPerLayer);

    // g-code utilities
    bool writeCLIBinary(std::string path, std::vector<std::vector<std::vector<Eigen::Vector3d> > >& layers,
    Eigen::Vector3f& left, Eigen::Vector3f& right, std::vector<double>& layerHeights);

    // I/O
    void readMesh(MyMesh& mesh, string sliceFileName);

private:
    // I/O
    void write_ply(const std::string& filename, const Eigen::MatrixXd& V, const Eigen::MatrixXd& N);

};

#endif // MESH_SLICER_H
