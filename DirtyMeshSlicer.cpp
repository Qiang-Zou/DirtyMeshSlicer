#include "DirtyMeshSlicer.h"

void DirtyMeshSlicer::chooseSurfaceRepair(const char* inputFileName, char* sliceFileName, string slicePoissonFileName, string slicePoissonFileName2) {
    if (this->surfaceRepairFlag) {
        cout << "MeshSlicer > Please input the surface repair method: 1.tetwild 2.poisson surface reconstruction 3.winding number" << endl;
        cin >> this->chooseSurfaceRepairFlag;
        switch (this->chooseSurfaceRepairFlag) {
            case 1:tetWildRecon(inputFileName, sliceFileName); break;
            case 2:poissonRec(inputFileName, slicePoissonFileName, slicePoissonFileName2);
            strcpy(sliceFileName, "mid2.ply"); break;
            case 3:windingNumberRecon(inputFileName, sliceFileName); break;
            default: break;
        }
    }
}

void checkFaceRobust(MyMesh& mesh, FaceHandle fh, float height) {
    float epsilon = 0.006;
    vector<MyMesh::Point> z;
    vector<VertexHandle> vhVector;
    for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it) {
        MyMesh::Point poi = mesh.point(*fv_it);
        z.push_back(poi);
        vhVector.push_back(*fv_it);
    }
    if (z[0][2] == height && z[0][2] == z[1][2] && z[1][2] == z[2][2]) {
        z[0][2] += epsilon;
        z[1][2] -= epsilon;
        mesh.set_point(vhVector[0], z[0]);
        mesh.set_point(vhVector[1], z[1]);
    }
    else if (z[0][2] == z[1][2] && z[0][2] == height) {
        z[0][2] += epsilon;
        z[1][2] -= epsilon;
        mesh.set_point(vhVector[0], z[0]);
        mesh.set_point(vhVector[1], z[1]);
    } else if (z[1][2] == z[2][2] && z[1][2] == height) {
        z[1][2] += epsilon;
        z[2][2] -= epsilon;
        mesh.set_point(vhVector[1], z[1]);
        mesh.set_point(vhVector[2], z[2]);
    } else if (z[2][2] == z[0][2] && z[2][2] == height) {
        z[2][2] += epsilon;
        z[0][2] -= epsilon;
        mesh.set_point(vhVector[2], z[2]);
        mesh.set_point(vhVector[0], z[0]);
    } else if (z[0][2] == height && z[1][2] > height && z[2][2] > height) {
        z[0][2] -= epsilon;
        mesh.set_point(vhVector[0], z[0]);
    } else if (z[0][2] == height && z[1][2] < height && z[2][2] < height) {
        z[0][2] += epsilon;
        mesh.set_point(vhVector[0], z[0]);
    } else if (z[1][2] == height && z[0][2] > height && z[2][2] > height) {
        z[1][2] -= epsilon;
        mesh.set_point(vhVector[1], z[1]);
    } else if (z[1][2] == height && z[0][2] < height && z[2][2] < height) {
        z[1][2] += epsilon;
        mesh.set_point(vhVector[1], z[1]);
    } else if (z[2][2] == height && z[0][2] > height && z[1][2] > height) {
        z[2][2] -= epsilon;
        mesh.set_point(vhVector[2], z[2]);
    } else if (z[2][2] == height && z[0][2] < height && z[1][2] < height) {
        z[2][2] += epsilon;
        mesh.set_point(vhVector[2], z[2]);
    }
}

int findUnvisitedFace(OpenMesh::FProp<bool>& visited, vector<OpenMesh::FaceHandle>& layerFaces) {
    int cur = 0;
    while (cur<layerFaces.size()) {
        if (visited[layerFaces[cur]] == false) {
            break;
        }
        cur++;
    }
    if (cur == layerFaces.size()) {
        return -1;
    }
    return cur;
}

void DirtyMeshSlicer::extractFaces(int* cursorPtr, vector<OpenMesh::FaceHandle>& layerFaces) {
    while ((*cursorPtr)<this->faces.size() && this->faces[(*cursorPtr)].min_z<this->layerLevel && this->faces[(*cursorPtr)].max_z<this->layerLevel) {
        (*cursorPtr)++;
    }
    int tempCursor = (*cursorPtr);
    while (tempCursor<this->faces.size() && this->faces[tempCursor].min_z<=this->layerLevel) {
        if (this->faces[tempCursor].max_z>=this->layerLevel) {
            layerFaces.push_back(this->faces[tempCursor].fh);
        }
        tempCursor++;
    }
}


void DirtyMeshSlicer::meshSlicing(MyMesh& mesh, float layerLevel, vector<OpenMesh::FaceHandle>& layerFaces, std::vector<vector<float> >& t,
std::vector<std::vector<OpenMesh::HalfedgeHandle> >& edgesPerLayer, OpenMesh::FProp<bool>& visited) {
    for(int i=0;i<layerFaces.size();i++) {
        visited[layerFaces[i]] = false;
    }
    int id;
    vector<OpenMesh::HalfedgeHandle> sliceEdges;
    vector<float> tTemp;
    bool errorFlag = false;
    while (!errorFlag && ((id = findUnvisitedFace(visited, layerFaces)) != -1)) {
        sliceEdges.clear();
        tTemp.clear();
        OpenMesh::FaceHandle fh = layerFaces[id];
        int originFaceIdx = fh.idx();
        checkFaceRobust(mesh, fh, layerLevel);
        visited[fh] = true;
        OpenMesh::FaceHandle next = fh;
        OpenMesh::FaceHandle nextNext;
        for (MyMesh::FaceHalfedgeIter fh_it = mesh.fh_iter(fh); fh_it.is_valid(); ++fh_it) {
            MyMesh::HalfedgeHandle heh = fh_it.handle();
            MyMesh::VertexHandle vh1 = mesh.from_vertex_handle(heh);
            MyMesh::VertexHandle vh2 = mesh.to_vertex_handle(heh);
            float z1 = mesh.point(vh1)[2];
            float z2 = mesh.point(vh2)[2];
            if (z1 >= layerLevel && z2 < layerLevel) {
                sliceEdges.push_back(heh);
                float t0 = (z1-layerLevel)/(z1-z2);
                tTemp.push_back(t0);
                MyMesh::HalfedgeHandle heh2 = mesh.next_halfedge_handle(heh);
                float z3 = mesh.point(mesh.from_vertex_handle(heh2))[2];
                float z4 = mesh.point(mesh.to_vertex_handle(heh2))[2];
                if (z3 < layerLevel && z4 >= layerLevel) {
                    t0 = (layerLevel-z3)/(z4-z3);
                    tTemp.push_back(t0);
                    MyMesh::HalfedgeHandle opposite_heh = mesh.opposite_halfedge_handle(heh2);
                    if (mesh.is_valid_handle(opposite_heh)) {
                        MyMesh::FaceHandle fh_opposite = mesh.face_handle(opposite_heh);
                        next = fh_opposite;
                    } else {
                        cout << "MeshSlicer >opposite edge handle error" << endl;
                    }
                } else {
                    heh2 = mesh.next_halfedge_handle(heh2);
                    z3 = mesh.point(mesh.from_vertex_handle(heh2))[2];
                    z4 = mesh.point(mesh.to_vertex_handle(heh2))[2];
                    t0 = (layerLevel-z3)/(z4-z3);
                    tTemp.push_back(t0);
                    MyMesh::HalfedgeHandle opposite_heh = mesh.opposite_halfedge_handle(heh2);
                    if (mesh.is_valid_handle(opposite_heh)) {
                        MyMesh::FaceHandle fh_opposite = mesh.face_handle(opposite_heh);
                        next = fh_opposite;
                    } else {
                        cout << "MeshSlicer >opposite edge handle error" << endl;
                    }
                }
                sliceEdges.push_back(heh2);
                break;
            }
        }
        if (next.idx() == -1) {
            cout << "MeshSlicer > error 1 occurs in layer height: " << layerLevel << endl;
            errorFlag = true;
            break;
        }
        nextNext = next;
        int count = 0;
        while (count <= mesh.n_faces() && next != fh) {
            count++;
            checkFaceRobust(mesh, next, layerLevel);
            visited[next] = true;
            for (MyMesh::FaceHalfedgeIter fh_it = mesh.fh_iter(next); fh_it.is_valid(); ++fh_it) {
                MyMesh::HalfedgeHandle heh = fh_it.handle();
                MyMesh::VertexHandle vh1 = mesh.from_vertex_handle(heh);
                MyMesh::VertexHandle vh2 = mesh.to_vertex_handle(heh);
                float z1 = mesh.point(vh1)[2];
                float z2 = mesh.point(vh2)[2];
                if (z1 < layerLevel && z2 >= layerLevel) {
                    float t1 = (layerLevel-z1)/(z2-z1);
                    tTemp.push_back(t1);
                    MyMesh::HalfedgeHandle opposite_heh = mesh.opposite_halfedge_handle(heh);
                    if (mesh.is_valid_handle(opposite_heh)) {
                        MyMesh::FaceHandle fh_opposite = mesh.face_handle(opposite_heh);
                        next = fh_opposite;
                    } else {
                        cout << "MeshSlicer > opposite edge handle error" << endl;
                    }
                    sliceEdges.push_back(heh);
                    break;
                }
            }
            if (next == nextNext) {
                cout << "MeshSlicer > error" << endl;
                break;
            }
            if (next.idx() == -1) {
                cout << "MeshSlicer > error 2 occurs in layer height: " << layerLevel << endl;
                errorFlag = true;
                break;
            }
        }
        if (count <= mesh.n_faces() && !errorFlag && sliceEdges.size() > 0) {
            edgesPerLayer.push_back(sliceEdges);
            t.push_back(tTemp);
        } else if (count > mesh.n_faces()) {
            cout << "MeshSlicer > circle number too much" << endl;
        }
    }
}

void DirtyMeshSlicer::calculateContour(std::vector<vector<float> >& t, std::vector<std::vector<OpenMesh::HalfedgeHandle> >& edgesPerLayer) {
    std::vector<std::vector<Eigen::Vector3d> > layersTemp;
    for (int i=0;i<t.size();i++) {
        std::vector<Eigen::Vector3d> multiHole;
        for (int j=0;j<t[i].size();j++) {
            MyMesh::VertexHandle vh1 = mesh.from_vertex_handle(edgesPerLayer[i][j]);
            MyMesh::VertexHandle vh2 = mesh.to_vertex_handle(edgesPerLayer[i][j]);
            MyMesh::Point p1 = mesh.point(vh1);
            MyMesh::Point p2 = mesh.point(vh2);
            MyMesh::Point p3 = p2-p1;
            MyMesh::Point pMid = p1 + t[i][j] * p3;
            Eigen::Vector3d p;
            p(0) = pMid[0];
            p(1) = pMid[1];
            p(2) = pMid[2];
            multiHole.push_back(p);
        }
        layersTemp.push_back(multiHole);
    }
    this->layers.push_back(layersTemp);
}

void DirtyMeshSlicer::findBoundingBox(MyMesh& mesh, Eigen::Vector3f& left, Eigen::Vector3f& right, std::vector<faceCompare>& faces) {
    for (OpenMesh::TriMesh_ArrayKernelT<>::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
        float min_x = INF;
        float min_y = INF;
        float min_z = INF;
        float max_x = (-1)*INF;
        float max_y = (-1)*INF;
        float max_z = (-1)*INF;
        for (MyMesh::FaceVertexIter it = mesh.fv_begin(*f_it); it.is_valid(); ++it) {
            MyMesh::Point myPoint = mesh.point(*it);
            if (min_x > myPoint[0]) {
                min_x = myPoint[0];
            }
            if (min_y > myPoint[1]) {
                min_y = myPoint[1];
            }
            if (min_z > myPoint[2]) {
                min_z = myPoint[2];
            }
            if (max_x < myPoint[0]) {
                max_x = myPoint[0];
            }
            if (max_y < myPoint[1]) {
                max_y = myPoint[1];
            }
            if (max_z < myPoint[2]) {
                max_z = myPoint[2];
            }
        }
        faceCompare fc;
        fc.fh = f_it.handle();
        fc.min_z = min_z;
        fc.max_z = max_z;
        faces.push_back(fc);
        left(0) = std::min(left(0), min_x);
        left(1) = std::min(left(1), min_y);
        left(2) = std::min(left(2), min_z);
        right(0) = std::max(right(0), max_x);
        right(1) = std::max(right(1), max_y);
        right(2) = std::max(right(2), max_z);
    }
}
void DirtyMeshSlicer::tetWildRecon(string inputFileName, string sliceFileName) {
    cout << "MeshSlicer > execute tetwild reconstruction" << endl;
    Eigen::MatrixXd v_in;
    Eigen::MatrixXi f_in;
    Eigen::MatrixXd v_out;
    Eigen::MatrixXi t_out;
    Eigen::VectorXd a_out;

    Eigen::MatrixXd v_s;
    Eigen::MatrixXi f_s;
    igl::readOBJ(inputFileName, v_in, f_in);
    tetwild::Args args;
    tetwild::tetrahedralization(v_in, f_in, v_out, t_out, a_out, args);
    tetwild::extractSurfaceMesh(v_out, t_out, v_s, f_s);
    igl::writeOBJ(sliceFileName, v_s, f_s);
}

void DirtyMeshSlicer::poissonRec(string inputFileName, string midFile, string slicePoissonFileName) {
    cout << "MeshSlicer > execute poisson surface reconstruction ..." << endl;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!igl::readOBJ(inputFileName, V, F)) {  
        std::cerr << "MeshSlicer > Failed to read .obj file " << inputFileName << std::endl;  
        return;
    }
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);// average normal
    write_ply(midFile, V, N);

    // string poissonPath = "../../PoissonRecon/Bin/Linux/PoissonRecon";
    string poissonPath = "../../PoissonRecon/Bin/Linux/SSDRecon";
    string execString = poissonPath+" --in "+midFile+" --out "+slicePoissonFileName;
    int result = std::system(execString.c_str());
    if (result == 0) {
        std::cout << "MeshSlicer > Command: " << execString << endl;
        std::cout << "MeshSlicer > Command executed successfully\n";  
    } else {  
        std::cerr << "MeshSlicer > Command execution failed\n";  
    }
}

void DirtyMeshSlicer::meshFaceSort() {
    std::sort(this->faces.begin(), this->faces.end(), [](faceCompare& lhs, faceCompare& rhs){
        if (lhs.min_z != rhs.min_z) {
            return lhs.min_z < rhs.min_z;
        }
        return lhs.max_z < rhs.max_z;
    });
}

void DirtyMeshSlicer::write_ply(const std::string& filename,  
               const Eigen::MatrixXd& V,  
               const Eigen::MatrixXd& N) {  
    std::ofstream plyFile(filename, std::ios::out | std::ios::binary);  
    if (!plyFile.is_open()) {  
        std::cerr << "Cannot open file " << filename << std::endl;  
        return;  
    }  
    plyFile << "ply" << std::endl;  
    plyFile << "format ascii 1.0" << std::endl;  
    plyFile << "comment Generated by my program" << std::endl;  
    plyFile << "element vertex " << V.rows() << std::endl;  
    plyFile << "property float x" << std::endl;  
    plyFile << "property float y" << std::endl;  
    plyFile << "property float z" << std::endl;  
    plyFile << "property float nx" << std::endl;  
    plyFile << "property float ny" << std::endl;  
    plyFile << "property float nz" << std::endl;  
    plyFile << "end_header" << std::endl;  
  
    for (int i = 0; i < V.rows(); ++i) {  
        plyFile << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << " ";  
        plyFile << N(i, 0) << " " << N(i, 1) << " " << N(i, 2) << std::endl;  
    }  
  
    plyFile.close();  
}

void DirtyMeshSlicer::readMesh(MyMesh& mesh, string sliceFileName) {
    cout << "MeshMeshSlicer > Start to read mesh " << sliceFileName << endl;
    if (!OpenMesh::IO::read_mesh(mesh, sliceFileName)) {
        std::cerr << "MeshSlicer > Read mesh error\n";
        exit(1);
    }
    cout << "MeshMeshSlicer > Read mesh " << sliceFileName << " ok!" << endl;
    // if (HasDuplicatedVertex(mesh)) {
    //     RemoveDuplicatedVertex(mesh);
    //     cout << HasDuplicatedVertex(mesh) << endl;
    // }
}

void DirtyMeshSlicer::windingNumberRecon(string inputFileName, string sliceFileName) {
    cout << "MeshSlicer > Start winding number based reconstruction ..." << endl;
    /* compute the query points */
    std::vector<std::vector<std::vector<Eigen::Vector3f> > > pos;
    // read the original mesh
    MyMesh newMesh;
    readMesh(newMesh, inputFileName);
    // find bounding box
    std::vector<faceCompare> faces;
    Eigen::Vector3f left(1e8 * Eigen::Vector3f::Ones()), right(-1e8 * Eigen::Vector3f::Ones());
    findBoundingBox(newMesh, left, right, faces);
    // get the resolution
    double windingNumberResolution = 0.01;
    cout << "The size of the bounding box is " << std::to_string(right[0]-left[0]) << " * " << std::to_string(right[1]-left[1])
    << " * " << std::to_string(right[2]-left[2]) << ". Please input the resolution:" << endl;
    cin >> windingNumberResolution;

    // compute the grid parameters
    double lengthX = (right[0]-left[0])*1.2;
    double lengthY = (right[1]-left[1])*1.2;
    double lengthZ = (right[2]-left[2])*1.2;
    int nx = std::ceil(lengthX/windingNumberResolution);
    int ny = std::ceil(lengthY/windingNumberResolution);
    int nz = std::ceil(lengthZ/windingNumberResolution);
    double origin[3];
    origin[0] = left[0]-(right[0]-left[0])*0.1;
    origin[1] = left[1]-(right[1]-left[1])*0.1;
    origin[2] = left[2]-(right[2]-left[2])*0.1;
    for (int k=0;k<nz;k++) {
        std::vector<std::vector<Eigen::Vector3f> > tempz;
        for (int j=0;j<ny;j++) {
            std::vector<Eigen::Vector3f> tempy;
            for (int i=0;i<nx;i++) {
                Eigen::Vector3f tempx;
                tempx(0) = origin[0]+i*windingNumberResolution;
                tempx(1) = origin[1]+j*windingNumberResolution;
                tempx(2) = origin[2]+k*windingNumberResolution;
                tempy.push_back(tempx);
            }
            tempz.push_back(tempy);
        }
        pos.push_back(tempz);
    }

    // write the query points to a .dmat file
    std::vector<std::vector<float> > wa;
    for (int k=0;k<nz;k++) {
        for (int j=0;j<ny;j++) {
            for (int i=0;i<nx;i++) {
                std::vector<float> tempw;
                tempw.push_back(pos[k][j][i](0));
                tempw.push_back(pos[k][j][i](1));
                tempw.push_back(pos[k][j][i](2));
                wa.push_back(tempw);
            }
        }
    }
    igl::writeDMAT("WNinput.dmat", wa);

    cout << "MeshSlicer > Computing winding number ..." << endl;
    string wnPath = "../../fast-winding-number-soups/build/fastwinding";
    string execString = wnPath + " " + inputFileName + " WNinput.dmat WNoutput.dmat";
    int result = std::system(execString.c_str());
    // read the output file
    std::vector<std::vector<float> > windingNumbers;
    igl::readDMAT("WNoutput.dmat", windingNumbers);
    
    // set the grid of marching cube
    cout << "MeshSlicer > Start marching cube ..." << endl;
    grid3d myPointGrid;
    myPointGrid.set_grid_dimensions(nx, ny, nz);
    myPointGrid.set_ratio_aspect(windingNumberResolution, windingNumberResolution, windingNumberResolution);
    myPointGrid.set_r0(origin[0], origin[1], origin[2]);
    for (int k=0;k<nz;k++) {
        for (int j=0;j<ny;j++) {
            for (int i=0;i<nx;i++) {
                myPointGrid.set_grid_value(i,j,k,windingNumbers[k*ny*nx+j*nx+i][0]);
            }
        }
    }
    MC33 myMC;
    myMC.set_grid3d(&myPointGrid);
    MC33_real isovalue = 0.5;
    surface* mySurface = myMC.calculate_isosurface(isovalue);
    Eigen::MatrixXd v_s(mySurface->get_num_vertices(), 3);
    Eigen::MatrixXi f_s(mySurface->get_num_triangles(), 3);
    cout << "MeshSlicer > Marching cube over!" << endl;
    // write the surface to .obj file
    for (int i=0;i<mySurface->get_num_vertices();i++) {
        v_s(i,0) = mySurface->getVertex(i)[0];
        v_s(i,1) = mySurface->getVertex(i)[1];
        v_s(i,2) = mySurface->getVertex(i)[2];
    }
    for (int i=0;i<mySurface->get_num_triangles();i++) {
        f_s(i,0) = mySurface->getTriangle(i)[0];
        f_s(i,1) = mySurface->getTriangle(i)[1];
        f_s(i,2) = mySurface->getTriangle(i)[2];
    }
    igl::writeOBJ(sliceFileName, v_s, f_s);
    cout << "MeshSlicer > Write mesh " << sliceFileName << " ok!" << endl;
    cout << "MeshSlicer > Winding number based reconstruction over!" << endl;
}

bool DirtyMeshSlicer::writeCLIBinary(std::string cliPath, std::vector<std::vector<std::vector<Eigen::Vector3d> > >& layers,
Eigen::Vector3f& left, Eigen::Vector3f& right, std::vector<double>& layerHeights) {
    auto writeUInt = [](uint16_t i, ofstream& out) {
        union u {uint16_t i; unsigned char c[2];} ic;
        ic.i = i;
        out.write((char*)ic.c, 2);
    };
    auto areaCal = [](std::vector<Eigen::Vector3d>& points) {
        auto num = points.size();
        double area(0);
        for (size_t i(0); i < num; ++i) {
            area += points[i][0] * points[(i + 1) % num][1] - points[i][1] * points[(i + 1) % num][0];
        }
        return area / 2;
    };
    ofstream outCLIFile;
    outCLIFile.open(cliPath, ios::out | ios::binary);
    if (!outCLIFile.is_open()) {
        std::cerr << "MeshSlicer > error openning binary CLI file" << endl;
        return false;
    }
    // write header
    outCLIFile << "$$HEADERSTART" << std::endl;
    outCLIFile << "$$BINARY" << std::endl << "$$UNITS\/00000000.010000" << std::endl << "$$VERSION\/100" << std::endl;
    outCLIFile << "$$LABEL\/1,part1"<< std::endl << "$$DATE\/070920" << std::endl;
    outCLIFile << "$$DIMENSION\/" << std::fixed << std::setprecision(6) << std::setfill('0') << std::setw(15) <<
               left[0] << "," << std::setfill('0') << std::setw(15) << left[1] << "," << std::setfill('0') << std::setw(15) << left[2] << "," << std::setfill('0') << std::setw(15) <<
               right[0] << "," << std::setfill('0') << std::setw(15) << right[1] << "," << std::setfill('0') << std::setw(15) << right[2] << endl;
    outCLIFile << "$$LAYERS\/" << std::setfill('0') << std::setw(6) << layers.size() << endl;
    outCLIFile << "$$HEADEREND";

    // write geometry
    for (int i=0; i < layers.size(); i++) {
        uint16_t command = 128; // layer start
        writeUInt(command, outCLIFile);
        double layerH = layerHeights[i]-layerHeights[0];// z min = 0
        writeUInt(layerH * 100, outCLIFile);
        uint16_t dir, num, temp(1);
        double area;
        for (int j=0;j<layers[i].size();j++) {
            area = areaCal(layers[i][j]);
            if (abs(area) < 1e-6) continue; // super small regions
            dir = area > 0 ? 1 : 0;
            num = uint16_t(layers[i][j].size());
            command = 129;
            writeUInt(command, outCLIFile);
            writeUInt(temp, outCLIFile);
            writeUInt(dir, outCLIFile);
            writeUInt(num + 1, outCLIFile); // +1 due to p0 = pn
            for (int k=0;k<layers[i][j].size();k++) {
                writeUInt((layers[i][j][k](0)-left[0]) * 100, outCLIFile);
                writeUInt((layers[i][j][k](1)-left[1]) * 100, outCLIFile);
            }
            writeUInt((layers[i][j][0](0)-left[0]) * 100, outCLIFile);
            writeUInt((layers[i][j][0](1)-left[1]) * 100, outCLIFile);
        }        
    }
    outCLIFile.close();
    cout << "MeshSlicer > Write to binary CLI file " << cliPath << " ok! " << endl;
    return true;
}

DirtyMeshSlicer::~DirtyMeshSlicer() {}

