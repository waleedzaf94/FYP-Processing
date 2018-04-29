#include "ModelBuilder.hpp"

void ModelBuilder::setOutputModel(ModelBuilder::modelInfo & model) {
    this->outputModel = model;
}

void ModelBuilder::printModelInfo(ModelBuilder::modelInfo & model) {
    std::cout << "Vertices: " << model.vertices.size() << std::endl;
    std::cout << "Faces: " << model.faces.size() << std::endl;
    std::cout << "Normals: " << model.normals.size() << std::endl;
}

void ModelBuilder::printModelInfo() {
    this->printModelInfo(this->model);
}

void ModelBuilder::readFile(string filepath, string filetype)
{
    if (filetype == "off")
    {
        this->model = readOffFile(filepath);
        return;
    }
    else if (filetype == "obj")
    {
        this->model = readObjFile(filepath);
        return;
    }
    else if (filetype == "ply")
    {
        this->model = readPlyFile(filepath);
        return;
    }
}

void ModelBuilder::toPwnVector(Pwn_vector & points) {
    if (this->model.vertices.size() != this->model.normals.size()) {
        std::cerr << "Vertices and Normals should be the same in number\n";
        return;
    }
    points.clear();
    vertex_vector vertices = this->model.vertices;
    normal_vector normals = this->model.normals;
    for (int i=0; i<vertices.size(); i++) {
        vertexInfo v = vertices[i];
        normalInfo n = normals[i];
        Point_3 p (v.x, v.y, v.z);
        Vector_3 vn (n.nx, n.ny, n.nz);
        Point_with_normal pwn (p, vn);
        points.push_back(pwn);
    }
    
}

void ModelBuilder::writeFile(string filepath, string filetype)
{
    if (filetype == "off")
    {
        std::cout << "Writing off file" << std::endl;
        writeOffFile(filepath, outputModel);
        return;
    }
    else if (filetype == "obj")
    {
        std::cout << "Writing obj file" << std::endl;
        writeObjFile(filepath, outputModel);
        return;
    }
    else if (filetype == "ply")
    {
        std::cout << "Writing ply file" << std:: endl;
        if (points.empty())
        {
            std::cout << "Writing Ply File from ModelInfo" <<std::endl;
            writePlyFile(filepath, outputModel); // Working
        }
        else 
        {
            std::cout << "Writing Ply File from ModelInfo" <<std::endl;
            writePlyPointsAndNormals(filepath);     // Working
        }
        return;
    }
}

ModelBuilder::modelInfo ModelBuilder::readOffFile(string filename) {
    // TODO: Read normals
    std::ifstream file;
    file.open(filename, ios::in);
    
    vector<string> lines;
    face_vector faces;
    vertex_vector verts;
    modelInfo model;
    if (file.is_open()) {
        string l;
        while (getline(file, l))
            lines.push_back(l);
        int i=0;
        long numVertices=0, numFaces=0;
        vector<string> words;
        for (i=0; i<lines.size(); i++) {
            l = lines[i];
            boost::trim(l);
            if (l == "OFF") continue;
            if (l[0] == '#') continue;
            boost::split(words, l, boost::is_any_of(" "));
            numVertices = stof(words[0].c_str());
            numFaces = stof(words[1].c_str());
            break;
        }
        for (int j=i; j<(numVertices+i); j++) {
            l = lines[i];
            boost::trim(l);
            if (l == "") continue;
            boost::split(words, l, boost::is_any_of(" "));
            if (words.size() != 3) continue;
            vertexInfo v;
            v.x = stod(words[0].c_str());
            v.y = stod(words[1].c_str());
            v.z = stod(words[2].c_str());
            i = j;
            model.vertices.push_back(v);
        }
        for (int j=i; j<(numFaces+i); j++) {
            l = lines[i];
            boost::trim(l);
            if (l == "") continue;
            boost::split(words, l, boost::is_any_of(" "));
            faceInfo f;
            for (int k=1; k<words.size(); k++) {
                f.vertexIndices.push_back((size_t)stoi(words[k].c_str()));
            }
            i = j;
            model.faces.push_back(f);
        }
    }
    file.close();
    return model;
}

void ModelBuilder::writeMeshStats(ofstream & out, meshStats stats) {
    out << "# ms" << endl;
    out << "# tsa " << to_string(stats.totalSurfaceArea) << endl;
    out << "# hsa " << to_string(stats.horizontalSurfaceArea) << endl;
    out << "# usa " << to_string(stats.upSurfaceArea) << endl;
    out << "# dsa " << to_string(stats.downSurfaceArea) << endl;
    out << "# wsa " << to_string(stats.wallSurfaceArea) << endl;
    out << "# vcs " << to_string(stats.virtualCeilingSurfaceArea) << endl;
    out << "# vws " << to_string(stats.virtualWallSurfaceArea) << endl;
    out << "# es" << endl;
}

ModelBuilder::meshStats ModelBuilder::readMeshStats(vector<string> lines) {
    ModelBuilder::meshStats stats;
    for (string line: lines) {
        boost::trim(line);
        if (line == "") continue;
        vector<string> words;
        boost::split(words, line, boost::is_any_of(" "));
        if (words.size() < 2) continue;
        string code = words[1];
        boost::trim(code);
        if (code == "es") break;
        if (code == "tsa")
            stats.totalSurfaceArea = stof(words[2]);
        if (code == "hsa")
            stats.horizontalSurfaceArea = stof(words[2]);
        if (code == "usa")
            stats.upSurfaceArea = stof(words[2]);
        if (code == "dsa")
            stats.downSurfaceArea = stof(words[2]);
        if (code == "wsa")
            stats.wallSurfaceArea = stof(words[2]);
        if (code == "vws")
            stats.virtualWallSurfaceArea = stof(words[2]);
        if (code == "vcs")
            stats.virtualCeilingSurfaceArea = stof(words[2]);
    }
    return stats;
}

ModelBuilder::modelInfo ModelBuilder::readObjFile(string filename) {
    std::ifstream file;
    file.open(filename, ios::in);
    vector<string> lines;
    face_vector faces;
    normal_vector normals;
    vertex_vector verts;
    modelInfo model;
    if (file.is_open()) {
        string l;
        while (getline(file, l))
            lines.push_back(l);
        model.stats = readMeshStats(lines);
        for (string line: lines) {
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "));
            if (words[0] == "v") {
                vertexInfo v;
                v.x = stod(words[1].c_str());
                v.y = stod(words[2].c_str());
                v.z = stod(words[3].c_str());
                verts.push_back(v);
            }
            else if (words[0] == "vn") {
                normalInfo n;
                n.nx = stod(words[1].c_str());
                n.ny = stod(words[2].c_str());
                n.nz = stod(words[3].c_str());
                normals.push_back(n);
            }
            else if (words[0] == "f") {
                faceInfo f;
                // defined as vertex/texture/normal indices
                for (int i=1; i<4; i++) {
                    vector<string> w2;
                    boost::split(w2, words[i], boost::is_any_of("/"));
                    f.vertexIndices.push_back((size_t)(stoi(w2[0].c_str())-1));
//                    f.vertexIndices.push_back((size_t)stoi(w2[0].c_str()));
                }
                faces.push_back(f);
            }
        }
        file.close();
    }
    model.faces = faces;
    model.vertices = verts;
    model.normals = normals;
    return model;
}

ModelBuilder::modelInfo ModelBuilder::readPlyFile(string filename) {    
//    points = readPlyToPwn(filename);

    std::ifstream file;
    file.open(filename, ios::in);
    vector<string> lines;
    face_vector faces;
    normal_vector normals;
    vertex_vector verts;
    modelInfo model;
    long vcount=0, fcount = 0;
    bool n = false;
    if (file.is_open()) {
        string line;
        while (getline(file, line))
            lines.push_back(line);
        plyHeader header = readPlyHeader(lines);
        for (element e: header.elements) {
            if (e.name == "vertex") {
                vcount = e.count;
                cout << "Vert count: " << vcount << endl;
                for (property p: e.properties) {
                    if (p.name == "nx"){
                        n = true;
                        break;
                    }
                }
            }
            if (e.name == "face")
                fcount = e.count;
        }
        int eHeader = -1;
        for (int i=0; i<lines.size(); i++) {
            if (lines[i] == "end_header") {
                eHeader = i;
                break;
            }
        }
        int vc = vcount;
        int fc = fcount;
        for (int i = eHeader+1; i < lines.size(); i++) {
            while (vc > 0) {
                vertexInfo v;
                vector<string> words;
                boost::split(words, lines[i], boost::is_any_of(" "));
                v.x = stod(words[0].c_str());
                v.y = stod(words[1].c_str());
                v.z = stod(words[2].c_str());
                if (n) {
                    // Assume nx ny nz immediately follow x y z
                    normalInfo norm;
                    norm.nx = stod(words[3].c_str());
                    norm.ny = stod(words[4].c_str());
                    norm.nz = stod(words[5].c_str());
                    normals.push_back(norm);
                }
                verts.push_back(v);
                i++;
                vc--;
            }
            while (fc > 0) {
                faceInfo f;
                vector<string> words;
                boost::split(words, lines[i], boost::is_any_of(" "));
                for (int i=1; i<=stol(words[0]); i++) {
                    f.vertexIndices.push_back((size_t)stoi(words[i].c_str()));
                }
                faces.push_back(f);
                fc--;
                i++;
            }
        }
        file.close();
        model.faces = faces;
        model.normals = normals;
        model.vertices = verts;
    }
    else {
        cerr << "Error opening file: " << filename << endl;
    }
    return model;
}

ModelBuilder::plyHeader ModelBuilder::readPlyHeader(vector<string> lines) {
    plyHeader header;
    string line;
    for (int i=1; i<lines.size(); i++) {
        line = lines[i];
        if (line == "end_header") {
            break;
        }
        vector<string> words;
        boost::split(words, line, boost::is_any_of(" "));
        if (words[0] == "format") {
            string format = "";
            for (string word: words) {
                if (word == "format") continue;
                format += word;
            }
            header.format = format;
        }
        else if (words[0] == "comment") {
            header.comments.push_back(line);
        }
        else if (words[0] == "element") {
            element elem;
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "));
            elem.name = words[1];
            elem.count = atoi(words[2].c_str());
            int j;
            // Get element properties
            for (j=i+1; j < lines.size(); j++) {
                property p;
                line = lines[j];
                vector<string> ww;
                boost::split(ww, line, boost::is_any_of(" "));
                if (ww[0] != "property")
                    break;
                p.name = ww[ww.size()-1]; // Last word has property name
                string d = "";
                for (int k=1; k < ww.size()-1; k++) {
                    d = d + ww[k] + " ";
                }
                p.dataType = d;
                elem.properties.push_back(p);
            }
            i = j-1;
            header.elements.push_back(elem);
        }
        else {
            cerr << "Error reading ply header\n";
            return header;
        }
    }
    return header;
}

Pwn_vector ModelBuilder::readPlyToPwn(std::string fname) {
    Pwn_vector points;
    points.clear();
    
    // loads point set from a file
    // read_ply_points_and_normals takes an OutputIterator for storing the points
    // and a property_map to store the normal vector with each point
    std::ifstream stream(fname);
    
    if (!stream || !CGAL::read_ply_points_and_normals(stream, std::back_inserter(points), Point_map(), Normal_map())) {
        std::cerr << "Error. Cannot read file." << std::endl;
    }
    
    std::cout << points.size() << " points originally." << std::endl;
    return points;
}

void ModelBuilder::writeObjFile(string filename, modelInfo & model) {
    if (model.vertices.size() > 50000) {
        vector<modelInfo> models;
        splitModels(model, models, 50000);
        writeMultipleToOBJ(filename, models, model.stats);
    }
    else {
        ofstream out(filename);
        if (out.is_open()) {
            if (model.stats.totalSurfaceArea != 0) // Assume stats are useless if totalSurfaceArea is 0
                writeMeshStats(out, model.stats);
            out << "# "<< "vnm" << model.vertices.size() << endl;
            out << "# " << "vnn" <<model.normals.size() << endl;
            out << "# " << "fnm" <<model.faces.size() << endl;
            for (vertexInfo v: model.vertices) {
                out << "v " << v.x << " " << v.y << " " << v.z << endl;
            }
            for (normalInfo n: model.normals) {
                out << "vn " << n.nx << " " << n.ny << " " << n.nz << endl;
            }
            for (faceInfo f: model.faces) {
                out << "f ";
                for (size_t ind: f.vertexIndices) {
                    size_t i = ind+1;
                    out << i << "// ";
//                    out << i << "/" << i << "/" << i << " ";
//                    out << i << "/" << i << "/" << i << " ";
                }
                out << endl;
            }
            out.close();
        }
        else {
            cout << "Error opening file: " << filename << endl;
        }
    }
}

void ModelBuilder::writeMultipleToOBJ(string filename, vector<modelInfo> & models, meshStats & stats) {
    ofstream out(filename);
    if (stats.totalSurfaceArea != 0)
        writeMeshStats(out, stats);
    for (int i=0; i<models.size(); i++) {
        if (models[i].vertices.size() == 0)
            continue;
        out << "o Object." << (i+1) << endl;
        for (vertexInfo v: models[i].vertices) {
            out << "v " << v.x << " " << v.y << " " << v.z << endl;
        }
        for (normalInfo n: models[i].normals) {
            out << "vn " << n.nx << " " << n.ny << " " << n.nz << endl;
        }
        for (faceInfo f: models[i].faces) {
            out << "f ";
            for (size_t ind: f.vertexIndices) {
                size_t i = ind+1;
//                out << ind << "/" << ind << "/" << ind << " ";
//                out << i << "/" << i << "/" << i << " ";
                out << i << "// ";
            }
            out << endl;
        }
    }
}

void ModelBuilder::splitModels(modelInfo & model, vector<modelInfo> & models, int split) {
    int nModels = (model.vertices.size() / split) + 1;
    models.clear();
    for (int i=0; i<nModels; i++) {
        modelInfo m;
        models.push_back(m);
    }
    for (int i = 0; i<model.vertices.size(); i++) {
        models[i / split].vertices.push_back(model.vertices[i]);
    }
    for (int i=0; i<model.normals.size(); i++) {
        models[i / split].normals.push_back(model.normals[i]);
    }
    for (int i=0; i<model.faces.size(); i++) {
        faceInfo f = model.faces[i];
        int testIndex = f.vertexIndices[0] / split;
        bool valid = true;
        for (size_t t: f.vertexIndices) {
            int low = testIndex * split;
            int high = (testIndex + 1) * split;
            if ((t < low) || (t >= high)) {
                valid = false;
                break;
            }
        }
        if (valid)
            models[testIndex].faces.push_back(f);
    }
}

void ModelBuilder::writePlyFile(string filename, modelInfo & model) {
    ofstream out(filename);
    if (out.is_open()) {
        //write header
        writePlyHeader(out, model);
        vertex_vector vertices = model.vertices;
        face_vector faces = model.faces;
        normal_vector normals = model.normals;
        if (vertices.size() > 0) {
            if (normals.size() > 0) {
                for (int i=0; i<vertices.size(); i++) {
                    vertexInfo v = vertices[i];
                    normalInfo n = normals[i];
                    out << v.x << " " << v.y << " " << v.z << " " << n.nx << " " << n.ny << " " << n.nz << endl;
                }
            }
            else {
                for (vertexInfo v: vertices) {
                    out << v.x << " " << v.y << " " << v.z << endl;
                }
            }
        }
        if (faces.size() > 0) {
            for (faceInfo f: faces) {
                out << f.vertexIndices.size();
                for (int i: f.vertexIndices) {
                    out << " " << i;
                }
                out << endl;
            }
        }
        out.close();
    }
    else {
        cout << "Error opening out file: " << filename << endl;
    }
}

void ModelBuilder::writeOffFile(string filename, modelInfo & model) {
    // TODO: Write normals
    ofstream out(filename);
    out << "OFF\n";
    out << model.vertices.size() << " " << model.faces.size() << " 0\n";
    for (vertexInfo v: model.vertices) {
        out << v.x << " " << v.y << " " << v.z << endl;
    }
    for (faceInfo f: model.faces) {
        out << f.vertexIndices.size();
        for (int i: f.vertexIndices) {
            out << " " << i;
        }
        out << endl;
    }
    out.close();
}

void ModelBuilder::writePlyHeader(ofstream &out, modelInfo model) {
    out << "ply\n";
    out << "format ascii 1.0\n";
    vertex_vector vertices = model.vertices;
    face_vector faces = model.faces;
    normal_vector normals = model.normals;
    long vc = vertices.size(), fc = faces.size(), nc = normals.size();
    if (vc > 0) {
        out << "element vertex " << vc << endl;
        out << "property double x\n";
        out << "property double y\n";
        out << "property double z\n";
        if (nc > 0) {
            out << "property double nx\n";
            out << "property double ny\n";
            out << "property double nz\n";
        }
    }
    if (fc > 0) {
        out << "element face " << fc << endl;
        out << "property list uchar int vertex_indices\n";
    }
    out << "end_header\n";
}

void ModelBuilder::writePlyHeader(ofstream &out, Pwn_vector& points) {
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << std::distance(points.begin(), points.end()) << std::endl;
    out << "property double x\n";
    out << "property double y\n";
    out << "property double z\n";
    out << "property double nx\n";
    out << "property double ny\n";
    out << "property double nz\n";
    out << "end_header\n";
}

bool ModelBuilder::writePlyPointsAndNormals (Pwn_vector& points, std::string fname) {
    std::ofstream out(fname);
    if (!out) {
        std::cerr << "Error writing file: " << fname << std::endl;
        return false;
    }
    writePlyHeader(out, points);
    for (auto it = points.begin(); it != points.end(); it++) {
        Kernel::Point_3 point = it->first;
        Kernel::Vector_3 normal = it->second;
        out << point.x() << " " << point.y() << " " << point.z() << " " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
    }
    return true;
}

bool ModelBuilder::writePlyPointsAndNormals (std::string fname) {
    std::ofstream out(fname);
    if (!out) {
        std::cerr << "Error writing file: " << fname << std::endl;
        return false;
    }
    // PLY Header
    writePlyHeader(out, points);    

    //Vertices
    for (auto it = points.begin(); it != points.end(); it++) {
        Kernel::Point_3 point = it->first;
        Kernel::Vector_3 normal = it->second;
        out << point.x() << " " << point.y() << " " << point.z() << " " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
    }
    return true;
}

// Converters
void ModelBuilder::toPointAndFacetVectors(PointVector & points, FacetVector &faces) {
    for (ModelBuilder::vertexInfo v: model.vertices) {
        points.push_back(Point_3(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)));
    }
    for (ModelBuilder::faceInfo f: model.faces) {
        FacetIndices indices;
//        bool valid = true;
        for (int i: f.vertexIndices) {
//            if (i > (model.vertices.size() -1)) valid = false;
            indices.push_back(i);
        }
//        if (valid)
            faces.push_back(indices);
    }
}

void ModelBuilder::toPolyhedron(Polyhedron_3 & poly) {
    // TODO: Preserve normal information in Polyhedron
    PointVector points;
    FacetVector faces;
    toPointAndFacetVectors(points, faces);
    polyhedron_builder<HalfedgeDS> builder(points, faces);
    poly.delegate(builder);
}

