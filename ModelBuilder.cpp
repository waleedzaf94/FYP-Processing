#include "ModelBuilder.hpp"

void ModelBuilder::SetOutputModel(ModelBuilder::modelInfo & model)
{
    outputModel = model;
}

void ModelBuilder::PrintModelInfo(ModelBuilder::modelInfo & model)
{
    vertex_vector vertices = model.vertices;
    face_vector faces = model.faces;
    normal_vector normals = model.normals;
    long vc = vertices.size(), fc = faces.size(), nc = normals.size();
    std::cout << "Vertices " << vc << std::endl;
    std::cout << "Faces " << fc << std::endl;
    std::cout << "Normals " << nc << std::endl;
}
// This is the interface for the readFiles
//  File Path 
//  File Type ["obj", "off", "ply"]
void ModelBuilder::readFile(string filpath, string filetype)
{
    if (filetype == "off")
    {
        modelinf = readOffFile(filpath);
        return;
    }
    else if (filetype == "obj")
    {
        cout << "Inside Model Builder Reader";
        modelinf = readObjFile(filpath);
        return;
    }
    else if (filetype == "ply")
    {
        modelinf = readPlyFile(filpath);
        return;
    }
}


//  This is the interface for the writeFiles
//  File Path - output filepath
//  File Type ["obj", "off", "ply"]
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


// Throwing stoi exception!
ModelBuilder::modelInfo ModelBuilder::readOffFile(string filename) {
    std::ifstream file;
    file.open(filename, ios::in);
    
    // WTF
    vector<string> lines;
    face_vector faces;
    vertex_vector verts;
    modelInfo model;
    if (file.is_open()) {
        string l;
        while (getline(file, l))
            lines.push_back(l);
        int i=0;
        int numVertices=0, numFaces=0;
        vector<string> words;
        for (i=0; i<lines.size(); i++) {
            l = lines[i];
            boost::trim(l);
            if (l == "OFF") continue;
            if (l[0] == '#') continue;
            boost::split(words, l, boost::is_any_of(" "));
            numVertices = stoi(words[0].c_str());
            numFaces = stoi(words[1].c_str());
            break;
        }
        for (int j=i; j<(numVertices+i); j++) {
            l = lines[i];
            boost::trim(l);
            boost::split(words, l, boost::is_any_of(" "));
            vertexInfo v;
            v.x = stoi(words[0].c_str());
            v.y = stoi(words[1].c_str());
            v.z = stoi(words[2].c_str());
            i = j;
        }
        for (int j=i; j<(numFaces+i); j++) {
            l = lines[i];
            boost::trim(l);
            boost::split(words, l, boost::is_any_of(" " ));
            faceInfo f;
            for (int k=1; k<words.size(); k++) {
                f.vertexIndices.push_back(stoi(words[i].c_str()));
            }
        }
    }
    file.close();
    return model;
}
// Throwing stoi exception!
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
        for (string line: lines) {
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "));
            if (words[0] == "v") {
                vertexInfo v;
                v.x = atof(words[1].c_str());
                v.y = atof(words[2].c_str());
                v.z = atof(words[3].c_str());
                verts.push_back(v);
            }
            else if (words[0] == "vn") {
                normalInfo n;
                n.nx = atof(words[1].c_str());
                n.ny = atof(words[2].c_str());
                n.nz = atof(words[3].c_str());
                normals.push_back(n);
            }
            else if (words[0] == "f") {
                faceInfo f;
                // defined as vertex/texture/normal indices
                for (int i=1; i<4; i++) {
                    vector<string> w2;
                    boost::split(w2, words[i], boost::is_any_of("/"));
                    f.vertexIndices.push_back(atoi(w2[0].c_str()));
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
    points = readPlyToPwn(filename);

    std::ifstream file;
    file.open(filename, ios::in);
    vector<string> lines;
    face_vector faces;
    normal_vector normals;
    vertex_vector verts;
    modelInfo model;
    int vcount=0, fcount = 0;
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
                v.x = atof(words[0].c_str());
                v.y = atof(words[1].c_str());
                v.z = atof(words[2].c_str());
                if (n) {
                    // Assume nx ny nz immediately follow x y z
                    normalInfo norm;
                    norm.nx = atof(words[3].c_str());
                    norm.ny = atof(words[4].c_str());
                    norm.nz = atof(words[5].c_str());
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
                    f.vertexIndices.push_back(atoi(words[i].c_str()));
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
    ofstream out(filename);
    if (out.is_open()) {
        out << "# " << model.vertices.size() << " vertices" << endl;
        for (vertexInfo v: model.vertices) {
            out << "v " << v.x << " " << v.y << " " << v.z << endl;
        }
        out << "# " << model.normals.size() << " vertex normals" << endl;
        for (normalInfo n: model.normals) {
            out << "vn " << n.nx << " " << n.ny << " " << n.nz << endl;
        }
        out << "# " << model.faces.size() << " faces" << endl;
        for (faceInfo f: model.faces) {
            out << "f ";
            for (int i: f.vertexIndices) {
                out << i << " ";
            }
            out << endl;
        }
        out.close();
    }
    else {
        cout << "Error opening file: " << filename << endl;
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
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        if (nc > 0) {
            out << "property float nx\n";
            out << "property float ny\n";
            out << "property float nz\n";
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
    // WTF - This does not follow the convention of the other writePlyHeader 
    writePlyHeader(out, points);    
    //Vertices
    //    std::vector<Point_with_normal>::iterator it = points.begin();
    //    while (
    for (auto it = points.begin(); it != points.end(); it++) {
        Kernel::Point_3 point = it->first;
        Kernel::Vector_3 normal = it->second;
        out << point.x() << " " << point.y() << " " << point.z() << " " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
    }
    return true;
}

// TODO - writePlyPointAndNormals was not declared in this scope 
bool ModelBuilder::writePlyPointsAndNormals (std::string fname) {
    std::ofstream out(fname);
    if (!out) {
        std::cerr << "Error writing file: " << fname << std::endl;
        return false;
    }
    // PLY Header
    writePlyHeader(out, points);    

    //Vertices
    //    std::vector<Point_with_normal>::iterator it = points.begin();
    //    while (
    for (auto it = points.begin(); it != points.end(); it++) {
        Kernel::Point_3 point = it->first;
        Kernel::Vector_3 normal = it->second;
        out << point.x() << " " << point.y() << " " << point.z() << " " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
    }
    return true;
}

// Converters
void ModelBuilder::ToPointAndFacetVectors(PointVector & points, FacetVector &faces) {
    for (ModelBuilder::vertexInfo v: modelinf.vertices) {
        points.push_back(Point_3(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)));
    }
    for (ModelBuilder::faceInfo f: modelinf.faces) {
        FacetIndices indices;
        for (int i: f.vertexIndices) {
            indices.push_back(i);
        }
        faces.push_back(indices);
    }
}

void ModelBuilder::ToPolyhedron(Polyhedron_3 & poly) {
    PointVector points;
    FacetVector faces;
    ToPointAndFacetVectors(points, faces);
    // TODO
    // incrementBuilder(poly, points, faces);
}

