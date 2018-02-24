//
//  processingIO.cpp
//  processing
//
//  Created by Waleed Zafar on 22/2/2018.
//

#include "processingIO.hpp"

using namespace std;

modelInfo readPlyFile(string filename) {
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
                v.x = stof(words[0]);
                v.y = stof(words[1]);
                v.z = stof(words[2]);
                if (n) {
                    // Assume nx ny nz immediately follow x y z
                    normalInfo norm;
                    norm.nx = stof(words[3]);
                    norm.ny = stof(words[4]);
                    norm.nz = stof(words[5]);
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
                f.viCount = stol(words[0]);
                for (int i=1; i<=f.viCount; i++) {
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

plyHeader readPlyHeader(vector<string> lines) {
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









