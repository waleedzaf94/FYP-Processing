//
//  processingIO.hpp
//  processing
//
//  Created by Waleed Zafar on 22/2/2018.
//

#ifndef processingIO_hpp
#define processingIO_hpp

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

using namespace std;

struct vertexInfo {
    double x=0, y=0, z=0;
};
struct faceInfo {
    double x=0, y=0, z=0, d=0; // Coefficients for plane equation
    std::vector<int> vertexIndices;
};
struct normalInfo {
    double nx=0, ny=0, nz=0;
};
typedef std::vector<vertexInfo> vertex_vector;
typedef std::vector<normalInfo> normal_vector;
typedef std::vector<faceInfo> face_vector;
struct modelInfo {
    face_vector faces;
    vertex_vector vertices;
    normal_vector normals;
};

struct property {
    string dataType;
    string name;
};

struct element {
    string name;
    vector<property> properties;
    int count;
};

struct plyHeader {
    vector<element> elements;
    vector<string> comments;
    string format;
};

modelInfo readPlyFile(std::string);
modelInfo readObjFile(std::string);
plyHeader readPlyHeader(vector<string>);
void writePlyFile(std::string, modelInfo);
void writeObjFile(std::string, modelInfo);
void writePlyHeader(std::ofstream&, modelInfo);


#endif /* processingIO_hpp */
