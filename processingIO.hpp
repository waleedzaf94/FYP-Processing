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
    float x, y, z;
};
struct faceInfo {
    float x, y, z, d;
    long viCount;
    std::vector<int> vertexIndices;
};
struct normalInfo {
    float nx, ny, nz;
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
plyHeader readPlyHeader(vector<string>);


#endif /* processingIO_hpp */
