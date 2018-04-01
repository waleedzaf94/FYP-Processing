#ifndef ModelBuilder_h
#define ModelBuilder_h

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "Definitions.hpp"

using namespace std;

class ModelBuilder {
    public: 
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
    struct vertexInfo {
        double x=0, y=0, z=0;
    };
    struct faceInfo {
        double x=0, y=0, z=0, d=0; // Coefficients for plane equation
        vector<int> vertexIndices;
    };
    struct normalInfo {
        double nx=0, ny=0, nz=0;
    };
    typedef vector<vertexInfo> vertex_vector;
    typedef vector<normalInfo> normal_vector;
    typedef vector<faceInfo> face_vector;
    struct modelInfo {
        face_vector faces;
        vertex_vector vertices;
        normal_vector normals;
    };

    modelInfo modelinf;
    modelInfo outputModel;
    Pwn_vector points;
    void readFile(string filpath, string filetype);
    void writeFile(string filepath, string filetype);
    void ToPointAndFacetVectors(PointVector & points, FacetVector &faces);
    void ToPolyhedron(Polyhedron_3 &);
    void SetOutputModel(modelInfo &);
    bool writePlyPointsAndNormals (Pwn_vector&, std::string);
    bool writePlyPointsAndNormals (std::string);

    private:
    modelInfo readOffFile(string filpath);
    modelInfo readPlyFile(string filpath);
    modelInfo readObjFile(string filpath);
    void writeObjFile(string filename, modelInfo & model);
    void writePlyFile(string filename, modelInfo & model);
    void writeOffFile(string filename, modelInfo & model);
    void writePlyHeader(ofstream &out, modelInfo model);
    plyHeader readPlyHeader(vector<string> lines);
    Pwn_vector readPlyToPwn(std::string) ;
};

#endif