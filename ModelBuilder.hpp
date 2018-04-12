#ifndef ModelBuilder_h
#define ModelBuilder_h

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "Definitions.hpp"
#include "polyhedronBuilders.h"

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
        size_t count;
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
        vector<size_t> vertexIndices;
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

    modelInfo model;
    modelInfo outputModel;
    Pwn_vector points;
    
    ModelBuilder() {
        
    }
    
    void readFile(string, string);
    void writeFile(string, string);
    void toPointAndFacetVectors(PointVector & points, FacetVector &faces);
    void toPwnVector(Pwn_vector &);
    void toPolyhedron(Polyhedron_3 &);
    void setOutputModel(modelInfo &);
    bool writePlyPointsAndNormals (Pwn_vector&, std::string);
    bool writePlyPointsAndNormals (std::string);
    void printModelInfo(ModelBuilder::modelInfo &);
    void printModelInfo();
    void writeObjFile(string filename, modelInfo & model);
    void writePlyFile(string filename, modelInfo & model);
    void writeOffFile(string filename, modelInfo & model);
    void writePlyHeader(ofstream &out, modelInfo model);
    void writePlyHeader(ofstream &out, Pwn_vector& points);
    modelInfo readOffFile(string filpath);
    modelInfo readPlyFile(string filpath);
    modelInfo readObjFile(string filpath);

    private:
    plyHeader readPlyHeader(vector<string> lines);
    Pwn_vector readPlyToPwn(std::string) ;
};

#endif
