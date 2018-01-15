
//Standard
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include "vcgProcessing.h"

using namespace std;

// public functions
inline
void VCGProcessing::importOBJAsMesh(string filename) {    return VCGProcessing::importOBJAsMesh(filename, this->mesh);   }
inline
void VCGProcessing::saveMeshAsPLY(MyMesh &mesh, string filename) {
    //TODO: Fix this. Leads to some library error or sth.
//    int err = vcg::tri::io::ExporterPLY<MyMesh>::Save(mesh, filename.c_str(), 0);
    int err=-1;
    if (err == 0)
        cout << "Saved mesh at " << filename << endl;
    else
        cout << "Error saving mesh at " << filename << endl;
}
inline
void VCGProcessing::saveMeshAsOBJ(MyMesh& mesh, string filename) {
    int err = vcg::tri::io::ExporterOBJ<MyMesh>::Save(mesh, filename.c_str(), 0);
    if (err == 0)
        cout << "Saved mesh at " << filename << endl;
    else
        cout << "Error saving mesh at " << filename << endl;
}
inline
void VCGProcessing::printVertexLocation(MyVertex & vertex) {}
inline
MyMesh& VCGProcessing::getMesh() {    return this->mesh;   }
inline
void VCGProcessing::setOBJFolder(string fpath) {   this->objFolder = fpath;  }
inline
void VCGProcessing::setPLYFolder(string fpath) {   this->plyFolder = fpath;  }
inline
void VCGProcessing::performProcess()
{
    this->holeFillTrivialEar(this->mesh);
//    this->triFitting(this->mesh);
}


// private functions

inline
float EvalPlane(vcg::Plane3f &pl, std::vector<vcg::Point3f> posVec)
{
  float off=0;
  for(size_t i=0;i<posVec.size();++i)
    off += fabs(vcg::SignedDistancePlanePoint(pl,posVec[i]));

  off/=float(posVec.size());
  return off;
}
inline
void VCGProcessing::importOBJAsMesh(string filename, MyMesh& mesh) {
    int zero;
    if (vcg::tri::io::ImporterOBJ<MyMesh>::Open(mesh, filename.c_str(), zero) == vcg::tri::io::ImporterOBJ<MyMesh>::E_NOERROR) {
        cout << "Imported OBJ as Mesh from " << filename << endl;
    }
    else {
        cout << "Error importing file: " << filename << endl;
    }
}
inline
void VCGProcessing::ransacTest(MyMesh & mesh) {
//    cout << "Mesh has " << mesh.VN() << " vertices, " << mesh.FN() << " faces and " << mesh.EN() << " edges." << endl;
//    vcg::tri::UpdateBounding<MyMesh>::Box(mesh);
//
//    vcg::RansacFramework<MyMesh, vcg::BaseFeatureSet<MyMesh> > ran;
//    vcg::RansacFramework<MyMesh, vcg::BaseFeatureSet<MyMesh> >::Param params;
//    vcg::BaseFeatureSet<MyMesh>::Param bParams;
//    params.samplingRadiusPerc = 0.008;
//    params.evalSize = 50;
//    params.inlierRatioThr = 0.5;
//    params.iterMax = 400;
//    params.maxMatchingFeatureNum = 500;
//    bParams.featureSampleRatio = 0.5;
//    MyMesh mesh2;
//    this->importOBJAsMesh(this->fname, mesh2);
//    ran.Init(this->mesh, mesh2, params, bParams);
//    cout << "Ran Init" << endl;
}
inline
void VCGProcessing::createBoundingBox(MyMesh & mesh) {
    vcg::Box3<float> boundingBox;

}
inline
bool VCGProcessing::normalTest(typename vcg::face::Pos<MyMesh::FaceType> pos) {
    MyMesh::ScalarType thr = 0.0f;
    MyMesh::CoordType NdP = vcg::TriangleNormal<MyMesh::FaceType>(*pos.f);
    MyMesh::CoordType tmp, oop, soglia = MyMesh::CoordType(thr, thr, thr);
    vcg::face::Pos<MyMesh::FaceType> aux = pos;
    do {
        aux.FlipF();
        aux.FlipE();
        oop = vcg::Abs(tmp - ::vcg::TriangleNormal<MyMesh::FaceType>(*pos.f));
        if (oop < soglia)
            return false;
    } while ((aux != pos) && (!aux.IsBorder()));
    return true;
}
inline
void VCGProcessing::holeFillTrivialEar(MyMesh & mesh) {
    int holeSize = 15;

    //Update face-face topology
    vcg::tri::UpdateTopology<MyMesh>::FaceFace(mesh);

    vcg::tri::UpdateNormal<MyMesh>::PerVertexClear(mesh);
    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalized(mesh);
    // THIS IS NOT WOTKING ... Unable to UpdateNormal using Faces.
    // vcg::tri::UpdateNormal<MyMesh>::PerFace(mesh);
    // vcg::tri::UpdateNormal<MyMesh>::PerVertexPerFace(mesh);

    vcg::tri::UpdateFlags<MyMesh>::FaceBorderFromFF(mesh);
    assert(vcg::tri::Clean<MyMesh>::IsFFAdjacencyConsistent(mesh));

    //Compute the average of face areas
    float avg, sumA = 0.0f;
    int numA = 0, indices;
    indices = mesh.face.size();
    MyMesh::FaceIterator fi;
    for (fi = mesh.face.begin(); fi != mesh.face.end(); ++fi) {
        sumA += vcg::DoubleArea(*fi)/2;
        numA++;
        for (int ind=0; ind<3; ++ind) {
            fi->V(ind)->InitIMark();
        }
    }
    avg = sumA / numA;

    cout << "Average: " << avg << endl;


    //Algo - Issue with Trivial Ear, Replaced with selfIntersectionEar...
    vcg::tri::Hole<MyMesh>::EarCuttingIntersectionFill<vcg::tri::SelfIntersectionEar<MyMesh> >(mesh, holeSize, false);
    vcg::tri::UpdateFlags<MyMesh>::FaceBorderFromFF(mesh);
    assert(vcg::tri::Clean<MyMesh>::IsFFAdjacencyConsistent(mesh));

    //Start refining
    MyMesh::VertexIterator vi;
    MyMesh::FaceIterator f;
    vector<MyMesh::FacePointer> vf;
    f = mesh.face.begin();
    f += indices;
    for (; f != mesh.face.end(); ++f) {
        if (!f->IsD()) {
            f->SetS();
        }
    }

    vector<MyMesh::FacePointer *> FPP;
    vector<MyMesh::FacePointer> added;
    vector<MyMesh::FacePointer>::iterator vfit;
    int i = 1;
    for (f = mesh.face.begin(); f != mesh.face.end(); ++f) {
        if (!(*f).IsD()) {
            if (f->IsS()) {
                f->V(0)->IsW();
                f->V(1)->IsW();
                f->V(2)->IsW();
            }
            else {
                f->V(0)->ClearW();
                f->V(1)->ClearW();
                f->V(2)->ClearW();
            }
        }
    }

    vcg::BaseParameterClass pp;
    vcg::LocalOptimization<MyMesh> Fs(mesh, &pp);
    Fs.SetTargetMetric(0.0f);
    Fs.Init<MyDelaunayFlip>();
    Fs.DoOptimization();

    do {
        vf.clear();
        f = mesh.face.begin();
        f += indices;
        for (; f != mesh.face.end(); ++f) {
            if (f->IsS()) {
                bool test = true;
                for (int ind = 0; ind < 3; ++ind) {
                    f->V(ind)->InitIMark();
                }
//                this is always false for some reason
//                test = (vcg::DoubleArea<MyMesh::FaceType>(*f)/2) > avg;
//                if (test)
                    vf.push_back(&(*f));
            }
        }
        i++;
        printf("Refining [%d] -> %d\n", i, int(vf.size()));

        FPP.clear();
        added.clear();
        for (vfit = vf.begin(); vfit != vf.end(); ++vfit) {
            FPP.push_back(&(*vfit));
        }
        int toAdd = vf.size();
        MyMesh::FaceIterator f1, f2;
        f2 = vcg::tri::Allocator<MyMesh>::AddFaces(mesh, toAdd*2, FPP);
        MyMesh::VertexIterator vertp = vcg::tri::Allocator<MyMesh>::AddVertices(mesh, toAdd);
        vector<MyMesh::FacePointer> added;
        added.reserve(toAdd);
        vfit = vf.begin();

        for (int i=0; i<toAdd; ++i, f2++, vertp++) {
            f1 = f2;
            f2++;
            TriSplit<MyMesh, CenterPointBarycenter<MyMesh> >::Apply(vf[i], &(*f1), &(*f2), &(*vertp), CenterPointBarycenter<MyMesh>());
            f1->SetS();
            f2->SetS();
            for (int itr=0; itr<3; itr++) {
                f1->V(itr)->SetW();
                f2->V(itr)->SetW();
            }
            added.push_back(&(*f1));
            added.push_back(&(*f2));
        }
        vcg::BaseParameterClass pp;
        vcg::LocalOptimization<MyMesh> FlippingSession(mesh, &pp);
        FlippingSession.SetTargetMetric(0.0f);
        FlippingSession.Init<MyDelaunayFlip>();
        FlippingSession.DoOptimization();
    } while (i < 5);
    cout << "Out of loop";
    vcg::LocalOptimization<MyMesh> Fiss(mesh, &pp);
    Fiss.SetTargetMetric(0.0f);
    Fiss.Init<MyDelaunayFlip>();
    Fiss.DoOptimization();
    int UBIT = MyMesh::VertexType::NewBitFlag();
    f = mesh.face.begin();
    f += indices;
    for (; f != mesh.face.end(); ++f) {
        if (f->IsS()) {
            for (int ind=0; ind<3; ++ind) {
                if (this->normalTest(vcg::face::Pos<MyMesh::FaceType>(&(*f), ind))) {
                    f->V(ind)->SetUserBit(UBIT);
                }
            }
            f->ClearS();
        }
    }
    for (vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi) {
        if (!(*vi).IsD()) {
            if (vi->IsUserBit(UBIT)) {
                (*vi).SetS();
                vi->ClearUserBit(UBIT);
            }
        }
    }

    vcg::tri::Smooth<MyMesh>::VertexCoordLaplacianBlend(mesh, 1, true);
    cout << "Completed. Saving..." << endl;
    string fpath = this->objFolder + "trivialEar.obj";
    vcg::tri::io::ExporterOBJ<MyMesh>::Save(mesh, fpath.c_str(), false);
}
inline
void VCGProcessing::triFitting(MyMesh & m){
    vcg::tri::Icosahedron(m);
//    need this but doesnt work cuz faces are not normalized
    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFaceNormalized(m);
    vcg::tri::UpdateBounding<MyMesh>::Box(m);

    // As a simple test just run over all the faces of a mesh
    // get a few random points over it, perturb them and fit a plane on them.
    vcg::Plane3f ple,plf,plw;
    int cnt=0;
    float scaleFac = m.bbox.Diag()/10.0f;
    printf("ScaleFac %f\n\n",scaleFac);
    vcg::math::MarsenneTwisterRNG rnd;
    for(int i=0;i<m.FN();++i)
    {
    std::vector<vcg::Point3f> ExactVec;
    std::vector<vcg::Point3f> PerturbVec;
    std::vector<float> WeightVec;
    vcg::Plane3f pl;
    pl.Init(vcg::Barycenter(m.face[i]),m.face[i].N());
    for(int j=0;j<200;++j)
    {
      vcg::Point3f p = vcg::tri::SurfaceSampling<MyMesh>::RandomPointInTriangle(m.face[i]);
      ExactVec.push_back(p);
      vcg::Point3f off = vcg::math::GeneratePointInUnitBallUniform<float>(rnd);
      p+=off*scaleFac;
      float w =  std::max(0.0f, 1.0f-fabs(vcg::SignedDistancePlanePoint(pl,p))/scaleFac);
      PerturbVec.push_back(p);
      WeightVec.push_back(w*w); // as weight we use the square of  (1-distance)
    }

    vcg::FitPlaneToPointSet(ExactVec,ple);
    float err=EvalPlane(ple,ExactVec);

    vcg::FitPlaneToPointSet(PerturbVec,plf);
    float err0=EvalPlane(plf,ExactVec);

    vcg::WeightedFitPlaneToPointSet(PerturbVec,WeightVec,plw);
    float err1=EvalPlane(plw,ExactVec);
    printf("Exact %5.3f Fit to Perturbed %5.3f Weighted fit to perturbed %5.3f\n",err,err0,err1);
    if(err0>err1) cnt++;
    }

    printf("\nWeighted Fitting was better %i on %i\n",cnt,m.FN());

    vcg::tri::Smooth<MyMesh>::VertexCoordLaplacianBlend(m, 1, true);
    cout << "Completed. Saving..." << endl;
    string fpath = this->objFolder + "fitting.obj";
    vcg::tri::io::ExporterOBJ<MyMesh>::Save(m, fpath.c_str(), false);
}
