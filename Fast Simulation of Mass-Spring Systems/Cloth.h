#pragma once
#include <Eigen/Sparse>
#include "FastBox.h"
#include "Plane.h"
#include "Sphere.h"
class Cloth {
friend class Renderer;
friend class FastBox;
public:
    Cloth(unsigned int nNodes = 80, float nodeMass = 1.0f, float shearStretchK = 400.0f, float bendK = 80.0f, float timeStep = 0.04f);
    void Run(unsigned int iterations = 8);
    void ComputeNormals();
    unsigned int GetSNodeCount() const { return snodeCount; }
    void SphereCollision(FastBox& fastbox, Sphere& sphere);
    void PlaneCollision(FastBox& fastbox, Plane& ground);
    void SelfCollision(FastBox& fastbox);
    void SetExternalForce();
private:
    void NodeInit();
    void SpringInit();
    void LMatrixBuild(float shearStretchK, float bendK);
    void JMatrixBuild(float shearStretchK, float bendK);
    void MMatrixBuild(float nodeMass);
    void Mph2LMatrixBuild();
    
    inline void LocalStep();
    inline void GlobalStep();
    void IndicesInit();
    bool TrianglePointProj(unsigned int triInd, unsigned int nodeInd, float& u, float& v);
    bool EdgeProj(unsigned int p0, unsigned int p1, unsigned int q0, unsigned int q1);

    float nodeMass;
    float timeStep;
    unsigned int snodeCount;
    unsigned int nodeCount;
    unsigned int springCount;
    Eigen::VectorXf nodes;
    Eigen::VectorXf previousNodes;
    Eigen::VectorXi springs;
    Eigen::VectorXf springsRestLength;
    Eigen::VectorXf springsDirection;
    Eigen::VectorXf y;
    Eigen::VectorXf fext;
    Eigen::SparseMatrix<int> triPointConstraints;

    Eigen::SparseMatrix<float> lMatrix;
    Eigen::SparseMatrix<float> jMatrix;
    Eigen::SparseMatrix<float> mMatrix;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > Mph2L;

    //for rendering
    Eigen::VectorXi indices;
    Eigen::VectorXf normals;

};