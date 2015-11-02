#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "ofMain.h"

const ofMatrix3x3 IDENTITY3X3 = ofMatrix3x3(1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f);

struct Material {
    Material(float density, float yMod, float pRatio, const ofColor& color)
        : density(density), yMod(yMod), pRatio(pRatio), color(color) {}
    float density;
    float yMod;
    float pRatio;
    ofColor color;
};

struct RigidBody {
public:
    RigidBody(const string& modesFileName, const string& objFileName, const Material& material, float scale = 1.f);

    void rotate(float rad, const ofVec3f& axis);
    
    void step(float dt);
    void stepW(float dt);
    
    int audioStep(float dt, const ofVec3f& impulse, int vertex, float dt_q, float* qSum);

    ofVec3f getXi(int i) const;
    ofVec3f getVi(int i) const;

public:
    ofMesh mesh;
    const Material& material;

    // Constant quantities
    float m;
    ofMatrix3x3 IBody;
    ofMatrix3x3 IBodyInv;

    // State variables
    ofVec3f x;          // position (center of mass)
    ofQuaternion q;     // orientation
    ofVec3f P;          // linear momentum
    ofVec3f L;          // angular momentum

    // Derived quantities
    ofMatrix3x3 IInv;   // IInv = R * IBodyInv * R^T
    ofMatrix3x3 R;      // rotation matrix from q (object to world)
    ofMatrix3x3 RInv;
    ofVec3f v;          // linear velocity      v = P / m
    ofVec3f w;          // angular velocity     w = IInv * L


    // Modes (Constant)
    vector<vector<ofVec3f>> phi;    // eigenvectors
    vector<float> omega;            // natural frequencies

    // Modal amplitudes
    vector<float> qq[3];        // 3 most recent q vectors: q(k-2),q(k-1),q(k)
    int qkAt;                   // index where q(k) is stored
};

#endif
