#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "ofMain.h"

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
    RigidBody(const ofMesh& triMesh, const Material& material);
    
    void scale(float s);
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
    ofMatrix3x3 IInv;
    ofMatrix3x3 R;      // rotation matrix from q
    ofVec3f v;          // linear velocity
    ofVec3f w;          // angular velocity
};

#endif
