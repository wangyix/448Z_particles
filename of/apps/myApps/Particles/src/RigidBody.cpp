#include "RigidBody.h"
#include <assert.h>
#include <iostream>
#include <fstream>

static void readObj(const string& fileName, float scale, ofMesh* mesh) {
    cout << "Reading geometry data from " << fileName << endl;
    ifstream file;
    file.open(fileName, ios::in);
    if (!file.is_open()) {
        cout << "Failed to open " << fileName << endl;
        return;
    }
    mesh->clear();
    vector<ofVec3f> normals;
    string line;
    while (getline(file, line)) {
        char c;
        istringstream iss(line);
        iss >> c;
        if (c == '#') continue;
        else if (c == 'v') {
            ofVec3f v;
            iss >> v.x >> v.y >> v.z;
            mesh->addVertex(v * scale);
            normals.emplace_back(0.f, 0.f, 0.f);
        } else if (c == 'f') {
            int indices[3];
            iss >> indices[0] >> indices[1] >> indices[2];
            ofVec3f v[3];
            for (int i = 0; i < 3; i++) {
                v[i] = mesh->getVertex(--indices[i]);
            }
            ofVec3f n = (v[1] - v[0]).crossed(v[2] - v[0]).normalized();
            for (int i = 0; i < 3; i++) {
                normals[indices[i]] += n;
            }
            mesh->addTriangle(indices[0], indices[1], indices[2]);
        } else{
            std::cout << "Warning: unrecognized line type " << c << endl;
        }
    }
    file.close();
    for (int i = 0; i < normals.size(); i++) {
        normals[i].normalize();
    }
    mesh->addNormals(normals);
    mesh->setMode(OF_PRIMITIVE_TRIANGLES);
}


static float signedVolume(const ofMeshFace& tri) {
    const ofVec3f& v1 = tri.getVertex(0);
    const ofVec3f& v2 = tri.getVertex(1);
    const ofVec3f& v3 = tri.getVertex(2);
    float v321 = v3.x * v2.y * v1.z;
    float v231 = v2.x * v3.y * v1.z;
    float v312 = v3.x * v1.y * v2.z;
    float v132 = v1.x * v3.y * v2.z;
    float v213 = v2.x * v1.y * v3.z;
    float v123 = v1.x * v2.y * v3.z;
    return (-v321 + v231 + v312 - v132 - v213 + v123) / 6.f;
}

// computes integral x^p*y^q*z^r dxdydz over the tetrahedron formed by a face and the
// origin, where two of p,q,r are 0. The param "coord" determines which of x,y,z has the nonzero
// exponent (0=x, 1=y, 2=z), and "pow" determines that exponent, which can be 0, 1, or 2.
// http://research.microsoft.com/en-us/um/people/chazhang/publications/icip01_ChaZhang.pdf
static float signedMoment(const ofMeshFace& tri, int coord, int pow) {
    assert(0 <= coord && coord < 3);
    float x[3], y[3], z[3];
    for (int i = 0; i < 3; i++) {
        x[i] = tri.getVertex(i)[(coord) % 3];
        y[i] = tri.getVertex(i)[(coord + 1) % 3];
        z[i] = tri.getVertex(i)[(coord + 2) % 3];
    }
    float M000 = (-x[2] * y[1] * z[0]
        + x[1] * y[2] * z[0]
        + x[2] * y[0] * z[1]
        - x[0] * y[2] * z[1]
        - x[1] * y[0] * z[2]
        + x[0] * y[1] * z[2]) / 6.f;
    switch (pow) {
    case 0:
        return M000;    // this is just the signed volume of the tetrahedron
        break;
    case 1:
        return 0.25f * (x[0] + x[1] + x[2]) * M000;
        break;
    case 2:
        return 0.1f * (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]
            + x[0] * x[1] + x[1] * x[2] + x[2] * x[0])
            * M000;
        break;
    default:
        assert(false);
        break;
    }
}

// computes integrals x^p*y^q*z^r dxdydz over a face tetrahedron where p+q+r=2.
// http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
static void signedMoment2(const ofMeshFace& tri,
    //float* M,
    //float* Mx, float* My, float* Mz,
    float* Mxx, float* Myy, float* Mzz,
    float* Mxy, float* Myz, float* Mxz) {
    ofVec3f V[3];
    for (int i = 0; i < 3; i++) {
        V[i] = tri.getVertex(i);
    }
    ofVec3f E1 = V[1] - V[0];
    ofVec3f E2 = V[2] - V[0];
    ofVec3f delta = E1.crossed(E2);

    // f1[0] = f1(x), f1[1] = f1(y), etc
    const int X = 0;
    const int Y = 1;
    const int Z = 2;
    float f1[3];
    float f2[3];
    float f3[3];
    float g[3][3];
    for (int w = 0; w < 3; w++) {   // iterating over x, y, z
        float w0 = V[0][w];
        float w1 = V[1][w];
        float w2 = V[2][w];
        f1[w] = w0 + w1 + w2;
        f2[w] = w0*w0 + w1*(w0 + w1) + w2*f1[w];
        f3[w] = w0*w0*w0 + w1*(w0*w0 + w0*w1 + w1*w1) + w2*f2[w];
        for (int i = 0; i < 3; i++) {
            float wi = V[i][w];
            g[i][w] = f2[w] + wi*(f1[w] + wi);
        }
    }
    //*M = (delta[0] / 6.f) * f1[X];
    //*Mx = 0.5f * (delta[0] / 12.f) * f2[X];
    //*My = 0.5f * (delta[1] / 12.f) * f2[Y];
    //*Mz = 0.5f * (delta[2] / 12.f) * f2[Z];
    *Mxx = (1.f / 3.f) * (delta[0] / 20.f) * f3[X];
    *Myy = (1.f / 3.f) * (delta[1] / 20.f) * f3[Y];
    *Mzz = (1.f / 3.f) * (delta[2] / 20.f) * f3[Z];
    *Mxy = 0.5f * (delta[0] / 60.f) * (V[0].y*g[0][X] + V[1].y*g[1][X] + V[2].y*g[2][X]);
    *Myz = 0.5f * (delta[1] / 60.f) * (V[0].z*g[0][Y] + V[1].z*g[1][Y] + V[2].z*g[2][Y]);
    *Mxz = 0.5f * (delta[2] / 60.f) * (V[0].x*g[0][Z] + V[1].x*g[1][Z] + V[2].x*g[2][Z]);
}

RigidBody::RigidBody(const string& fileName, const Material& material, float scale)
    :
    material(material),
    x(0.f, 0.f, 0.f),
    q(0.f, 0.f, 0.f, 1.f),
    P(0.f, 0.f, 0.f),
    L(0.f, 0.f, 0.f),
    R(IDENTITY3X3),
    RInv(IDENTITY3X3),
    v(0.f, 0.f, 0.f),
    w(0.f, 0.f, 0.f) {

    readObj(fileName, scale, &mesh);

    // compute zeroth and first order moments
    float M = 0.f;
    float Mx = 0.f, My = 0.f, Mz = 0.f;
    const vector<ofMeshFace>* triangles = &mesh.getUniqueFaces();
    for (const ofMeshFace& tri : *triangles) {
        M += signedMoment(tri, 0, 0);
        Mx += signedMoment(tri, 0, 1);
        My += signedMoment(tri, 1, 1);
        Mz += signedMoment(tri, 2, 1);
    }

    m = material.density * abs(M);          // mass
    ofVec3f r = ofVec3f(Mx, My, Mz) / M;    // center of mass

    // move mesh so its center of mass is at origin
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        mesh.setVertex(i, mesh.getVertex(i) - r);
    }
    triangles = &mesh.getUniqueFaces();
    
    // compute second-order moments
    float Mxx = 0.f, Myy = 0.f, Mzz = 0.f;
    float Mxy = 0.f, Myz = 0.f, Mxz = 0.f;
    for (const ofMeshFace& tri : *triangles) {
        float mxx, myy, mzz, mxy, myz, mxz;
        signedMoment2(tri, &mxx, &myy, &mzz, &mxy, &myz, &mxz);
        Mxx += mxx, Myy += myy, Mzz += mzz;
        Mxy += mxy, Myz += myz, Mxz += mxz;
    }

    float Ixx = material.density * (Myy + Mzz);
    float Iyy = material.density * (Mzz + Mxx);
    float Izz = material.density * (Mxx + Myy);
    //float Ixy = material.density * Mxy;
    //float Iyz = material.density * Myz;
    //float Ixz = material.density * Mxz;
    /*IBody = ofMatrix3x3(Ixx, -Ixy, -Ixz,    // moment of inertia
                        -Ixy, Iyy, -Iyz,
                        -Ixz, -Iyz, Izz);
    IBodyInv = IBody.inverse();*/
    //Ixx *= 100.f;
    IBody = ofMatrix3x3(Ixx, 0.f, 0.f,    // assuming Ixy,Iyz,Ixz=0
                        0.f, Iyy, 0.f,
                        0.f, 0.f, Izz);
    IBodyInv = ofMatrix3x3(1.f/Ixx, 0.f, 0.f,    // assuming Ixy,Iyz,Ixz=0
                           0.f, 1.f/Iyy, 0.f,
                           0.f, 0.f, 1.f/Izz);
    IInv = IBodyInv;
}

void RigidBody::rotate(float rad, const ofVec3f& axis) {
    ofVec3f a = axis.normalized();
    float halfAngle = 0.5f * rad;
    float sin = sinf(halfAngle);
    ofQuaternion dq = ofQuaternion(sin*a.x, sin*a.y, sin*a.z, cosf(halfAngle));
    q = dq * q;
    q.normalize();

    R.setRotate(q);
    RInv = R.transposed();
    IInv = R * IBodyInv * R.transposed();
    w = IInv * L;
}

void RigidBody::step(float dt) {
    x += dt * v;

    // update w using Euler's equation
    assert(dt > 0.f);
    // not quite backwards euler???
    ofVec3f wBody = RInv * w;
    //ofVec3f tauBody = RInv * (dL / dt);
    float w1 = wBody.x;
    float w2 = wBody.y;
    float w3 = wBody.z;
    float I1 = IBody.a;
    float I2 = IBody.e;
    float I3 = IBody.i;
    ofMatrix3x3 A(I1 / dt, (I3 - I2)*w3, 0.f,
                  0.f, I2 / dt, (I1 - I3)*w1,
                  (I2 - I1)*w2, 0.f, I3 / dt);
    ofVec3f b(I1*w1 / dt, I2*w2 / dt, I3*w3 / dt);
    wBody = A.inverse() * b;// (b + tauBody);
    w = R * wBody;

    /*ofVec3f dwBodydt;
    float Ixx = IBody.a;
    float Iyy = IBody.e;
    float Izz = IBody.i;
    dwBodydt.x = -(Izz - Iyy)*wBody.y*wBody.z / Ixx;
    dwBodydt.y = -(Ixx - Izz)*wBody.z*wBody.x / Iyy;
    dwBodydt.z = -(Iyy - Ixx)*wBody.x*wBody.y / Izz;
    wBody += dt * dwBodydt;
    w = R * wBody;
    */

    // update q, R, IInv using w
    float wMag = w.length();
    float halfAngle = 0.5f * dt * wMag;
    ofVec3f axis = w / wMag;
    float sin = sinf(halfAngle);
    ofQuaternion dq = ofQuaternion(sin*axis.x, sin*axis.y, sin*axis.z, cosf(halfAngle));
    q = dq * q;
    q.normalize();
    R.setRotate(q);
    RInv = R.transposed();
    IInv = R * IBodyInv * RInv;
}
