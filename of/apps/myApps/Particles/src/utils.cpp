#include "utils.h"

#include <assert.h>
#include <complex>

using namespace std;

/*static float signedVolume(const ofMeshFace& tri) {
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
}*/

// computes integral x^p*y^q*z^r dxdydz over the tetrahedron formed by a face and the
// origin, where two of p,q,r are 0. The param "coord" determines which of x,y,z has the nonzero
// exponent (0=x, 1=y, 2=z), and "pow" determines that exponent, which can be 0, 1, or 2.
// http://research.microsoft.com/en-us/um/people/chazhang/publications/icip01_ChaZhang.pdf
float signedMoment(const ofMeshFace& tri, int coord, int pow) {
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
        return 0.f;
        break;
    }
}

// computes integrals x^p*y^q*z^r dxdydz over a face tetrahedron where p+q+r=2.
// http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
void signedMoment2(const ofMeshFace& tri,
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


void computeHankels(double kr, int N, vector<complex<double>>& h) {
    const complex<double> I(0.0, 1.0);
    h.resize(N);

    if (N <= 0) return;
    complex<double> e_neg_iz = exp(-I*kr);
    h[0] = I / kr * e_neg_iz;
    if (N == 1) return;
    h[1] = -(kr - I) / (kr*kr) * e_neg_iz;

    for (int n = 1; n < N - 1; n++) {
        h[n + 1] = (2 * n + 1) / kr * h[n] - h[n - 1];
    }
}

void computeHankelsAndDerivatives(double kr, int N, vector<complex<double>>& h, vector<complex<double>>& h1) {
    if (N <= 0) {
        h.clear();
        h1.clear();
        return;
    }
    const complex<double> I(0.0, 1.0);
    computeHankels(kr, N + 1, h);
    h1.resize(N);
    h1[0] = (kr - I)*exp(-I*kr) / (kr*kr);
    for (int n = 1; n < N; n++) {
        h1[n] = 0.5 * (h[n - 1] - h[n] / kr + h[n + 1]);
    }
    h.pop_back();
}

// Associated Legendre polynomials
// Recurrence formulas from here:
// https://en.wikipedia.org/wiki/Associated_Legendre_polynomials#Recurrence_formula
void computeLegendrePolys(double x, int N, vector<double>& P_storage, vector<double*>& P) {
    assert(-1.0 <= x && x <= 1.0);
    P_storage.resize(N*N);
    P.resize(N);

    if (N <= 0) return;
    P[0] = &P_storage[0];
    P[0][0] = 1.0;
    if (N == 1) return;
    P[1] = &P_storage[2];
    double sqrt_one_minus_xx = sqrt(max(1.0 - x*x, 0.0));
    P[1][-1] = 0.5 * sqrt_one_minus_xx;
    P[1][0] = x;
    P[1][1] = -sqrt_one_minus_xx;

    for (int n = 1; n < N - 1; n++) {
        // compute pointer to P(n+1)(0)
        P[n + 1] = P[n] + (2 * n + 2);
        // compute P(n+1) for all m except for the 4 outermost m
        for (int m = -n + 1; m <= n - 1; m++) {
            P[n + 1][m] = ((2 * n + 1)*x*P[n][m] - (n + m)*P[n - 1][m]) / (n - m + 1);
        }
        // compute P(n+1) for the 2 largest m
        P[n + 1][n + 1] = -(2 * n + 1) * sqrt_one_minus_xx * P[n][n];
        P[n + 1][n] = x * (2 * n + 1) * P[n][n];
        // compute P(n+1) for the 2 smallest m (I derived these two formulas)
        P[n + 1][-n - 1] = sqrt_one_minus_xx / (2 * n + 2) * P[n][-n];
        P[n + 1][-n] = x * P[n][-n];
    }
}

