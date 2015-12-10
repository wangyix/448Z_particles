#ifndef UTILS_H
#define UTILS_H

#include "ofMain.h"

#include <complex>

const double airC = 340.0;
const double fluidRho = 1.225;
const complex<double> I(0.0, 1.0);

float signedMoment(const ofMeshFace& tri, int coord, int pow);
void signedMoment2(const ofMeshFace& tri, float* Mxx, float* Myy, float* Mzz,
                                          float* Mxy, float* Myz, float* Mxz);

void computeYConstants(int N, vector<double>& C_storage, vector<double*>& C);
void computeHankels(double kr, int N, vector<complex<double>>& h);
void computeHankelsAndDerivatives(double kr, int N, vector<complex<double>>& h, vector<complex<double>>& h1);
void computeLegendrePolys(double x, int N, vector<double>& P_storage, vector<double*>& P);
void computeLegendrePolysAndDerivatives(double x, int N, vector<double>& P_storage, vector<double*>& P,
                                        vector<double>& P1_storage, vector<double*>& P1);

void computeSphericalHarmonics(int N, const ofVec3f& p, vector<complex<double>>& Y_storage);

#endif
