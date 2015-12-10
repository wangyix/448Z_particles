#ifndef UTILS_H
#define UTILS_H

#include "ofMain.h"

#include <complex>

float signedMoment(const ofMeshFace& tri, int coord, int pow);
void signedMoment2(const ofMeshFace& tri, float* Mxx, float* Myy, float* Mzz,
                                          float* Mxy, float* Myz, float* Mxz);

void computeHankelsAndDerivatives(double kr, int N, vector<complex<double>>& h, vector<complex<double>>& h1);
void computeLegendrePolysAndDerivatives(double x, int N, vector<double>& P_storage, vector<double*>& P,
                                        vector<double>& P1_storage, vector<double*>& P1);

#endif
