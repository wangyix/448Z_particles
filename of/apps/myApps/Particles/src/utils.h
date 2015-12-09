#ifndef UTILS_H
#define UTILS_H

#include "ofMain.h"

float signedMoment(const ofMeshFace& tri, int coord, int pow);
void signedMoment2(const ofMeshFace& tri, float* Mxx, float* Myy, float* Mzz,
                                          float* Mxy, float* Myz, float* Mxz);



#endif
