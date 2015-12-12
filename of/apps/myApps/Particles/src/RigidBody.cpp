#include "RigidBody.h"
#include "utils.h"

#include <assert.h>
#include <iostream>
#include <fstream>
#include <omp.h>
#include <random>

void readObj(const string& fileName, float scale, ofMesh& mesh, vector<float>& vertexAreaSums) {
    cout << "Reading geometry data from " << fileName << endl;
    ifstream file;
    file.open(fileName, ios::in);
    if (!file.is_open()) {
        cout << "Failed to open " << fileName << endl;
        return;
    }
    mesh.clear();
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
            mesh.addVertex(v * scale);
            normals.emplace_back(0.f, 0.f, 0.f);
            vertexAreaSums.push_back(0.f);
        } else if (c == 'f') {
            int indices[3];
            iss >> indices[0] >> indices[1] >> indices[2];
            ofVec3f v[3];
            for (int i = 0; i < 3; i++) {
                v[i] = mesh.getVertex(--indices[i]);
            }
            ofVec3f n_double_area = (v[1] - v[0]).crossed(v[2] - v[0]);
            for (int i = 0; i < 3; i++) {
                normals[indices[i]] += n_double_area;
                vertexAreaSums[indices[i]] += 0.5f * n_double_area.length();
            }
            mesh.addTriangle(indices[0], indices[1], indices[2]);
        } else{
            std::cout << "Warning: unrecognized line type " << c << endl;
        }
    }
    file.close();
    for (int i = 0; i < normals.size(); i++) {
        normals[i].normalize();
    }
    mesh.addNormals(normals);
    mesh.setMode(OF_PRIMITIVE_TRIANGLES);
}

void RigidBody::readModes(const string& fileName, float E, float nu, float rho, float sizeScale,
                          vector<vector<ofVec3f>>* phi, vector<float>* omega) {

    float modesG = E / (2.f * (1 + nu));
    float materialG = material.E / (2.f * (1 + material.nu));
    float stiffnessScale = sqrtf(materialG / modesG);

    float densityScale = sqrtf(rho / material.rho);

    float omegaScale = stiffnessScale * densityScale / sizeScale;
    float phiScale = densityScale / sqrtf(sizeScale*sizeScale*sizeScale);

    cout << "Reading modes data from " << fileName << endl;
    ifstream file;
    file.open(fileName, ios::in);
    if (!file.is_open()) {
        cout << "Failed to open " << fileName << endl;
        return;
    }

    phi->clear();
    omega->clear();
    int numModes, numVertices;
    file >> numModes;
    file >> numVertices;
    float omegaMax = 0.f;
    float omegaMin = numeric_limits<float>::max();
    vector<bool> modeIsUnderdamped(numModes);
    for (int i = 0; i < numModes; i++) {
        float eigenValue;
        file >> eigenValue;
        // only record the underdamped modes
        float wi = omegaScale * sqrtf(eigenValue);
        float xii = 0.5f * (alpha / wi + beta*wi);
        modeIsUnderdamped[i] = (0.f < xii && xii < 1.f);
        if (modeIsUnderdamped[i]) {
            omega->push_back(wi);
            if (wi > omegaMax) {
                omegaMax = wi;
            }
            if (wi < omegaMin) {
                omegaMin = wi;
            }
        }
    }
    for (int i = 0; i < numModes; i++) {
        // only record the underdamped modes
        if (modeIsUnderdamped[i]) {
            phi->push_back(vector<ofVec3f>());
            vector<ofVec3f>& phi_i = phi->back();
            for (int j = 0; j < numVertices; j++) {
                float x, y, z;
                ofVec3f phi_i_j;
                file >> phi_i_j.x >> phi_i_j.y >> phi_i_j.z;
                phi_i.push_back(phiScale * phi_i_j);
            }
        } else {
            for (int j = 0; j < numVertices; j++) {
                float unused;
                file >> unused >> unused >> unused;
            }
        }
    }
    file.close();
    cout << omega->size() << " underdamped modes; " << (numModes-omega->size()) << " modes discarded" << endl;
    cout << "Min freq: " << (omegaMin/(2.f*PI)) << " hz   Max freq: " << (omegaMax/(2.f*PI)) << " hz" << endl;
}



void RigidBody::computeMIBodyIBodyInv() {
    // compute zeroth and first order moments
    float M = 0.f;                          // volume
    float Mx = 0.f, My = 0.f, Mz = 0.f;     // center of mass
    const vector<ofMeshFace>* triangles = &mesh.getUniqueFaces();
    for (const ofMeshFace& tri : *triangles) {
        M += signedMoment(tri, 0, 0);
        Mx += signedMoment(tri, 0, 1);
        My += signedMoment(tri, 1, 1);
        Mz += signedMoment(tri, 2, 1);
    }

    m = material.rho * abs(M);  // mass
    ofVec3f centerOfMass = ofVec3f(Mx, My, Mz) / M;

    // move mesh so its center of mass is at origin
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        mesh.setVertex(i, mesh.getVertex(i) - centerOfMass);
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

    float Ixx = material.rho * (Myy + Mzz);
    float Iyy = material.rho * (Mzz + Mxx);
    float Izz = material.rho * (Mxx + Myy);
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
    IBodyInv = ofMatrix3x3(1.f / Ixx, 0.f, 0.f,    // assuming Ixy,Iyz,Ixz=0
        0.f, 1.f / Iyy, 0.f,
        0.f, 0.f, 1.f / Izz);
    IInv = IBodyInv;
}

// For checking analytical dSdn value with numerical value
static complex<double> Snm(int n, int m, const ofVec3f& p, double k) {
    vector<double> C_storage;
    vector<double*> C;
    computeYConstants(n+1, C_storage, C);    // N should be the max out of all the modes

    // compute row of A: the normal derivatives of each basis function at this vertex
    double r = p.length();
    //double theta = acos(x.z / r);
    double phi = atan2(p.y, p.x);

    vector<complex<double>> h;
    computeHankels(k*r, n+1, h);

    vector<double> P_storage;
    vector<double*> P;
    computeLegendrePolys(p.z / r, n+1, P_storage, P);

    return h[n] * C[n][m] * P[n][m] * exp(I*(double)m*phi);
}



void RigidBody::computeModeCoeffs(const vector<float>& vertexAreaSums) {
    const vector<ofVec3f>& vertices = mesh.getVertices();
    const vector<ofVec3f>& normals = mesh.getNormals();
    int numVertices = mesh.getNumVertices();
    int numModes = omega.size();                                         // omega.size();   testsetsTESTESTSETESTEST
    assert(vertexAreaSums.size() == numVertices);
    
    const int N = 7;
    const int CANDIDATES = 16;
    const int MAX_SOURCES = 20;

    modeMultipoleSources.resize(numModes);

    vector<double> C_storage;
    vector<double*> C;
    computeYConstants(N, C_storage, C);    // N should be the max out of all the modes

    vector<vector<double>> modeMeanErrors(numModes);
    vector<vector<double>> modeMaxErrors(numModes);
    vector<vector<double>> modeBNormErrors(numModes);

    default_random_engine generator;
    normal_distribution<float> normalDist(0.f, 0.9f/3.f);
    uniform_real_distribution<float> uniformDist(0.3f, 0.9f);

    omp_set_num_threads(4);
    #pragma omp parallel for
    for (int j = 0; j < numModes; j++) {
        printf("mode_%d ", j);

        const vector<ofVec3f>& modeDisplacements = phi[j];
        double w = omega[j];
        double k = w / airC;

        // build vector b
        Eigen::VectorXcd b(numVertices);
        for (int vi = 0; vi < numVertices; vi++) {
            float weight = vertexAreaSums[vi];
            b(vi) = weight * fluidRho * w * w * modeDisplacements[vi].dot(normals[vi]);
            /*// bvi test input
            const double eps = 0.001;
            complex<double> dpdn_test = (Snm(0, 0, p + eps*normal, k) - Snm(0, 0, p - eps*normal, k)) / (2.0*eps);
            dpdn_test += 3.0*I*(Snm(1, 1, p + eps*normal, k) - Snm(1, 1, p - eps*normal, k)) / (2.0*eps);
            dpdn_test += complex<double>(5.0,-7.0)*(Snm(2, -1, p + eps*normal, k) - Snm(2, -1, p - eps*normal, k)) / (2.0*eps);
            b(vi) = weight * dpdn_test;
            */
        }

        Eigen::VectorXcd b_residual = b;     // residual
        bool mode5CoeffsFound = false;
        bool mode20CoeffsFound = false;
        int sources = 0;
        while ((/*!mode5CoeffsFound ||*/ !mode20CoeffsFound) && (sources++ < MAX_SOURCES)) {
            //printf("%d ", sources);

            const int numCandidates = min(CANDIDATES, numVertices);

            // pick candidate positions for the next source to add
            vector<ofVec3f> candidatePositions(numCandidates);
            vector<int> shuffledIndices(numVertices);
            for (int i = 0; i < numVertices; i++) {
                shuffledIndices[i] = i;
            }
            random_unique(shuffledIndices.begin(), shuffledIndices.end(), numCandidates);
            for (int i = 0; i < numCandidates; i++) {
                const ofVec3f& vertex = vertices[shuffledIndices[i]];
                //float a = min(abs(normalDist(generator)), 0.9f);
                float a = uniformDist(generator);
                candidatePositions[i] = a * vertex;
            }

            // compute A matrix for each candidate position
            //vector<Eigen::MatrixXcd> candidateAMatrices(numCandidates, Eigen::MatrixXcd(numVertices, N*N));
            MultipoleSource bestCandidate;
            Eigen::VectorXcd bestCandidateResidual;
            double minResidualSqNorm = b_residual.squaredNorm();
            for (int i = 0; i < numCandidates; i++) {
                Eigen::MatrixXcd A(numVertices, N*N);
                const ofVec3f& sourcePos = candidatePositions[i];

                for (int vi = 0; vi < mesh.getNumVertices(); vi++) {
                    const ofVec3f& p = vertices[vi] - sourcePos;
                    const ofVec3f& normal = normals[vi];
                    double weight = vertexAreaSums[vi];

                    // compute row of A: the normal derivatives of each basis function at this vertex
                    double r = p.length();
                    //double theta = acos(x.z / r);
                    double phi = atan2(p.y, p.x);
                    vector<complex<double>> h, h1;
                    computeHankelsAndDerivatives(k*r, N, h, h1);
                    vector<double> P_storage, P1_storage;
                    vector<double*> P, P1;
                    computeLegendrePolysAndDerivatives(p.z / r, N, P_storage, P, P1_storage, P1);
                    double nx = normal.x, ny = normal.y, nz = normal.z;
                    int col = 0;
                    for (int n = 0; n < N; n++) {
                        for (int m = -n; m <= n; m++) {
                            complex<double> dPdn = P1[n][m] * (nz - p.z*(p.dot(normal)) / (r*r)) / r;
                            complex<double> dhdn = h1[n] * k * (p.dot(normal) / r);
                            complex<double> dedn_over_e = I*(double)m*(p.x*ny - p.y*nx) / (double)(p.x*p.x + p.y*p.y);
                            complex<double> dSdn = C[n][m] * exp(I*(double)m*phi) * (h[n] * P[n][m] * dedn_over_e + h[n] * dPdn + dhdn*P[n][m]);
                            A(vi, col) = weight * dSdn;
                            col++;
                            /*// For checking analytical dSdn value with numerical value
                            complex<double> analytical = dSdn;
                            double eps = 0.001;
                            complex<double> numerical = (Snm(n, m, p + eps*normal, k) - Snm(n, m, p - eps*normal, k)) / (2.0 * eps);;
                            double relError = abs(analytical - numerical) / abs(analytical);
                            printf("%lf\n", relError);*/
                        }
                    }
                    assert(col == N*N);
                }
                Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr(A);
                Eigen::VectorXcd c = qr.solve(b_residual);
                Eigen::VectorXcd candidateResidual = b_residual - A*c;
                double candidateResidualSqNorm = candidateResidual.squaredNorm();
                if (candidateResidualSqNorm < minResidualSqNorm) {
                    minResidualSqNorm = candidateResidualSqNorm;
                    bestCandidateResidual = candidateResidual;
                    bestCandidate.pos = sourcePos;
                    bestCandidate.N = N;
                    bestCandidate.coeffs = c;
                }
            }

            modeMultipoleSources[j].push_back(bestCandidate);
            b_residual = bestCandidateResidual;



            // record some errors
            double maxError = 0.0;
            double meanError = 0.0;
            for (int i = 0; i < b.rows(); i++) {
                double relError = abs(b_residual(i)) / abs(b(i));
                maxError = max(relError, maxError);
                meanError += relError;
            }
            meanError /= b.rows();
            double normError = b_residual.norm() / b.norm();

            modeMeanErrors[j].push_back(meanError);
            modeMaxErrors[j].push_back(maxError);
            modeBNormErrors[j].push_back(normError);


            const vector<double>& normErrors = modeBNormErrors[j];
            double prevNormError = normErrors.size() > 1 ? normErrors[normErrors.size() - 2] : 100000.0;
            if ((normError <= 0.05 && prevNormError > 0.05) && !mode5CoeffsFound) {
                mode5CoeffsFound = true;
            }
            if ((normError <= 0.2 && prevNormError > 0.2) && !mode20CoeffsFound) {
                mode20CoeffsFound = true;
            }

        } // while (!mode5CoeffsFound || !mode20CoeffsFound)

    } // for each mode

    modeExpansionMaxOrder = 0;
    for (int j = 0; j < numModes; j++) {
        const vector<MultipoleSource>& sources = modeMultipoleSources[j];
        for (int i = 0; i < sources.size(); i++) {
            if (sources[i].N > modeExpansionMaxOrder) {
                modeExpansionMaxOrder = sources[i].N;
            }
        }
    }

    ofstream of("./out.txt", ios::out | ios::trunc);

    for (int j = 0; j < numModes; j++) {
        of << endl << endl;
        of << "mode " << j << ":  freq = " << omega[j] / (2.0*PI) << endl;
        for (int i = 0; i < modeMeanErrors[j].size(); i++) {
            of << "Source " << i << " at " << modeMultipoleSources[j][i].pos << ".   max error " << modeMaxErrors[j][i] * 100
                << "%, mean error " << modeMeanErrors[j][i] * 100
                << "%, |b_res|/|b| " << modeBNormErrors[j][i] * 100 << "%" << endl;
        }
    }

    of.close();


}

complex<double> MultipoleSource::evaluate(const ofVec3f& x, double k, const vector<double*>& C) const {
    ofVec3f p = x - pos;
    double r = p.length();
    //double theta = acos(x.z / r);
    double phi = atan2(p.y, p.x);

    vector<complex<double>> h;
    computeHankels(k*r, N, h);

    vector<double> P_storage;
    vector<double*> P;
    computeLegendrePolys(p.z / r, N, P_storage, P);

    complex<double> sum(0.0, 0.0);
    int i = 0;
    for (int n = 0; n < N; n++) {
        for (int m = -n; m <= n; m++) {
            complex<double> Snm = h[n] * C[n][m] * P[n][m] * exp(I*(double)m*phi);
            sum += (coeffs(i) * Snm);
            i++;
        }
    }
    return sum;
}

// point should be given in obj space
double RigidBody::evaluateAbsTransferFunction(const ofVec3f& x, int mode, const vector<double*>& C) {
    float w = omega[mode];
    double k = w / airC;
    
    const vector<MultipoleSource>& sources = modeMultipoleSources[mode];
    complex<double> sum(0.0, 0.0);
    for (int i = 0; i < sources.size(); i++) {
        sum += sources[i].evaluate(x, k, C);
    }
    return abs(sum);
}


RigidBody::RigidBody(const string& modesFileName, float E, float nu, float rho, float alpha, float beta,
                     const string& objFileName, const Material& material, float sizeScale,
                     bool isSphere)
    :
    material(material),
    x(0.f, 0.f, 0.f),
    q(0.f, 0.f, 0.f, 1.f),
    P(0.f, 0.f, 0.f),
    L(0.f, 0.f, 0.f),
    R(IDENTITY3X3),
    RInv(IDENTITY3X3),
    v(0.f, 0.f, 0.f),
    w(0.f, 0.f, 0.f),
    alpha(alpha),
    beta(beta),
    isSphere(isSphere)
{
    vector<float> vertexAreaSums;
    readObj(objFileName, sizeScale, mesh, vertexAreaSums);

    computeMIBodyIBodyInv();

    readModes(modesFileName, E, nu, rho, sizeScale, &phi, &omega);
    assert(phi.size() == omega.size());
    assert(phi[0].size() == mesh.getNumVertices());
    
    // initialize modal amplitude vectors to 0s
    for (int k = 0; k < 3; k++) {
        qq[k] = vector<float>(omega.size(), 0.f);
    }
    qkAt = 0;


    // determine sphere radius if this is a sphere
    if (isSphere) {
        r = 0.f;
        for (int i = 0; i < mesh.getNumVertices(); i++) {
            const ofVec3f& vertex = mesh.getVertex(i);
            float dist = vertex.length();
            if (dist > r) {
                r = dist;
            }
        }
    }
    
    // FOR TUNING DAMPING PARAMS
    topModes = true;
    nModesOnly = 94;

    computeModeCoeffs(vertexAreaSums);
}

void RigidBody::rotate(float rad, const ofVec3f& axis) {
    ofVec3f a = axis.normalized();
    float halfAngle = 0.5f * rad;
    float sin = sinf(halfAngle);
    ofQuaternion dq = ofQuaternion(sin*a.x, sin*a.y, sin*a.z, cosf(halfAngle));
    //q = dq * q;
    q *= dq;    // NOTE: openFrameworks quaternion multiply is flipped!!!!
    q.normalize();

    R.setRotate(q);
    RInv = R.transposed();
    IInv = R * IBodyInv * R.transposed();
    w = IInv * L;
}

void RigidBody::step(float dt) {
    x += dt * v;

    // update q, R, IInv using w
    float wMag = w.length();
    float halfAngle = 0.5f * dt * wMag;
    ofVec3f axis = w / wMag;
    float sin = sinf(halfAngle);
    ofQuaternion dq = ofQuaternion(sin*axis.x, sin*axis.y, sin*axis.z, cosf(halfAngle));
    //q = dq * q;
    q *= dq;    // NOTE: openFrameworks quaternion multiply is flipped!!!!
    q.normalize();
    R.setRotate(q);
    RInv = R.transposed();
    IInv = R * IBodyInv * RInv;

    w = IInv * L;   // ????
}

void RigidBody::stepW(float dt) {
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
}

int RigidBody::stepAudio(float dt, const vector<VertexImpulse>& impulses, float dt_q,
                         const ofVec3f& listenPos1, const ofVec3f& listenPos2, float* samples) {
    int numModes = omega.size();

    // evaluate the transfer function of each mode for each listening position
    vector<float> absTransferFunctions1(numModes);
    vector<float> absTransferFunctions2(numModes);

    ofVec3f listenPos1_obj = RInv * (listenPos1 - x);
    float listenDist1_obj = listenPos1_obj.length();
    ofVec3f listenPos2_obj = RInv * (listenPos2 - x);
    float listenDist2_obj = listenPos2_obj.length();

    vector<double> C_storage1, C_storage2;
    vector<double*> C1, C2;
    computeYConstants(modeExpansionMaxOrder, C_storage1, C1);
    computeYConstants(modeExpansionMaxOrder, C_storage2, C2);

    omp_set_num_threads(4);
    #pragma omp parallel for
    for (int i = 0; i < numModes; i++) {
        absTransferFunctions1[i] = evaluateAbsTransferFunction(listenPos1_obj, i, C1);
        absTransferFunctions2[i] = evaluateAbsTransferFunction(listenPos2_obj, i, C2);
    }

    float h = dt_q;

    // all impulses will be spread out over the first time-step of q as constant forces.
    // convert the impulses to forces in bodyspace.
    vector<ofVec3f> forces;
    for (const VertexImpulse& vim : impulses) {
        forces.push_back((RInv * vim.impulse) / h);
    }

    // compute q vectors and their sums
    int qsToCompute = max((int)(dt / h), 1);
    for (int k = 0; k < qsToCompute; k++) {
        qkAt = (qkAt + 1) % 3;
        const vector<float>& qk1 = qq[(qkAt + 2) % 3];
        const vector<float>& qk2 = qq[(qkAt + 1) % 3];
        vector<float>& qk = qq[qkAt];
        
        float pSum1 = 0.f;
        float pSum2 = 0.f;
        for (int i = 0; i < numModes; i++) {
            float wi = omega[i];
            float xii = 0.5f * (alpha/wi + beta*wi);
            //if (0.f < xii && xii < 1.f) {    // underdamped (don't need this; overdamped frequencies were removed)
if (0.f < xii && xii < 1.f &&   // for tuning damping params
    ((topModes && i >= numModes-nModesOnly) || (!topModes && i < nModesOnly))) {
                float wdi = wi * sqrtf(1 - xii*xii);

                float ei = exp(-xii*wi*h);
                float thetai = wdi * h;
                float gammai = asinf(xii);

                qk[i] = 2.f*ei*cosf(thetai)*qk1[i] - ei*ei*qk2[i];

                // impulses are applied evenly over the first time-step; no force applied for other time-steps
                if (k == 0) {
                    float phi_i_dot_F = 0.f;
                    for (int j = 0; j < impulses.size(); j++) {
                        int vertex = impulses[j].vertex;
                        ofVec3f force = forces[j];
                        phi_i_dot_F += phi[i][vertex].dot(force);
                    }
                    qk[i] += (2.f*(ei*cosf(thetai + gammai) - ei*ei*cosf(2.f*thetai + gammai)) / (3.f*wi*wdi))
                        * (phi_i_dot_F);
                }
} else {
    qk[i] = 0.f;
}
            assert(!isnan(qk[i]));
            pSum1 += qk[i] * absTransferFunctions1[i];
            pSum2 += qk[i] * absTransferFunctions2[i];
        }

        samples[2*k] += pSum1;
        samples[2*k + 1] += pSum2;
    }

    return 2*qsToCompute;
}

int RigidBody::closestVertexIndex(const ofVec3f& worldPos) const {
    ofVec3f r = RInv * (worldPos - x);
    float minDistSq = numeric_limits<float>::max();
    int minIndex = -1;
    for (int i = 0; i < mesh.getNumVertices(); i++) {
        float distSq = (mesh.getVertex(i) - r).lengthSquared();
        if (distSq < minDistSq) {
            minDistSq = distSq;
            minIndex = i;
        }
    }
    return minIndex;
}

ofVec3f RigidBody::getXi(int i) const {
    return x + R * mesh.getVertex(i);
}

ofVec3f RigidBody::getVi(int i) const {
    return v + w.crossed(R * mesh.getVertex(i));
}
