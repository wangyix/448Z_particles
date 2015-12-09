#include "ofApp.h"

#include <assert.h>
#include <algorithm>

static const Material STEEL_MATERIAL = Material(8940.f, 123.4f * 1e9f, 0.34f, ofColor(255, 0, 0, 255));
static const Material CERAMIC_MATERIAL = Material(2700.f, 72.f * 1e9f, 0.19f, ofColor(0, 255, 0, 255));
static const Material GLASS_MATERIAL = Material(2700.f, 62.f * 1e9f, 0.20f, ofColor(0, 0, 255, 255));
static const Material PLASTIC_MATERIAL = Material(1200.f, 2.4f * 1e9f, 0.37f, ofColor(255, 0, 255, 255));

static const Material materials[NUM_MATERIALS] = { STEEL_MATERIAL, CERAMIC_MATERIAL, GLASS_MATERIAL, PLASTIC_MATERIAL };

static const Material wallMaterial = STEEL_MATERIAL;


static const string sphereObjFileName = "C:/Users/wangyix/Desktop/GitHub/CS448Z/of/apps/myApps/Particles/models/sphere/sphere.obj";
static const string rodObjFileName = "C:/Users/wangyix/Desktop/GitHub/CS448Z/of/apps/myApps/Particles/models/rod/rod.obj";
static const string groundObjFileName = "C:/Users/wangyix/Desktop/GitHub/CS448Z/of/apps/myApps/Particles/models/ground/ground.obj";
static const string sphereModesFileName = "C:/Users/wangyix/Desktop/GitHub/CS448Z/of/apps/myApps/Particles/models/sphere/modes.txt";
static const string rodModesFileName = "C:/Users/wangyix/Desktop/GitHub/CS448Z/of/apps/myApps/Particles/models/rod/modes.txt";
static const string groundModesFileName = "C:/Users/wangyix/Desktop/GitHub/CS448Z/of/apps/myApps/Particles/models/ground/modes.txt";


/*static int secondsToSamples(float t) {
    return ((int)(t * AUDIO_SAMPLE_RATE)) * CHANNELS;
}*/

#define SECONDS_TO_SAMPLES(t) (((int)(t * AUDIO_SAMPLE_RATE)) * CHANNELS)

static const int AUDIO_SAMPLES_PAD = SECONDS_TO_SAMPLES(AUDIO_BUFFER_PAD_TIME);

//--------------------------------------------------------------
void ofApp::setup(){
    windowResized(ofGetWidth(), ofGetHeight());

    // initialize rigid bodies
    bodies.push_back(RigidBody(groundModesFileName, 2e11f, 0.4f, 1.f, 30.f, 1e-11f, groundObjFileName, PLASTIC_MATERIAL, 0.008f));
    //bodies.push_back(RigidBody(rodModesFileName, 7e10f, 0.3f, 1.f, 50.f, 1e-11f, rodObjFileName, PLASTIC_MATERIAL, 1.f));
    //sphereBodies.push_back(RigidBody(sphereModesFileName, 7e10f, 0.3f, 1.f, 30.f, 1e-11f, sphereObjFileName, PLASTIC_MATERIAL, 0.04f, true));
    //sphereBodies.push_back(RigidBody(sphereModesFileName, 7e10f, 0.3f, 1.f, 30.f, 1e-11f, sphereObjFileName, PLASTIC_MATERIAL, 0.06f, true));
    //sphereBodies.push_back(RigidBody(sphereModesFileName, 7e10f, 0.3f, 1.f, 30.f, 1e-11f, sphereObjFileName, PLASTIC_MATERIAL, 0.09f, true));

    bodies[0].x = 0.5f * pMin + 0.5f * pMax;
    //sphereBodies[0].x = 0.25f * pMin + 0.75f * pMax;// +ofVec3f(0.f, 2.f, 0.f);
    //sphereBodies[1].x = sphereBodies[0].x + ofVec3f(-0.2f, 0.f, 0.f);//0.5f * pMin + 0.5f * pMax;
    //sphereBodies[2].x = sphereBodies[1].x + ofVec3f(-0.2f, 0.f, 0.f);//0.5f * pMin + 0.5f * pMax;
    //bodies[2].x = 0.75f * pMin + 0.25f * pMax;
    //bodies[0].rotate(PI / 6.f, ofVec3f(0.f, 0.f, 1.f));
    //bodies[0].rotate(-PI / 6.f, ofVec3f(1.f, 0.f, 0.f));
    //bodies[0].L = ofVec3f(0.f, 0.f, 1000.f);
    //bodies[0].w = bodies[0].IInv * bodies[0].L;

    //bodies[1].x = 0.7f * pMin + 0.3f * pMax + ofVec3f(0.f, 2.f, 0.f);
    //bodies[1].L = ofVec3f(0.f, 0.f, 1000.f);
    //bodies[1].w = bodies[0].IInv * bodies[0].L;


    for (RigidBody& body : bodies) {
        allBodies.push_back(&body);
    }
    for (RigidBody& sphereBody : sphereBodies) {
        allBodies.push_back(&sphereBody);
    }

    // initialize light
    ofSetSmoothLighting(true);
    pointLight.setDiffuseColor(ofFloatColor(.85, .85, .55));
    pointLight.setSpecularColor(ofFloatColor(1.f, 1.f, 1.f));

    gravity = ofVec3f(0.f, GRAVITY_MAG, 0.f);
    attract = false;

    listenPos = ofVec3f(0.5f * (pMin.x + pMax.x), 0.5f * (pMin.y + pMax.y), BOX_ZMAX);

    ofSoundStreamSetup(CHANNELS, 0, AUDIO_SAMPLE_RATE, BUFFER_SIZE, 4);
    //ofSetFrameRate(100);

    audioBuffer.pushZeros(AUDIO_SAMPLES_PAD);

    qScale = 200.f;
    accelAudioScale = 0.08f;
}

//--------------------------------------------------------------
void ofApp::exit() {
}

enum WALL_ID{ NONE=-1, XMIN=-2, XMAX=-3, YMIN=-4, YMAX=-5, ZMIN=-6, ZMAX=-7 };

int ofApp::particleCollideWall(const ofVec3f& p, const ofVec3f& v, float tMin, float* t) {
    /*assert(tMax > 0.f);
    float tt = tMax;
    int id = NONE;
    if (v.x > 0.f) {
        float t = (pMax.x - p.x) / v.x;
        if (tMin < t && t < tt) {
            tt = t;
            id = XMAX;
        }
    } else if (v.x < 0.f) {
        float t = (pMin.x - p.x) / v.x;
        if (tMin < t && t < tt) {
            tt = t;
            id = XMIN;
        }
    }
    if (v.y > 0.f) {
        float t = (pMax.y - p.y) / v.y;
        if (tMin < t && t < tt) {
            tt = t;
            id = YMAX;
        }
    } else if (v.y < 0.f) {
        float t = (pMin.y - p.y) / v.y;
        if (tMin < t && t < tt) {
            tt = t;
            id = YMIN;
        }
    }
    if (v.z > 0.f) {
        float t = (pMax.z - p.z) / v.z;
        if (tMin < t && t < tt) {
            tt = t;
            id = ZMAX;
        }
    } else if (v.z < 0.f) {
        float t = (pMin.z - p.z) / v.z;
        if (tMin < t && t < tt) {
            tt = t;
            id = ZMIN;
        }
    }
    if (tt < tMax) {
        *t = tt;
    }
    return id;*/
    return sphereCollideWall(p, v, 0.f, tMin, t);
}

int ofApp::sphereCollideWall(const ofVec3f& p, const ofVec3f& v, float r, float tMin, float* t) {
    int id = NONE;
    if (v.x > 0.f) {
        float tt = (pMax.x - r - p.x) / v.x;
        if (tMin <= tt && tt < *t) {
            *t = tt;
            id = XMAX;
        }
    } else {
        float tt = (pMin.x + r - p.x) / v.x;
        if (tMin <= tt && tt < *t) {
            *t = tt;
            id = XMIN;
        }
    }
    if (v.y > 0.f) {
        float tt = (pMax.y - r - p.y) / v.y;
        if (tMin <= tt && tt < *t) {
            *t = tt;
            id = YMAX;
        }
    } else {
        float tt = (pMin.y + r - p.y) / v.y;
        if (tMin <= tt && tt < *t) {
            *t = tt;
            id = YMIN;
        }
    }
    if (v.z > 0.f) {
        float tt = (pMax.z - r - p.z) / v.z;
        if (tMin <= tt && tt < *t) {
            *t = tt;
            id = ZMAX;
        }
    } else {
        float tt = (pMin.z + r - p.z) / v.z;
        if (tMin <= tt && tt < *t) {
            *t = tt;
            id = ZMIN;
        }
    }
    return id;
}

// will only return smaller root or no collision
static bool spheresCollide(const ofVec3f& p1, const ofVec3f& v1, float r1,
                           const ofVec3f& p2, const ofVec3f& v2, float r2,
                           float tMin, float* t) {
    ofVec3f cDiff = p1 - p2;
    ofVec3f vDiff = v1 - v2;
    float rSum = r1 + r2;
    float a = vDiff.lengthSquared();
    if (a == 0.f) {
        return false;
    }
    float b_over_2 = cDiff.dot(vDiff);
    float c = cDiff.lengthSquared() - rSum*rSum;
    float D_over_4 = b_over_2*b_over_2 - a*c;
    if (D_over_4 < 0.f) {
        return false;
    }
    float tt;
    if (b_over_2 > 0.f) {
        float z = -b_over_2 - sqrtf(D_over_4);
        tt = z / a;
    } else {
        float z = -b_over_2 + sqrtf(D_over_4);
        tt = c / z;
    }
    if (tMin <= tt && tt < *t) {
        *t = tt;
        return true;
    }
    return false;
}


static float computeTau(float r1Inv, float m1Inv, float v1, float E1,
    float r2Inv, float m2Inv, float v2, float E2, float V) {
    float r = 1.f / (r1Inv + r2Inv);
    float m = 1.f / (m1Inv + m2Inv);
    float E = 1.f / ((1 - v1*v1) / E1 + (1 - v2*v2) / E2);
    return 2.87f * pow((m*m / (r*E*E*abs(V))), 0.2);
}

static float computeSConst(const ofVec3f& p, float r, float m,
                           const ofVec3f& listenPos, float tau, const ofVec3f& impulse) {
    ofVec3f toListener = listenPos - p;
    float dist = toListener.length();
    ofVec3f toListenerDir = toListener / dist;

    float J = impulse.length();
    ofVec3f VDir = impulse / J;

    float pConst = 1.2f * r * r * r * VDir.dot(toListenerDir) / (2.f * 330.f * dist);
    float d2Vdt2Const = PI / (2.f * m * tau) * abs(J);
    float SConst = -12.f / (tau * tau);

    float combinedConst = pConst * d2Vdt2Const * SConst;
    return combinedConst;
}

static ofVec3f wallIdToNormal(int wallId) {
    ofVec3f n;
    switch (wallId) {
    case XMIN:
        n = ofVec3f(1.f, 0.f, 0.f);
        break;
    case XMAX:
        n = ofVec3f(-1.f, 0.f, 0.f);
        break;
    case YMIN:
        n = ofVec3f(0.f, 1.f, 0.f);
        break;
    case YMAX:
        n = ofVec3f(0.f, -1.f, 0.f);
        break;
    case ZMIN:
        n = ofVec3f(0.f, 0.f, 1.f);
        break;
    case ZMAX:
        n = ofVec3f(0.f, 0.f, -1.f);
        break;
    default:
        assert(false);
        break;
    }
    return n;
}

//--------------------------------------------------------------
void ofApp::update() {
    float dt = min(0.01, ofGetLastFrameTime());
    if (dt <= 0.f) return;

    // apply non-rotational forces to bodies
    for (RigidBody* bodyPtr : allBodies) {
        RigidBody& body = *bodyPtr;
        body.v += gravity * dt;
        if (attract) {
            ofVec3f toAttractPos = attractPos - body.x;
            float dist = toAttractPos.length();
            if (dist >= 0.01f) {
                toAttractPos /= dist;
                body.v += (toAttractPos * MOUSE_CURSOR_MASS / dist) * dt;
            }
        }
        body.P = body.m * body.v;
    }

    const float e = 0.5f;   // coefficient of restitution

    float qSums[AUDIO_SAMPLE_RATE];    // 1 second worth
    memset(qSums, 0, AUDIO_SAMPLE_RATE*sizeof(float));
    int qsComputed = 0;

    // compute collisions of vertices against walls
    for (RigidBody& body : bodies) {

        vector<VertexImpulse> impulses;

        int i_c = -1;                           // index of the vertex that collides
        ofVec3f ri_c(0.f, 0.f, 0.f);            // world ri of the vertex that collides
        ofVec3f xi_c(0.f, 0.f, 0.f);            // world position of vertex that collides
        ofVec3f vi_c(0.f, 0.f, 0.f);            // world velocity of vertex that collides
        while (true) {
            // find vertex with earliest wall collision, if any
            float dt_c = dt;  // collision will occur dt_c from now
            int wallId = NONE;
            for (int i = 0; i < body.mesh.getNumVertices(); i++) {
                ofVec3f ri = body.R * body.mesh.getVertex(i);
                ofVec3f xi = body.x + ri;
                ofVec3f vi = body.v + (body.w.crossed(ri));
                float t;
                int id = particleCollideWall(xi, vi, -100000000.f, &dt_c);
                if (id != NONE) {
                    wallId = id;
                    i_c = i;
                    ri_c = ri;
                    xi_c = xi;
                    vi_c = vi;
                }
            }

            // compute impulse imparted by collision at this vertex, if any,
            // and accumulate effect of impulse into P, L
            if (wallId != NONE) {
                ofVec3f n = wallIdToNormal(wallId);

                float j = -(1.f + e)*(vi_c.dot(n)) /
                    (1.f / body.m + ((body.IInv * (ri_c.crossed(n))).crossed(ri_c)).dot(n));

                ofVec3f impulse = j*n;

                // update linear, angular momentum with impulse
                body.P += impulse;
                body.L += (ri_c.crossed(impulse));
                // update linear, angular velocities from momentum
                body.v = (body.P / body.m);
                body.w = (body.IInv * body.L);

                impulses.emplace_back(i_c, impulse);

            } else {
                // no collision
                break;
            }
        }

        body.step(dt);
        body.stepW(dt);

        // compute modal noise amplitudes
        int qsComputedThisBody = body.stepAudio(dt, impulses, 1.f / AUDIO_SAMPLE_RATE, qSums);
        if (qsComputedThisBody > qsComputed) {
            qsComputed = qsComputedThisBody;
        }
    }

    // ============================================================================================

    float accelAudioSamples[SECONDS_TO_SAMPLES(0.5)];
    memset(accelAudioSamples, 0, SECONDS_TO_SAMPLES(0.5)*sizeof(float));
    int accelAudioStart = SECONDS_TO_SAMPLES(0.5);
    int accelAudioEnd = 0;

    const int numSpheres = sphereBodies.size();
    vector<vector<VertexImpulse>> sphereImpulses(numSpheres);   // keeps track of impulse(s) applied to each sphereBody

    float dtProcessed = 0.f;
    while (true) {
        // find next collision
        int i_c = -1;       // index of sphere that collides
        int j_c = -1;       // index of other sphere that collides, or wall id that collides
        float dt_c = dt - dtProcessed;
        for (int i = 0; i < numSpheres; i++) {
            RigidBody& sphere1 = sphereBodies[i];
            // collide with other spheres
            for (int j = i + 1; j < numSpheres; j++) {
                RigidBody& sphere2 = sphereBodies[j];
                if (spheresCollide(sphere1.x, sphere1.v, sphere1.r, sphere2.x, sphere2.v, sphere2.r, 0.f, &dt_c)) {
                    i_c = i;
                    j_c = j;
                }
            }
            // collide with walls
            int wallId = sphereCollideWall(sphere1.x, sphere1.v, sphere1.r, 0.f, &dt_c);
            if (wallId != NONE) {
                i_c = i;
                j_c = wallId;
            }
        }

        // step all spheres forward until collision
        if (dt_c > 0.f) {
            for (int i = 0; i < numSpheres; i++) {
                RigidBody& body = sphereBodies[i];
                body.step(dt_c);
                body.stepW(dt_c);
            }
        }

        if (i_c == -1) {
            // all collisions (if any) have been resolved
            break;
        }

        RigidBody& sphereBody = sphereBodies[i_c];

        float tau;
        float SConst;
        ofVec3f contactPos;
        if (j_c < NONE) {   // sphere-wall collision
            int wallId = j_c;
            ofVec3f n = wallIdToNormal(wallId);

            // compute impulse
            float vn = sphereBody.v.dot(n);
            ofVec3f impulse = (1.f + e) * abs(vn) * sphereBody.m * n;

            contactPos = sphereBody.x - sphereBody.r * n;

            // apply impulse to sphere
            sphereBody.P += impulse;
            sphereBody.v = (sphereBody.P / sphereBody.m);
            
            // compute tau, SConst for sphere-wall collision
            tau = computeTau(1.f / sphereBody.r, 1.f / sphereBody.m, sphereBody.material.nu, sphereBody.material.E,
                0.f, 0.f, wallMaterial.nu, wallMaterial.E, vn);
            SConst = computeSConst(sphereBody.x, sphereBody.r, sphereBody.m, listenPos, tau, impulse);

            // record impulse for modal sound computation later
            sphereImpulses[i_c].emplace_back(sphereBody.closestVertexIndex(contactPos), impulse);
        
        } else {    // sphere-sphere collision
            RigidBody& sphereBody2 = sphereBodies[j_c];

            ofVec3f n = (sphereBody.x - sphereBody2.x).normalized();
            float vn = (sphereBody2.v - sphereBody.v).dot(n);
            float J = (1.f + e)*vn / (1.f / sphereBody.m + 1.f / sphereBody2.m);
            ofVec3f impulse = J * n;

            contactPos = sphereBody.x - sphereBody.r * n;

            // apply impulse to spheres
            sphereBody.P += impulse;
            sphereBody.v = (sphereBody.P / sphereBody.m);
            sphereBody2.P -= impulse;
            sphereBody2.v = (sphereBody2.P / sphereBody2.m);

            // compute tau, SConst
            tau = computeTau(1.f / sphereBody.r, 1.f / sphereBody.m, sphereBody.material.nu, sphereBody.material.E,
                1.f / sphereBody2.r, 1.f / sphereBody2.m, sphereBody2.material.nu, sphereBody2.material.E, abs(vn));
            float SConst1 = computeSConst(sphereBody.x, sphereBody.r, sphereBody.m, listenPos, tau, impulse);
            float SConst2 = computeSConst(sphereBody2.x, sphereBody2.r, sphereBody2.m, listenPos, tau, -impulse);
            SConst = SConst1 + SConst2;
SConst *= 20.f;

            // record impulse for modal sound computation later
            sphereImpulses[i_c].emplace_back(sphereBody.closestVertexIndex(contactPos), impulse);
            sphereImpulses[j_c].emplace_back(sphereBody2.closestVertexIndex(contactPos), -impulse);
        }

        // compute acceleration noise samples for this collision
        // TODO: check for possible overrun of accelAudioSamples array
        int i = SECONDS_TO_SAMPLES((contactPos - listenPos).length() / 330.f);  // retarded time
        if (i < accelAudioStart) {
            accelAudioStart = i;
        }
        for (float t = 0.f; t < tau; t += 1.f / AUDIO_SAMPLE_RATE) {
            float sample = (accelAudioScale * SConst) * (t - 0.5f * tau) * sin(PI*t / tau);
            for (int j = 0; j < CHANNELS; j++) {
                accelAudioSamples[i++] += sample;
            }
        }
        if (i > accelAudioEnd) {
            accelAudioEnd = i;
        }

        dtProcessed += dt_c;
    }

    for (int i = 0; i < numSpheres; i++) {
        RigidBody& body = sphereBodies[i];

        // compute modal amplitues from impulses applied
        int qsComputedThisBody = body.stepAudio(dt, sphereImpulses[i], 1.f / AUDIO_SAMPLE_RATE, qSums);
        if (qsComputedThisBody > qsComputed) {
            qsComputed = qsComputedThisBody;
        }
    }


    // scale qSums to get audio samples
    float audioSamples[CHANNELS * AUDIO_SAMPLE_RATE];   // 1 second worth
    int audioSamplesComputed = 0;
float maxSample = 0.f;
float minSample = 0.f;
    for (int k = 0; k < qsComputed; k++) {
        for (int i = 0; i < CHANNELS; i++) {
            audioSamples[audioSamplesComputed++] = qScale * qSums[k];
        }
maxSample = max(maxSample, (qScale * qSums[k]));
minSample = min(minSample, (qScale * qSums[k]));
    }
if (maxSample > 1.f || minSample < -1.f) {
    printf("%f\t\t%f ------------------------\n", maxSample, minSample);
}else if (maxSample > 0.1f || minSample < -0.1f) {
    printf("%f\t\t%f\n", maxSample, minSample);
}
    
    // check if audioSamples has trailing zeros for sync adjustment
    const float threshold = 1e-9f;
    int trailingZerosStartAt = audioSamplesComputed;
    while (trailingZerosStartAt > 0 && abs(audioSamples[trailingZerosStartAt - 1]) < threshold) {
        trailingZerosStartAt--;
    }
    assert(trailingZerosStartAt % CHANNELS == 0);
    
    // push audio samples
    int audioSamplesToPush = audioSamplesComputed;
    audioBufferLock.lock();
    // only add/remove zeros if number of trailing zeros is sufficiently high;
    // this prevents situations where you remove a small part of a sin wave where it crosses 0
    int trailingZeros = audioSamplesComputed - trailingZerosStartAt;
    if (trailingZeros > SECONDS_TO_SAMPLES(0.5f * dt)) {
        int bufferSizeAfterPush = audioBuffer.size() + audioSamplesToPush;
        int zerosToAdd = AUDIO_SAMPLES_PAD - bufferSizeAfterPush;
        //printf("%d\n", zerosToAdd);
        if (zerosToAdd > 0) {
            audioSamplesToPush += zerosToAdd;
            memset(&audioSamples[audioSamplesComputed], 0, zerosToAdd*sizeof(float));
        } else if (zerosToAdd < 0) {
            int zerosToRemove = -zerosToAdd;
            audioSamplesToPush -= min(zerosToRemove, trailingZeros);
        }
    } 
    audioBuffer.push(audioSamples, audioSamplesToPush);
    audioBufferLock.unlock();




    // push acceleration noise audio samples
    if (accelAudioEnd > accelAudioStart) {
        accelAudioBufferLock.lock();
        int additionalCapcityNeeded = accelAudioEnd - accelAudioBuffer.size();
        if (additionalCapcityNeeded > 0) {
            accelAudioBuffer.pushZeros(additionalCapcityNeeded);
            accelAudioEnd = accelAudioBuffer.size();  // in case audioBuffer couldn't push the requested amount of zeros 
        }
        auto iter = accelAudioBuffer.at(accelAudioStart);
        for (int i = accelAudioStart; i < accelAudioEnd; i++) {
            *iter += accelAudioSamples[i];
            ++iter;
        }
        accelAudioBufferLock.unlock();
    }
}

static void drawCylinder(const ofVec3f& p1, const ofVec3f& p2) {
    ofVec3f targetDir = p2 - p1;
    float l = targetDir.length();
    targetDir /= l;

    ofCylinderPrimitive cylinder;
    cylinder.setPosition(0.f, 0.f, 0.f);
    cylinder.set(0.1f * PIXELS_PER_METER, l * PIXELS_PER_METER);

    ofVec3f cylDir = ofVec3f(0.f, 1.f, 0.f);
    ofVec3f rotateAxis = cylDir.crossed(targetDir);
    float degreesRotate = acosf(cylDir.dot(targetDir)) / PI * 180.f;

    cylinder.rotate(degreesRotate, rotateAxis);
    cylinder.setPosition(0.5f * (p1 + p2) * PIXELS_PER_METER);
    ofSetColor(0, 200, 200);
    cylinder.draw();
}

static void drawPoint(const ofVec3f& p, float size, const ofColor& color) {
    ofBoxPrimitive box(size*PIXELS_PER_METER, size*PIXELS_PER_METER, size*PIXELS_PER_METER);
    box.setPosition(p*PIXELS_PER_METER);
    ofSetColor(color);
    box.draw();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    
    ofEnableLighting();
    pointLight.enable();

    ofEnableDepthTest();
    
    ofSetColor(200, 200, 200);
    leftWall.draw();
    rightWall.draw();
    backWall.draw();
    topWall.draw();
    bottomWall.draw();
    
    for (RigidBody* bodyPtr : allBodies) {
        RigidBody& body = *bodyPtr;
        ofSetColor(body.material.color);
        ofPushMatrix();
        const ofMatrix3x3& R = body.R;
        const ofVec3f& T = body.x;
        ofMatrix4x4 objToWorld(R.a, R.d, R.g, 0.f,
                               R.b, R.e, R.h, 0.f,
                               R.c, R.f, R.i, 0.f,
                               T.x, T.y, T.z, 1.f / PIXELS_PER_METER);
        ofLoadMatrix(objToWorld * viewMatrix);
        body.mesh.draw();
        ofPopMatrix();
    }

    ofDisableLighting();
    ofSetColor(255, 255, 255);
    ofDrawBitmapString(ofToString(ofGetFrameRate()) + "fps", 10, 15);
}

//--------------------------------------------------------------
void ofApp::audioOut(float* output, int bufferSize, int nChannels) {
    float buffer1[CHANNELS * BUFFER_SIZE];
    float buffer2[CHANNELS * BUFFER_SIZE];

    {
        audioBufferLock.lock();
        int samplesPopped = audioBuffer.pop(buffer1, bufferSize * CHANNELS);
        audioBufferLock.unlock();
        int samplesRemaining = CHANNELS * bufferSize - samplesPopped;
        if (samplesRemaining > 0) {
            memset(&buffer1[samplesPopped], 0, samplesRemaining * sizeof(float));
        }
    }
    {
        accelAudioBufferLock.lock();
        int samplesPopped = accelAudioBuffer.pop(buffer2, bufferSize * CHANNELS);
        accelAudioBufferLock.unlock();
        int samplesRemaining = CHANNELS * bufferSize - samplesPopped;
        if (samplesRemaining > 0) {
            memset(&buffer2[samplesPopped], 0, samplesRemaining * sizeof(float));
        }
    }

    // add samples from modal audio buffer and acceleration audio buffer
    for (int i = 0; i < CHANNELS * BUFFER_SIZE; i++) {
        output[i] = buffer1[i] + buffer2[i];
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    RigidBody& body = *allBodies[0];
    float& alpha = *const_cast<float*>(&body.alpha);
    float& beta = *const_cast<float*>(&body.beta);
    int& nModes = body.nModesOnly;
    bool& topModes = body.topModes;

    switch (key) {
    case '1':
        beta /= 1.05f;
        printf("beta = %e\n", beta);
        break;
    case '2':
        beta *= 1.05f;
        printf("beta = %e\n", beta);
        break;
    case '3':
        beta /= 10.f;
        printf("beta = %e\n", beta);
        break;
    case '4':
        beta *= 10.f;
        printf("beta = %e\n", beta);
        break;
    case '5':
        alpha /= 1.05f;
        printf("alpha = %e\n", alpha);
        break;
    case '6':
        alpha *= 1.05f;
        printf("alpha = %e\n", alpha);
        break;
    case '8':
        nModes -= 2;
        break;
    case '9':
        nModes += 2;
        break;
    case '0':
        topModes = !topModes;
        break;
    case '-':
        qScale /= 1.1f;
        printf("qScale = %f\n", qScale);
        break;
    case '=':
        qScale *= 1.1f;
        printf("qScale = %f\n", qScale);
        break;
    case ',':
        accelAudioScale /= 1.1f;
        printf("accelAudioScale = %f\n", accelAudioScale);
        break;
    case '.':
        accelAudioScale *= 1.1f;
        printf("accelAudioScale = %f\n", accelAudioScale);
        break;
    default:
        break;
    }

    if (key == '8' || key == '9' || key == '0') {
        if (topModes) {
            printf("top %d modes only\n", nModes);
        } else {
            printf("bottom %d modes only\n", nModes);
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    switch (key) {
    case OF_KEY_LEFT:
        gravity = ofVec3f(-GRAVITY_MAG, 0.f, 0.f);
        break;
    case OF_KEY_RIGHT:
        gravity = ofVec3f(GRAVITY_MAG, 0.f, 0.f);
        break;
    case OF_KEY_UP:
        gravity = ofVec3f(0.f, -GRAVITY_MAG, 0.f);
        break;
    case OF_KEY_DOWN:
        gravity = ofVec3f(0.f, GRAVITY_MAG, 0.f);
        break;
    default:
        break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    attractPos = ofVec3f(x / PIXELS_PER_METER, y / PIXELS_PER_METER, 0.5f * (BOX_ZMIN + BOX_ZMAX));
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    attract = true;
    attractPos = ofVec3f(x / PIXELS_PER_METER, y / PIXELS_PER_METER, 0.5f * (BOX_ZMIN + BOX_ZMAX));
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    attract = false;
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    ofSetupScreenPerspective(w, h, 60.f, BOX_ZMAX * PIXELS_PER_METER, BOX_ZMIN * PIXELS_PER_METER);
    viewMatrix = ofGetCurrentMatrix(OF_MATRIX_MODELVIEW);

    pMin.x = 0.f;
    pMax.x = w / PIXELS_PER_METER;
    pMin.y = 0.f;
    pMax.y = h / PIXELS_PER_METER;
    pMin.z = BOX_ZMIN;
    pMax.z = BOX_ZMAX;

    pointLight.setPosition(w / 2, 10.f, 0.5f*(BOX_ZMIN + BOX_ZMAX)*PIXELS_PER_METER);

    // initialize walls
    leftWall = ofPlanePrimitive((BOX_ZMAX - BOX_ZMIN)*PIXELS_PER_METER, h, 8, 8);
    rightWall = leftWall;
    leftWall.setPosition(0.f, h / 2, 0.5f*(BOX_ZMIN + BOX_ZMAX)*PIXELS_PER_METER);
    leftWall.rotate(90.f, 0.f, 1.f, 0.f);
    rightWall.setPosition(w, h / 2, 0.5f*(BOX_ZMIN + BOX_ZMAX)*PIXELS_PER_METER);
    rightWall.rotate(-90.f, 0.f, 1.f, 0.f);

    backWall = ofPlanePrimitive(w, h, 8, 8);
    backWall.setPosition(w / 2, h / 2, BOX_ZMIN*PIXELS_PER_METER);

    topWall = ofPlanePrimitive(w, (BOX_ZMAX - BOX_ZMIN)*PIXELS_PER_METER, 8, 8);
    bottomWall = topWall;
    topWall.setPosition(w / 2, 0.f, 0.5f*(BOX_ZMIN + BOX_ZMAX)*PIXELS_PER_METER);
    topWall.rotate(-90.f, 1.f, 0.f, 0.f);
    bottomWall.setPosition(w / 2, h, 0.5f*(BOX_ZMIN + BOX_ZMAX)*PIXELS_PER_METER);
    bottomWall.rotate(90.f, 1.f, 0.f, 0.f);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
