#pragma once

#include <mutex>

#include "ofMain.h"
#include "RingBuffer.h"

#define EXTENSION_SCENE 1


#define N_BALLS 12
//#define MAX_WAV_INSTANCES 64

#define PIXELS_PER_METER 50.0

#define BOX_ZMIN (-400.0 / PIXELS_PER_METER)
#define BOX_ZMAX 0.0

#define GRAVITY_MAG 6.0

#define AUDIO_SAMPLE_RATE 44100
#define CHANNELS 2

#define WAV_SAMPLES 4876

#define NUM_MATERIALS 4

#define MOUSE_CURSOR_MASS 80.0    // how attractive the cursor is when held down; unit is abitrary

struct Material {
    Material(float density, float yMod, float pRatio, const ofColor& color)
        : density(density), yMod(yMod), pRatio(pRatio), color(color) {}
    float density;
    float yMod;
    float pRatio;
    ofColor color;
};

struct Sphere {
    void setMaterial(const Material& m);
    ofVec3f p;
    ofVec3f v;
    float m;
    float r;
    float rFactor;  // restitution factor
    float yMod;
    float pRatio;
    ofColor color;
};

struct Spring {
    Spring(int ballId1, int ballId2, float L, float k, float c)
        : ballId1(ballId1), ballId2(ballId2), L(L), k(k), c(c) {}
    int ballId1;
    int ballId2;
    float L;    // length
    float k;    // spring constant
    float c;    // damping coefficient
};

struct Collision {
    Collision() {}
    int id;
    float t;
};

class ofApp : public ofBaseApp{
public:
	void setup() override;
    void exit() override;
    void update() override;
    void draw() override;

    void keyPressed(int key) override;
    void keyReleased(int key) override;
    void mouseMoved(int x, int y) override;
    void mouseDragged(int x, int y, int button) override;
    void mousePressed(int x, int y, int button) override;
    void mouseReleased(int x, int y, int button) override;
    void windowResized(int w, int h) override;
    void dragEvent(ofDragInfo dragInfo) override;
    void gotMessage(ofMessage msg) override;

    void audioOut(float* output, int bufferSize, int nChannels) override;

private:
    int wallCollide(const Sphere& ball, float tMin, float* t);
    void updateBallCollisions(int index, float tMin);

private:
    ofLight pointLight;
    ofPlanePrimitive leftWall, rightWall, bottomWall, topWall, backWall;

    ofVec3f pMin, pMax;                 // bounds on ball position
    Sphere balls[N_BALLS];
    vector<Spring> springs;

    ofVec3f gravity;                    // acceleration due to gravity
    
    bool attract;
    ofVec3f attractPos;

    float* ballCollisionTable;
    Collision wallCollisionTable[N_BALLS];

    // Each entry is the sample index that wav instance is currently being played at.  A negative
    // value indicates a delay before that instance begins playing.
    RingBuffer<float, CHANNELS * AUDIO_SAMPLE_RATE> audioBuffer;
    std::mutex audioBufferLock;

    ofVec3f listenPos;
};
