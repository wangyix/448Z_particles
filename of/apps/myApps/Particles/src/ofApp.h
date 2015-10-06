#pragma once

#include <mutex>

#include "ofMain.h"
#include "RingBuffer.h"

#define N_BALLS 100
#define BALL_RADIUS 10.0     // in pixels
#define MAX_WAV_INSTANCES 64

#define BOX_ZMIN -400.0
#define BOX_ZMAX 0.0

#define GRAVITY_MAG 400.0

#define AUDIO_SAMPLE_RATE 44100
#define WAV_SAMPLES 4876

#define NUM_MATERIALS 4

#define MOUSE_CURSOR_MASS 200000.0    // how attractive the cursor is when held down; unit is abitrary

struct WavInstance {
    WavInstance() : sampleAt(0), atten(0.f) {}
    WavInstance(int sampleAt, float atten) : sampleAt(sampleAt), atten(atten) {}
    int sampleAt;
    float atten;
};

struct Sphere {
    ofVec3f p;
    ofVec3f v;
    float rFactor;
    float m;
    float yMod;
    float pRatio;

    ofColor color;
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
    int wallCollide(const ofVec3f& c1, float r1, const ofVec3f& v1, float tMin, float* t);
    void updateBallCollisions(int index, float tMin);

private:
    ofLight pointLight;
    ofPlanePrimitive leftWall, rightWall, bottomWall, topWall, backWall;

    ofVec3f pMin, pMax;                 // bounds on ball position
    Sphere balls[N_BALLS];

    ofVec3f gravity;                    // acceleration due to gravity
    
    bool attract;
    ofVec3f attractPos;

    float* ballCollisionTable;
    Collision wallCollisionTable[N_BALLS];

    // Each entry is the sample index that wav instance is currently being played at.  A negative
    // value indicates a delay before that instance begins playing.
    RingBuffer<WavInstance, MAX_WAV_INSTANCES> wavInstances;
    std::mutex wavInstancesLock;
};
