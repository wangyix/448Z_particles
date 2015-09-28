#pragma once

#include <mutex>

#include "ofMain.h"
#include "RingBuffer.h"

#define N_BALLS 10
#define BALL_RADIUS 10.0     // in pixels
#define MAX_WAV_INSTANCES 64

#define AUDIO_SAMPLE_RATE 44100
#define WAV_SAMPLES 4876

#define GRAVITY_MAG 400.0

#define MOUSE_CURSOR_MASS 200000.0    // how attractive the cursor is when held down; unit is abitrary

struct WavInstance {
    WavInstance() : sampleAt(0), atten(0.f) {}
    WavInstance(int sampleAt, float atten) : sampleAt(sampleAt), atten(atten) {}
    int sampleAt;
    float atten;
};

class ofApp : public ofBaseApp{
public:
	void setup() override;
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
    void updatePMinMax();

    // Will update a ball's position.  If no collision occurs within dt, the ball will simply move
    // to p + dt * v, and the function will return dt. If a collision occurs within dt, the ball
    // will move to the position where the collision occurs, v will be updated to reflect the
    // collision, and the function will return the time until the collision.
    // rFactor is the restitution factor of the ball; vn will be filled with the normal velocity
    // at which the collision occurred if one occurs.
    float updatePositionUntilCollision(ofVec2f* p, ofVec2f* v, float dt, float rFactor, float* vn);

private:
    ofVec2f pMin, pMax;                 // bounds on ball position
    ofVec2f p[N_BALLS];                 // ball positions
    ofVec2f v[N_BALLS];                 // ball velocities
    float rFactors[N_BALLS];            // ball restitution factors

    ofVec2f gravity;                    // acceleration due to gravity
    
    bool mouseAttract;
    ofVec2f mousePos;

    // Each entry is the sample index that wav instance is currently being played at.  A negative
    // value indicates a delay before that instance begins playing.
    RingBuffer<WavInstance, MAX_WAV_INSTANCES> wavInstances;
    std::mutex wavInstancesLock;
};
