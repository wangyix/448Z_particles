#pragma once

#include <mutex>

#include "ofMain.h"
#include "RingBuffer.h"

#define N_BALLS 10
#define BALL_RADIUS 10.0     // in pixels

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
    float updatePosition(ofVec2f* p, ofVec2f* v, float dt, float rFactor, float* vn);

private:
    ofVec2f pMin, pMax;                 // bounds on ball position
    ofVec2f p[N_BALLS];                 // ball positions
    ofVec2f v[N_BALLS];                 // ball velocities
    float rFactors[N_BALLS];            // ball restitution factors
    ofVec2f gravity;                    // acceleration due to gravity

    //RingBuffer<float, 1470> audioBuffer;      // 44100/60 * 2 channels, so a frame's worth of samples
    RingBuffer<WavInstance, 64> wavInstances;   // each entry is the sample index that wav instance is currently being played at
    std::mutex wavInstancesLock;
};
