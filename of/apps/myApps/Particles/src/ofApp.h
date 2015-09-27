#pragma once

#include "ofMain.h"
#include "RingBuffer.h"

#define N_BALLS 10
#define BALL_RADIUS 10.0     // in pixels

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
    void updatePosition(ofVec2f* p, ofVec2f* v, float* dt, float rFactor);

private:
    ofVec2f pMin, pMax;                 // bounds on ball position
    ofVec2f p[N_BALLS];                 // ball positions
    ofVec2f v[N_BALLS];                 // ball velocities
    float rFactors[N_BALLS];            // ball restitution factors
    ofVec2f gravity;                    // acceleration due to gravity

    RingBuffer<float, 1024> audioSamples;
};
