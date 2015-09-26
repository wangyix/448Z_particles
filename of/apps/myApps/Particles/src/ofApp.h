#pragma once

#include "ofMain.h"

#define N_BALLS 10
#define BALL_RADIUS 10.0     // in pixels

class ofApp : public ofBaseApp{
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

private:
    void updatePMinMax();
    void updatePosition(ofVec2f* p, ofVec2f* v, float* dt, float rFactor);

private:
    ofVec2f pMin, pMax;                  // bounds on ball position
    ofVec2f p[N_BALLS];                 // ball positions
    ofVec2f v[N_BALLS];                 // ball velocities
    float rFactors[N_BALLS];            // ball restitution factors
    ofVec2f gravity;                    // acceleration due to gravity
};
