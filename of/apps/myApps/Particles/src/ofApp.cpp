#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    pMin.x = BALL_RADIUS;
    pMin.y = BALL_RADIUS;
    pMax.x = ofGetWidth() - BALL_RADIUS;
    pMax.y = ofGetHeight() - BALL_RADIUS;

    // initialize ball positions and velocities to random values
    const float MAX_SPEED = 500.f;
    for (int i = 0; i < N_BALLS; i++) {
        p[i].x = pMin.x + ofRandomuf() * (pMax.x - pMin.x);
        p[i].y = pMin.y + ofRandomuf() * (pMax.y - pMin.y);
        float speed = ofRandomuf() * MAX_SPEED;
        float theta = ofRandomuf() * TWO_PI;
        v[i].x = speed * cosf(theta);
        v[i].y = speed * sinf(theta);
    }

    gravity = ofVec2f(0.f, 400.f);

    ofSetFrameRate(60);
}

void ofApp::updatePosition(ofVec2f* p, ofVec2f* v, float* dt, float rFactor) {
    ofVec2f pAfterDt = *p + *dt * *v;
    ofVec2f pNext = pAfterDt;
    ofVec2f vNext = *v;

    float dtToProcess = *dt;
    if (pAfterDt.x < pMin.x) {
        float dtX = (pMin.x - p->x) / v->x;     // time to left wall
        if (dtX < dtToProcess) {
            dtToProcess = dtX;
            pNext.x = pMin.x;
            pNext.y = p->y + dtX * v->y;
            vNext.x *= -rFactor;
        }
    } else if (pAfterDt.x > pMax.x) {
        float dtX = (pMax.x - p->x) / v->x;     // time to right wall
        if (dtX < dtToProcess) {
            dtToProcess = dtX;
            pNext.x = pMax.x;
            pNext.y = p->y + dtX * v->y;
            vNext.x *= -rFactor;
        }
    }
    if (pAfterDt.y < pMin.y) {
        float dtY = (pMin.y - p->y) / v->y;     // time to top wall
        if (dtY < dtToProcess) {
            dtToProcess = dtY;
            pNext.y = pMin.y;
            pNext.x = p->x + dtY * v->x;
            vNext.y *= -rFactor;
        }
    } else if (pAfterDt.y > pMax.y) {
        float dtY = (pMax.y - p->y) / v->y;     // time to bottom wall
        if (dtY < dtToProcess) {
            dtToProcess = dtY;
            pNext.y = pMax.y;
            pNext.x = p->x + dtY * v->x;
            vNext.y *= -rFactor;
        }
    }
    *p = pNext;
    *v = vNext;
    *dt -= dtToProcess;
}

//--------------------------------------------------------------
void ofApp::update(){
    float dt = ofGetLastFrameTime();

    // update ball velocities
    for (int i = 0; i < N_BALLS; i++) {
        v[i] += dt * gravity;
    }

    // update ball positions
    for (int i = 0; i < N_BALLS; i++) {
        float dtRemain = dt;
        do {
            updatePosition(&p[i], &v[i], &dtRemain, 0.6f);
        } while (dtRemain > 0.f);
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    ofSetColor(255, 0, 0);
    ofFill();

    // draw balls
    for (int i = 0; i < N_BALLS; i++) {
        ofCircle(p[i], BALL_RADIUS);
    }

    ofDrawBitmapString(ofToString(ofGetFrameRate()) + "fps", 10, 15);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
