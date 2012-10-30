#pragma once

/*
 
 This Example was originally created by Kyle McDonald ( http://kylemcdonald.net/ ) 
 I ( Ben McChesney http://benmcchesney.com/blog ) just added some stuff to it later
 Kyle did all the hard work and deserves all the credit for making an awesome easy to use addon ! :D
 
 */

#include "ofMain.h"
#include "ofxFft.h"
#include "RadialBar.h"
#include "FftRange.h"
#include "RadialFft.h"
#include "ofxUI.h"
#include "Tweenzor.h"

#define MIC 0
#define NOISE 1
#define SINE 2
#define ANIMATE 3

#include "ofxOsc.h"
#include "ControllerCursor.h"

// listen on port 12345
#define PORT 12345
#define NUM_MSG_STRINGS 20



class testApp : public ofBaseApp {
public:
	void setup();
    void update() ;
	void plot(float* array, int length, float scale, float offset);
	void audioReceived(float* input, int bufferSize, int nChannels);
	void draw();
	void keyPressed(int key);

	int plotHeight, bufferSize;

	ofxFft* fft;

	float* audioInput;

	float appWidth;
	float appHeight;

	ofImage spectrogram;
	int spectrogramOffset;
    
	int mode;
    
    //Added after
    vector<RadialFft> visuals ;
    RadialFft radialFft;
    
    //ofxUI
    ofxUICanvas *fftGUI;
	void fftGUIEvent(ofxUIEventArgs &e) ;
    
    ofFbo screenFbo ;
    float clearAlpha ;
    
    bool bGuiEnabled ; 

    float amplitudeScale ;
    
    
    ofShader shader;
    ofImage image1 ;
    ofFbo fbo1 ;
    
    //OSC things
    ofxOscReceiver receiver;
    
    int current_msg_string;
    string msg_strings[NUM_MSG_STRINGS];
    float timers[NUM_MSG_STRINGS];

    ControllerCursor cursor0 ;
    ControllerCursor cursor1 ;

    //float cursorX , cursorY ;
    //string cursorState ;
    
    void updateOSC( ) ;
    
    
};
