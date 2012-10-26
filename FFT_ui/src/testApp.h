#pragma once

#include "ofMain.h"
#include "ofxFft.h"
#include "FftRange.h"
#include "RadialFft.h"
#include "ofxUI.h"
#include "Tweenzor.h"

#define MIC 0
#define NOISE 1
#define SINE 2
#define ANIMATE 3

class testApp : public ofBaseApp{
	public:
        void setup();
        void update() ;
        void draw();
        void keyPressed(int key);
    
        //FFT Vars
        void plot(float* array, int length, float scale, float offset);
        void audioReceived(float* input, int bufferSize, int nChannels);
        
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
        
        
        ofFbo screenFbo ;
        float clearAlpha ;
        
        bool bGuiEnabled ;
        
        float amplitudeScale ;
    
        //ofxUI
        ofxUICanvas *fftControls;
        ofxUICanvas *fftGUI;
        void fftControlsEvent(ofxUIEventArgs &e) ;
        void fftGUIEvent(ofxUIEventArgs &e) ;
        
        void disableAllofxUI( ) ;
    
        float fftGraphScale ; 
        float hueTimeMultiplier ; 
};
