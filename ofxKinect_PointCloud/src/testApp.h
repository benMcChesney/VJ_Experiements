#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"'

#include "ofxUI.h"
#include "ofxSyphon.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

#include "ofxOsc.h"

// listen on port 12345
#define PORT 12345
#define NUM_MSG_STRINGS 20

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	    
    //added for ofxUI
    ofxUICanvas *gui;
	void guiEvent(ofxUIEventArgs &e);
    
    float minBlobSize , maxBlobSize ;
    float pointCloudMinZ , pointCloudMaxZ ;
    
    float meshHueTimeMultiplier ;
    float meshHueRangeMin , meshHueRangeMax ;
    
    bool bToggleTrails ;
    bool bToggleCameraJitter ;
    
    //Camera 
    ofVec3f cameraOrigin ;
	ofEasyCam easyCam;
    float jitterForceX , jitterForceY ;
    float jitterTimeOffsetX , jitterTimeOffsetY ;
    float jitterCameraDistance ;
    
    //Syphon Stuff !
    ofTexture tex;
    
	ofxSyphonServer mainOutputSyphonServer;
	ofxSyphonServer individualTextureSyphonServer;
	
	ofxSyphonClient mClient;
    
    //Effects
    ofFbo trailFbo ;
    float fboFadeAmount ;
    float extrudeDepth ; 
    float extrudeNoiseStrength ;
    
    vector<ofFbo> pointCloudFbos ;
    int currentFboIndex ;
    
    bool bGuiEnabled ;
    
    //OSC FFT information
    ofxOscReceiver receiver;
    
    int current_msg_string;
    string msg_strings[NUM_MSG_STRINGS];
    
    //FFT
    float scaledAmplitude ;
    float amplitude ; 

};
