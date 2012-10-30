#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    
    scaledAmplitude = 0 ;
    amplitude  = 0 ; 
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    minBlobSize = 20 ;
    maxBlobSize = (kinect.width*kinect.height)/2 ;
    pointCloudMinZ = -100 ;
    pointCloudMaxZ = 100 ;
    
    meshHueTimeMultiplier = 0.0f ;
    bToggleTrails = false ;
    bToggleCameraJitter = false ; 
    jitterForceX = 0.0f ;
    jitterForceY = 0.0f ;
    jitterTimeOffsetX = 0.0f ;
    jitterTimeOffsetY = 0.0f ;
    extrudeDepth = 100.0f ;
    extrudeNoiseStrength = 0.0f ;
    jitterCameraDistance = 800.0f ; 
    //Setup ofxUI
    float dim = 24;
	float xInit = OFX_UI_GLOBAL_WIDGET_SPACING;
    float length = 320-xInit;
    
    gui = new ofxUICanvas(0, 0, length+xInit, ofGetHeight());
	gui->addWidgetDown(new ofxUILabel("KINECT PARAMS", OFX_UI_FONT_MEDIUM ));
    gui->addWidgetDown(new ofxUIRangeSlider(length, dim, 0.0, 255.0, farThreshold, nearThreshold, "DEPTH RANGE"));
    gui->addWidgetDown(new ofxUIRangeSlider(length, dim, 0.0, ((kinect.width * kinect.height ) / 2 ), minBlobSize , maxBlobSize, "OPENCV BLOB SIZE"));
    gui->addWidgetDown(new ofxUIRangeSlider(length, dim, 0 , 10000 , pointCloudMinZ , pointCloudMaxZ, "POINT CLOUD Z RANGE")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 255.0f , meshHueTimeMultiplier , "MESH TIME HUE MULTIPLIER")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 255.0f , fboFadeAmount , "FBO FADE AMOUNT")) ;
    gui->addWidgetDown(new ofxUIToggle(dim, dim, bToggleTrails , "TOGGLE TRAILS")) ;
    gui->addWidgetRight(new ofxUIToggle(dim, dim, bToggleCameraJitter , "TOGGLE CAMERA JITTER")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 2500.0f , jitterForceX , "X JITTER")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 2500.0f , jitterForceY , "Y JITTER")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 6.0f , jitterTimeOffsetX , "JITTER TIME X")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 6.0f , jitterTimeOffsetY , "JITTER TIME Y")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 10.0f , extrudeDepth , "EXTRUDE DEPTH")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 0.0f , 500.0f , extrudeNoiseStrength , "EXTRUDE NOISE STRENGTH")) ;
    gui->addWidgetDown(new ofxUISlider(length, dim, 400.0f , 2000.0f , jitterCameraDistance , "JITTER CAMERA DISTANCE")) ;
        
    ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);
    gui->loadSettings("GUI/kinectSettings.xml") ;
    gui->disable() ;

    //Syphon
    mainOutputSyphonServer.setName("Screen Output");
	individualTextureSyphonServer.setName("Texture Output");
    
	mClient.setup();
    
    mClient.setApplicationName("Simple Server");
    mClient.setServerName("");
    
    trailFbo.allocate( ofGetWidth() , ofGetHeight() , GL_RGBA ) ;
    

    currentFboIndex = 0 ;
    for ( int i = 0 ; i < 10 ; i++ )
    {
        pointCloudFbos.push_back( ofFbo() ) ;
        pointCloudFbos[i].allocate( ofGetWidth() , ofGetHeight() , GL_RGBA ) ;
        pointCloudFbos[i].begin() ;
            ofClear( 1 , 1 , 1 , 0 ) ; 
        pointCloudFbos[i].end() ;
    }

    bGuiEnabled = false ;
    
    // listen on the given port
	cout << "listening for osc messages on port " << PORT << "\n";
	receiver.setup(PORT);
    
	current_msg_string = 0;
    
    cameraOrigin = easyCam.getPosition() ;
    
    
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground( 0 , 0, 0 );
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
        if ( bDrawPointCloud == false )
        {
            // we do two thresholds - one for the far plane and one for the near plane
            // we then do a cvAnd to get the pixels which are a union of the two thresholds
            if(bThreshWithOpenCV) {
                grayThreshNear = grayImage;
                grayThreshFar = grayImage;
                grayThreshNear.threshold(nearThreshold, true);
                grayThreshFar.threshold(farThreshold);
                cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
            } else {
                
                // or we do it ourselves - show people how they can work with the pixels
                unsigned char * pix = grayImage.getPixels();
                
                int numPixels = grayImage.getWidth() * grayImage.getHeight();
                for(int i = 0; i < numPixels; i++) {
                    if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                        pix[i] = 255;
                    } else {
                        pix[i] = 0;
                    }
                }
            }
            
            // update the cv images
            grayImage.flagImageChanged();
            
            // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
            // also, find holes is set to true so we will get interior contours as well....
            // findContours( ofxCvGrayscaleImage&  input, int minArea, int maxArea, int nConsidered, bool bFindHoles, bool bUseApproximation ) ;
            
            contourFinder.findContours(grayImage, minBlobSize , maxBlobSize , 20, false);
        }
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
    
    //currentFboIndex++ ;
    //if ( currentFboIndex >= ( pointCloudFbos.size() - 1 ) )
    //{
    //    currentFboIndex = 0 ;
    //}
    
    // check for waiting messages
	while(receiver.hasWaitingMessages())
    {
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(&m);
        if(m.getAddress() == "/sumAmplitude" )
        {
            amplitude = m.getArgAsFloat(0) ;
            scaledAmplitude = m.getArgAsFloat(1) ;
        }
    }
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
        
       // if ( bToggleTrails )
            trailFbo.begin () ;
        
        if ( bToggleTrails )
        {
            //ofSetColor( 255 , 255 , 255 , fboFadeAmount ) ;
           // trailFbo.draw( 0 , 0 ) ;
            ofSetColor( 0 , 0 , 0, fboFadeAmount ) ;
            ofRect( 0 , 0, ofGetWidth() , ofGetHeight() ) ;
        }
        else
        {
            ofClear( 1 , 1 , 1 , 0 ) ; 
        }

        ofSetColor( 255 , 255 ,255 ) ;
        
        if ( bToggleCameraJitter == true )
        {
            ofVec3f origin = cameraOrigin ;
            origin.x = ofSignedNoise( ofGetElapsedTimef() * jitterTimeOffsetX ) * jitterForceX ;
            origin.y = ofSignedNoise( ofGetElapsedTimef() * jitterTimeOffsetY ) * jitterForceY ;
            origin.z = jitterCameraDistance + amplitude * 5.0f ;
//            easyCam.setPosition( origin ) ;
            easyCam.setGlobalPosition( origin.x , origin.y , origin.z ) ;
           // easyCam.lookAt( ofVec3f( 0 , 0 , 0 ) ) ;
//            easyCam.setDistance(600.0f + scaledAmplitude/2.0f ) ;
           // ofPushMatrix( ) ;
           // ofRotateX(origin.x) ;
           // ofRotateY(origin.y) ;
        }
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
        
        if ( bToggleCameraJitter == true )
        {
        //    ofPopMatrix() ;
        }
        /*
        
        for ( int i = 0 ; i < pointCloudFbos.size() ; i++)
        {
            ofPushMatrix() ;
                ofTranslate( 0 , 0 , 300 * i ) ;
                ofSetColor( 255 , 255 , 255 ) ;
                cout << " i : " << i << endl ; 
                pointCloudFbos[ i ].draw( 0 , 0 ) ;
            ofPopMatrix() ;
        }*/
        
        trailFbo.end() ;
        ofSetColor( 255 , 255 , 255 ) ;
        ofPushMatrix( ) ;
        ofTranslate( 0 , ofGetHeight() ) ;
        ofScale( 1 , -1 , 1 ) ;
        trailFbo.draw(0 , 0 ) ;
        ofPopMatrix( ) ;
        mainOutputSyphonServer.publishScreen();

	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
		
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
    /*
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,652); */
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS );
	int step = 2;
    //Get out hue offset
    int timeHue = ofGetElapsedTimef() * meshHueTimeMultiplier ;
    timeHue %= 255 ;
    
    ofColor hueOffset = ofColor::fromHsb( timeHue , 255.0f , 255.0f ) ;
	
    
   // pointCloudFbos[ currentFboIndex ].begin() ;
    //    ofClear( 1 , 1 , 1 , 0 ) ;
  //  pointCloudFbos[ currentFboIndex ].end() ;
    
    float _time = ofGetElapsedTimef()* 0.75f ;
    for(int y = 0; y < h; y += step)
    {
		for(int x = 0; x < w; x += step)
        {
            float noiseStep = ofSignedNoise( _time + x ) * extrudeNoiseStrength * amplitude * extrudeDepth ; // * extrudeNoiseStrength + amplitude * extrudeDepth ; // * 5.0f ;
			if(kinect.getDistanceAt(x, y) > 0)
            {
                ofVec3f vertex = kinect.getWorldCoordinateAt(x, y) ;
                if ( vertex.z > pointCloudMinZ && vertex.z < pointCloudMaxZ )
                {
                    float zOffset = noiseStep ;
                    mesh.addVertex( vertex ) ;
                    vertex.z += zOffset ;
                    mesh.addVertex( vertex );

                    ofColor col = kinect.getColorAt(x,y) + hueOffset ;
                    mesh.addColor( col );
                    mesh.addColor( col );
                }
				
			}
		}
	}
	glPointSize(3);
    //for ( int i = 0 ; i < pointCloudFbos.size() ; i++ )
    //{
        ofPushMatrix();
            // the projected points are 'upside down' and 'backwards'
            ofScale(1, -1, -1);
            ofTranslate(0, 0, -1000 ); // center the points a bit
            glEnable(GL_DEPTH_TEST);
             //   pointCloudFbos[ currentFboIndex ].begin() ;
                    mesh.draw( );
            //    pointCloudFbos[ currentFboIndex ].end() ;
            glDisable(GL_DEPTH_TEST);
        ofPopMatrix();
    //}
	
}


//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.widget->getName();
	int kind = e.widget->getKind();
	
	if(name == "DEPTH RANGE")
	{
		ofxUIRangeSlider *slider = (ofxUIRangeSlider *) e.widget;
        farThreshold = slider->getScaledValueLow() ;
        nearThreshold = slider->getScaledValueHigh() ; 
	}
    
    if(name == "OPENCV BLOB SIZE")
	{
		ofxUIRangeSlider *slider = (ofxUIRangeSlider *) e.widget;
        minBlobSize = slider->getScaledValueLow() ;
        maxBlobSize = slider->getScaledValueHigh() ;
	}
    
    if(name == "POINT CLOUD Z RANGE" )
	{
		ofxUIRangeSlider *slider = (ofxUIRangeSlider *) e.widget;
        pointCloudMinZ = slider->getScaledValueLow() ;
        pointCloudMaxZ = slider->getScaledValueHigh() ;
	}
    
    if(name == "MESH TIME HUE MULTIPLIER" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        meshHueTimeMultiplier = slider->getScaledValue() ;
	}
    
    if(name == "FBO FADE AMOUNT" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        fboFadeAmount = slider->getScaledValue() ;
	}
    
    if ( name == "TOGGLE TRAILS")
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        bToggleTrails = toggle->getValue() ; 
    }
    
    if ( name == "TOGGLE CAMERA JITTER")
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        bToggleCameraJitter = toggle->getValue() ;
    }
    
    if (name == "X JITTER" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        jitterForceX = slider->getScaledValue() ;
	}
    
    if (name == "Y JITTER" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        jitterForceY = slider->getScaledValue() ;
	}
    
    if (name ==  "JITTER TIME X" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        jitterTimeOffsetX = slider->getScaledValue() ;
	}
    
    if (name == "JITTER TIME Y" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        jitterTimeOffsetY = slider->getScaledValue() ;
	}
    
    if (name == "EXTRUDE DEPTH" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        extrudeDepth = slider->getScaledValue() ;
	}
    
    if (name == "EXTRUDE NOISE STRENGTH" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        extrudeNoiseStrength = slider->getScaledValue() ;
	}
    
    if (name == "JITTER CAMERA DISTANCE" )
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
        jitterCameraDistance = slider->getScaledValue() ;
	}
    
    // gui->addWidgetDown(new ofxUISlider(length, dim, 400.0f , 2000.0f , jitterCameraDistance , "JITTER CAMERA DISTANCE")) ;
    gui->saveSettings("GUI/kinectSettings.xml") ; 
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
            
        case 'g':
        case 'G':
            bGuiEnabled = !bGuiEnabled ;
            if ( bGuiEnabled == true )
                gui->enable() ;
            else
                gui->disable() ;
            break ;
        
        case 'P':
        case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
            /*
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			*/
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
