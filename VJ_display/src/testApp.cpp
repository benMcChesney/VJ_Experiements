#include "testApp.h"

//#include "ofxSimpleGuiToo.h"


void testApp::setup()
{
    Tweenzor::init( ) ; 
    ofSetVerticalSync(false);

	plotHeight = 45;
	bufferSize = 512;

	fft = ofxFft::create(bufferSize, OF_FFT_WINDOW_HAMMING);
	// To use FFTW, try:
	// fft = ofxFft::create(bufferSize, OF_FFT_WINDOW_HAMMING, OF_FFT_FFTW);
	spectrogram.allocate(bufferSize, fft->getBinSize(), OF_IMAGE_GRAYSCALE);
	memset(spectrogram.getPixels(), 0, (int) (spectrogram.getWidth() * spectrogram.getHeight()) );
	spectrogramOffset = 0;

	audioInput = new float[bufferSize];
	memset(audioInput, 0, sizeof(float) * bufferSize);

	// 0 output channels,
	// 1 input channel
	// 44100 samples per second
	// [bins] samples per buffer
	// 4 num buffers (latency)

	ofSoundStreamSetup(0, 1, this, 44100, bufferSize, 4);

	mode = MIC ;
	appWidth = ofGetWidth();
	appHeight = ofGetHeight();

	ofBackground(0, 0, 0);

    //get number of bins
    const int len = fft->getBinSize() ;

    //Controls
    
    //set a default FftRange
    FftRange r = FftRange( 0, 256, 256 );

   // for ( int i = 0 ; i < 4 ; i++ )
   // {
        //Generate a random color if none is found in XML
        ofColor color ;
        color.setHex( ofRandom( 0x111111 , 0xFFFFFFF ) ) ;
        //visuals.push_back( RadialFft( 50.0f, 300.0f, 6.0f, ofVec2f( ofGetWidth() /2 , ofGetHeight()/2 ) , r ,color ) ) ;
        radialFft = RadialFft( 50.0f, 300.0f, 6.0f, ofVec2f( ofGetWidth() /2 , ofGetHeight()/2 ) , r ,color ) ; 
   // }

    /*
     This uses Reza Alli's ofxUI 
     I really like how easy it is to use and that it automatically saves all the values to an XML file
     Below we create a GUI canvas for each of our visulizers so we can tweak parameters
    */
    
    
    //for ( int i = 0 ; i < visuals.size() ; i++ )
   // {

        float dim = 24;
        float xInit = OFX_UI_GLOBAL_WIDGET_SPACING;
        float length = 250-xInit;
    
        fftGUI = new ofxUICanvas(0, 0, ofGetWidth(), ofGetHeight());
        
        fftGUI->addWidgetDown(new ofxUILabel("FFT PARAMS", OFX_UI_FONT_LARGE));
        //fftGUI->addRangeSlider("RED", 0.0, 1 , len -1 , 0 , len , length-xInit,dim);
    
        ////gui.addSlider ( string title , float targetFloat , float minValue, float maxValue ) ;
        //    fftGUI->addWidgetDown(new ofxUIRangeSlider(length-xInit,dim, 0.0, len-2 , 0 , len-1 , "MAX RANGE"));
        fftGUI->addWidgetRight(new ofxUIRangeSlider(length-xInit,dim, 0.0, len-2 , 0 , len-1 , "FFT RANGE"));
        fftGUI->addWidgetRight(new ofxUISlider(length-xInit,dim , 0.0f , 450.0f , radialFft.globalCenterOffset , "CENTER OFFSET" ) ) ;
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 1.0f , 1200.0f , radialFft.globalExtrusion , "GLOBAL EXTRUSION" ) ) ;
        fftGUI->addWidgetRight(new ofxUISlider(length-xInit,dim , 20.0f , 300.0f , radialFft.range.maxExtrusion , "MAX EXTRUSION" ) ) ;
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 1.0f , 40.0f , radialFft.globalBarWidth , "BAR WIDTH" ) ) ;
    
        fftGUI->addWidgetDown(new ofxUI2DPad(length-xInit,120, ofPoint( 0 , ofGetWidth() ) , ofPoint ( 0 , ofGetHeight() )  , radialFft.position  , "X + Y"));
   
        fftGUI->addWidgetRight(new ofxUIToggle( 60 , 50 , radialFft.doFill , "DRAW FILL" ) ) ;
        fftGUI->addWidgetDown(new ofxUIToggle( 60 , 50 , radialFft.drawBothSides , "DRAW BOTH SIDES" ) ) ;
    
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.1f , 15.0f , radialFft.noiseTimeMultiplier , "NOISE TIME MULTIPLIER" ) ) ;
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 400.0f , radialFft.noiseStrength , "NOISE STRENGTH" ) ) ;
        fftGUI->addWidgetRight(new ofxUISlider(length-xInit,dim , 0.0f , 4.0f , radialFft.noiseIndexMultiplier , "NOISE INDEX MULTIPLIER" ) ) ;
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 40.0f , radialFft.amplitudeMultiplier , "AMPLITUDE MULTIPLIER" ) ) ;
    
    
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 360.0f , radialFft.hue , "HUE" ) ) ;
        fftGUI->addWidgetRight(new ofxUISlider(length-xInit,dim , 0.0f , 360.0f , radialFft.saturation, "SATURATION" ) ) ;
        fftGUI->addWidgetRight(new ofxUISlider(length-xInit,dim , 0.0f , 360.0f , radialFft.brightness , "BRIGHTNESS" ) ) ;
    
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 1000.0f , radialFft.hueFftMultiplier , "HUE FFT MULTIPLIER" ) ) ;
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 1000.0f , radialFft.amplitudeDiffMultiplier , "AMPLITUDE DIFF MULTIPLIER" ) ) ;
    
        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 255.0f , clearAlpha , "CLEAR ALPHA" ) ) ;
        fftGUI->addWidgetRight(new ofxUISlider(length-xInit,dim , 0.0f , 1000.0f , amplitudeScale , "GLOBAL AMPLITUDE SCALE" ) ) ;

        fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 0.5f , radialFft.interpolateTime , "INTERPOLATE TIME" ) ) ;
    
   // float interpolatedMeanAmplitude ;
    //float interpolateTime ;
    
    /*
    
     if ( name == "GLOBAL AMPLITUDE SCALE")
     {
     ofxUISlider *slider = (ofxUISlider *) e.widget;
     ampltiudeScale = slider->getScaledValue() ;
     }
     

     
     if ( name == "GLOBAL AMPLITUDE SCALE")
     {
     ofxUISlider *slider = (ofxUISlider *) e.widget;
     ampltiudeScale = slider->getScaledValue() ;
     }
     
     */

        ofAddListener(fftGUI->newGUIEvent,this,&testApp::fftGUIEvent);
        fftGUI->loadSettings( "GUI/FFT_0.xml") ;
    

    //we load a font and tell OF to make outlines so we can draw it as GL shapes rather than textures
    shader.load("shaders/fragFun.vert", "shaders/tunnel.frag");
    
    image1.loadImage( "shaders/text2.jpg" ) ;
    fbo1.allocate( image1.width , image1.height ) ;
    
    ofSetFrameRate( 60 ) ;
    ofSetCircleResolution( 180 ) ;
    
    screenFbo.allocate(ofGetWidth(), ofGetHeight() , GL_RGBA ) ;
    screenFbo.begin() ;
    ofClear( 1 , 1 , 1 , 0 ) ;
    screenFbo.end() ;
    
    
    //we load a font and tell OF to make outlines so we can draw it as GL shapes rather than textures
    shader.load("shaders/fragFun.vert", "shaders/tunnel.frag");
    image1.loadImage( "shaders/tex1.jpg" ) ;
    fbo1.allocate( image1.width , image1.height ) ;
    
    ofSetVerticalSync( true ) ;
    
    //setup OSC
    cout << "listening for osc messages on port " << PORT << "\n";
	receiver.setup(PORT);
    
	current_msg_string = 0;

    cursor0.setup( ofColor( 0 , 255 , 0 ) , ofPoint ( 0 , 0 ) ) ;
    cursor1.setup( ofColor( 0 , 0 , 255 ) , ofPoint ( 0 , 0 ) ) ;

    
    
}

void testApp::fftGUIEvent(ofxUIEventArgs &e)
{
    string name = e.widget->getName();
	int kind = e.widget->getKind();
    
    if ( name == "FFT RANGE" )
    {
        ofxUIRangeSlider *rslider = (ofxUIRangeSlider *) e.widget; 
        radialFft.range.startIndex = rslider->getScaledValueLow() ;
        radialFft.range.endIndex = rslider->getScaledValueHigh() ;
    }
    
    if ( name == "CENTER OFFSET" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.globalCenterOffset = slider->getScaledValue() ; 
    }
    
    if ( name == "GLOBAL EXTRUSION" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.globalExtrusion = slider->getScaledValue() ;
    }
    
    if ( name == "MAX EXTRUSION" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.range.maxExtrusion = slider->getScaledValue() ;
    }
    
    if ( name == "BAR WIDTH"  )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.globalBarWidth = slider->getScaledValue() ;
    }
    
    if ( name == "NOISE TIME MULTIPLIER"  )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.noiseTimeMultiplier = slider->getScaledValue() ;
    }
    
    if ( name == "NOISE STRENGTH" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.noiseStrength = slider->getScaledValue() ;
    }
    
    if ( name == "NOISE INDEX MULTIPLIER"  )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.noiseIndexMultiplier = slider->getScaledValue() ;
    }
    
    if ( name == "CENTER OFFSET" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.globalCenterOffset = slider->getScaledValue() ;
    }
    
    if ( name == "DRAW FILL" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        radialFft.doFill = toggle->getValue() ; 
    }

    
    if ( name == "DRAW BOTH SIDES" )
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        radialFft.drawBothSides = toggle->getValue() ;
    }
    
    if( name == "AMPLITUDE MULTIPLIER" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.amplitudeMultiplier = slider->getScaledValue() ; 
    }
    
    if(name == "X + Y")
    {
        ofxUI2DPad *pad = (ofxUI2DPad *) e.widget;
        radialFft.position = pad->getScaledValue() ;
    }
    
    if( name == "HUE" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.hue = slider->getScaledValue() ;
    }
    
    if( name == "SATURATION" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.saturation = slider->getScaledValue() ;
    }
    
    if( name == "BRIGHTNESS" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.brightness = slider->getScaledValue() ;
    }
    
    if( name == "HUE FFT MULTIPLIER" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.hueFftMultiplier = slider->getScaledValue() ;
        //radialFft.updateColor() ;
    }
    
    if ( name == "AMPLITUDE DIFF MULTIPLIER" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radialFft.amplitudeDiffMultiplier = slider->getScaledValue() ; 
    }
    
    if ( name == "CLEAR ALPHA" )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        clearAlpha = slider->getScaledValue() ; 
    }
    
    if ( name == "GLOBAL AMPLITUDE SCALE")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        amplitudeScale = slider->getScaledValue() ; 
    }
    
    
    if ( name == "INTERPOLATE TIME"  )
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
         radialFft.interpolateTime = slider->getScaledValue() ;
    }
    
    /*
     fftGUI->addWidgetDown(new ofxUISlider(length-xInit,dim , 0.0f , 0.5f , radialFft.interpolateTime , "INTERPOLATE TIME" ) ) ;
    
     */

    fftGUI->saveSettings("GUI/FFT_0.xml" ) ;
}


void testApp::update()
{
    
    Tweenzor::update( ofGetElapsedTimeMillis() ) ; 
    //Get amplitudes and pass them on to visuals
    float * amplitudes = fft->getAmplitude() ;

    ////if ( ofGetElapsedTimef() > 30 )
    radialFft.update( amplitudes ) ;
    //for ( int i = 0 ; i < visuals.size() ; i++ )
    //{
    //    visuals[i].update( amplitudes ) ;
    //}
    
    //Update OSC
    updateOSC( ) ;
}

void testApp::updateOSC( )
{
    // hide old messages
	for(int i = 0; i < NUM_MSG_STRINGS; i++){
		if(timers[i] < ofGetElapsedTimef()){
			msg_strings[i] = "";
		}
	}
    
	// check for waiting messages
	while(receiver.hasWaitingMessages()){
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(&m);
        
        if ( m.getAddress() == "/numCursors/" )
        {
            int numCursors = m.getArgAsInt32(0) ;
           // if ( numCursors > 0 )
           //     cout << "num cursors! " << numCursors << endl ;
        }
        //Cursor #
        // m.setAddress("/numCursors/") ;
//        m.addIntArg(activeHands) ;
        // m.setAddress("/cursor/"+ ofToString( trackedHandOrder ) + "/position");
        
		// check for mouse moved message
		else if(m.getAddress() == "/cursor/0/position"){
			// both the arguments are int32's
            ofPoint p = ofPoint ( m.getArgAsFloat( 0 ) , m.getArgAsFloat( 1 ) ) ; 
           // cout << "position 0 " << p.x <<  " , " << p.y << endl ;
            cursor0.update( p ) ;
		}
           
        else if(m.getAddress() == "/cursor/1/position"){
            
            
            ofPoint p = ofPoint ( m.getArgAsFloat( 0 ) , m.getArgAsFloat( 1 ) ) ;
           // cout << "position 1 " << p.x <<  " , " << p.y << endl ;
            
            cursor1.update( p ) ;
            
			// both the arguments are int32's
			//cursorX = m.getArgAsFloat( 0 ) ;
			//cursorY = m.getArgAsFloat( 1 );
		}
		// check for mouse button message
	
		else{
			// unrecognized message: display on the bottom of the screen
			string msg_string;
			msg_string = m.getAddress();
			msg_string += ": ";
			for(int i = 0; i < m.getNumArgs(); i++){
				// get the argument type
				msg_string += m.getArgTypeName(i);
				msg_string += ":";
				// display the argument - make sure we get the right type
				if(m.getArgType(i) == OFXOSC_TYPE_INT32){
					msg_string += ofToString(m.getArgAsInt32(i));
				}
				else if(m.getArgType(i) == OFXOSC_TYPE_FLOAT){
					msg_string += ofToString(m.getArgAsFloat(i));
				}
				else if(m.getArgType(i) == OFXOSC_TYPE_STRING){
					msg_string += m.getArgAsString(i);
				}
				else{
					msg_string += "unknown";
				}
			}
			// add to the list of strings to display
			msg_strings[current_msg_string] = msg_string;
			timers[current_msg_string] = ofGetElapsedTimef() + 5.0f;
			current_msg_string = (current_msg_string + 1) % NUM_MSG_STRINGS;
			// clear the next line
			msg_strings[current_msg_string] = "";
		}
        
	}

}


void testApp::draw()
{
   //  if ( ofGetElapsedTimef() < 30 )
   //      return ;
    
   
       
     
	
    
	fbo1.begin( ) ;
    image1.draw ( 0 , 0 ) ;
    fbo1.end( ) ;
    
    ofTexture tex1 = image1.getTextureReference();
    
  //  ofSetFrameRate( 60 ) ;
   // ofSetVerticalSync( true ) ;
    
	ofSetColor(245, 58, 135);
	ofFill();
	
	//if( doShader ){
		shader.begin();
        //we want to pass in some varrying values to animate our type / color
        //shader.setUniform1f("timeValX", ofGetElapsedTimef() * 0.1 );
        //shader.setUniform1f("timeValY", -ofGetElapsedTimef() * 0.18 );
        
        //we also pass in the mouse position
        //we have to transform the coords to what the shader is expecting which is 0,0 in the center and y axis flipped.
        
        // so here is the trick, important one:
        // tex1 is the texture from fbo1.
        // we assign this to texture 0 , i.e. "textBase" in the shader.
        // this is as drawing into the fbo  image1. (from shader point of view)
        // so instead of making :
        // fbo2.draw(0,0) after shader parameters we make this line:
        shader.setUniformTexture("texBase",   tex1, 0 ); //look that is number 0: and
        // textures 0 are the ones used do "draw".
        // so we could do this making fbo1.draw.
        
        
        
        ofMap( mouseX , 0.0f , ofGetWidth() , 0.0f , 1.0f ) ;
        //shader.setUniform2f("mouse", ofMap( mouseX , 0.0f , ofGetWidth() , 0.0f , 1.0f ) ,
        //                             ofMap( mouseY , 0.0f , ofGetHeight() , 0.0f , 1.0f ) );
        shader.setUniform1f("time", ofGetElapsedTimef() );
        shader.setUniform2f("resolution", ofGetWidth() , ofGetHeight() ) ;
    
            
        float noiseDiff = ofNoise( ofGetElapsedTimef() ) * radialFft.noiseTimeMultiplier * ( cursor1.nPosition.y * radialFft.noiseStrength ) ;
        float diff = radialFft.amplitudeDiffMultiplier * cursor1.nPosition.x * radialFft.interpolatedMeanAmplitude  ;
        
        shader.setUniform1f("soundOffset" , diff + noiseDiff ) ;
    
        ofColor hueColor = radialFft.color ;
        ofColor hueAdd = ofColor::fromHsb( cursor0.nPosition.x * 255.0f , radialFft.color.getSaturation() + sin ( ofGetElapsedTimef() ) * 50.0f , cursor0.nPosition.y * 255.0f) ;
        hueColor = hueAdd ; 
        shader.setUniform1f( "red" , hueColor.r / 255.0f ) ;
        shader.setUniform1f( "green" , hueColor.g / 255.0f ) ;
        shader.setUniform1f( "blue" , hueColor.b / 255.0f ) ;

        cursor0.draw( ) ;
        cursor1.draw( ) ;
        
//	}
	
    ofSetColor( 255 , 255 , 255 ) ;
    ofRect( 0 , 0 , ofGetWidth() , ofGetHeight() ) ;
	
	///if( doShader ){
		shader.end();
	//}

    
    
    //return ;

    
    ofEnableAlphaBlending() ;
   // screenFbo.begin() ;
   // ofSetColor( 0 ,  0, 0 , clearAlpha ) ;
   // ofRect( 0 , 0, ofGetWidth() , ofGetHeight() ) ;
     ofSetColor ( 255.0f , 255.0f , 255.0f ) ;

    ofEnableSmoothing() ;

   // for ( int i = 0 ; i < visuals.size() ; i++ )
   // {
   //     visuals[i].draw( ) ;
   // }

   // screenFbo.end() ;
    
    //ofSetColor( 255 , 255  ,255 , clearAlpha ) ;
    //screenFbo.draw( 0 , 0 ) ;
   
            
    if ( bGuiEnabled == true )
    {
        ofSetColor ( 255 , 255 , 255 )  ;
        
        radialFft.draw( ) ; 
        
        //gui.draw() ;
        ofPushMatrix() ;
        ofTranslate( 15 , ofGetHeight() +- 75 ) ;
        ofDrawBitmapString( "G : Toggle GUI " , 0 , 0 ) ;
        ofTranslate( 0 , 15 ) ;
        ofDrawBitmapString( "A , M , N , S change the FFT Input " , 0 , 0 ) ;
        string curInput ;
        if (mode == MIC) {	curInput = "Microphone" ; }
        else if (mode == NOISE) { curInput = "Noise" ; }
        else if (mode == SINE) { curInput = "Mouse" ;}
        else if ( mode == ANIMATE ) {  curInput = "Time" ; }
        ofTranslate( 0 , 15 ) ;
        ofDrawBitmapString( "Current Input is: " + curInput , 0 , 0 ) ;
        ofPopMatrix() ;
        
        int graphYOffset = 10 ;
        ofSetHexColor(0xffffff);
        ofPushMatrix();
        ofTranslate( ( ofGetWidth() +- bufferSize )/2 , ofGetHeight() + plotHeight * -3 + -3 * graphYOffset );
        ofDrawBitmapString("Time Domain", 0, 0);
        //Draw Range
        const float binLength = (float)(fft->getBinSize()) ;
        ofEnableAlphaBlending() ;
        ofSetColor ( 255 , 0 , 0 , 125 ) ;
        ofFill() ;
        
        //for int i = 0 , for each range
        //float alpha = 255.0f / (float)visuals.size()  ;
        float ySpacing = plotHeight  ; /// (float)visuals.size() ;
        //for ( int i = 0 ; i < visuals.size() ; i++ )
        //{
        ofSetColor ( radialFft.color , 255 ) ;
        ofRect ( (float)radialFft.range.startIndex/binLength * bufferSize , ySpacing * 0 , (float)radialFft.range.numIndicies/binLength * bufferSize , ySpacing ) ;
        // }
        
        //void testApp::plot(float* array, int length, float scale, float offset) {
        plot(audioInput, bufferSize, plotHeight , 0);
        
       
        ofTranslate(0, plotHeight + graphYOffset );
        ofDrawBitmapString("Frequency Domain", 0, 0);
        ofSetColor ( 255 , 255 , 255 ) ;
        ofFill() ;
        fft->draw(0, 0, fft->getSignalSize(), plotHeight);
        ofTranslate( 0 ,  plotHeight + graphYOffset );
        spectrogram.update();
        spectrogram.draw(0, 0, bufferSize , plotHeight );
        
       
        ofPopMatrix();
        /*
        string msg = ofToString((int) ofGetFrameRate()) + " fps";
        
        ofSetColor( 0 , 0 , 0 ) ;
        ofRect( 0 , 0 , ofGetWidth() , 250 ) ;
        ofSetColor( 255 , 255 , 255 ) ;
        string buf;
        buf = "listening for osc messages on port" + ofToString(PORT);
        ofDrawBitmapString(buf, 10, 20);
        
        for(int i = 0; i < NUM_MSG_STRINGS; i++){
            ofDrawBitmapString(msg_strings[i], 10, 40 + 15 * i);
        }*/
    }

    
    

}


void testApp::plot(float* array, int length, float scale, float offset) {
	ofNoFill();
	ofRect(0, 0, length, plotHeight);
	glPushMatrix();
	glTranslatef(0, plotHeight / 2 + offset, 0);
	ofBeginShape();
	for (int i = 0; i < length; i++)
		ofVertex(i, array[i] * scale);
	ofEndShape();
	glPopMatrix();
}

void testApp::audioReceived(float* input, int bufferSize, int nChannels) {
	if (mode == MIC) {
		// store input in audioInput buffer
		memcpy(audioInput, input, sizeof(float) * bufferSize);
	} else if (mode == NOISE) {
		for (int i = 0; i < bufferSize; i++)
			audioInput[i] = ofRandom(-1, 1);
	} else if (mode == SINE) {
		for (int i = 0; i < bufferSize; i++)
			audioInput[i] = sinf(PI * i * mouseX / appWidth);
	} else if ( mode == ANIMATE ) {
        for (int i = 0; i < bufferSize; i++)
			audioInput[i] = sinf(PI * i * ( sin( ofGetElapsedTimef() * .5f ) * appWidth )    / appWidth);
    }

	fft->setSignal(audioInput);

	float* curFft = fft->getAmplitude();
	int spectrogramWidth = (int) spectrogram.getWidth();
	unsigned char* pixels = spectrogram.getPixels();
	for(int i = 0; i < fft->getBinSize(); i++)
		pixels[i * spectrogramWidth + spectrogramOffset] = (unsigned char) (255. * curFft[i]);
	spectrogramOffset = (spectrogramOffset + 1) % spectrogramWidth;
}

void testApp::keyPressed(int key) {

    switch (key)
    {
        case 'p':
        {
          //  drawPadding = !drawPadding;
           // gui->setDrawWidgetPadding(drawPadding);
        }
            break;
            
        case 'g':
        {
            //bGuiEnabled = false ;
            
            fftGUI->toggleVisible();
            bGuiEnabled = fftGUI->isVisible() ;
           
        }
            break;
            
            //FFT Mode
        case 'm': mode = MIC; break;
        case 'n': mode = NOISE; break;
        case 's': mode = SINE; break;
        case 'a': mode = ANIMATE ; break ;
            
        default:
            break;
    }
    /*
    if(key>='0' && key<='9')
    {
		gui.setPage(key - '0');
		gui.show();
	}
	else
	{
        switch (key)
        {
            //switching pages
            case ' ': gui.toggleDraw(); break;
            case '[': gui.prevPage(); break;
            case ']': gui.nextPage(); break;
            case 'p': gui.nextPageWithBlank(); break;

            //FFT Mode
            case 'm': mode = MIC; break;
            case 'n': mode = NOISE; break;
            case 's': mode = SINE; break;
            case 'a': mode = ANIMATE ; break ;

            case 'g' :
            case 'G' : gui.toggleDraw() ; break ;
        }
    */
}
