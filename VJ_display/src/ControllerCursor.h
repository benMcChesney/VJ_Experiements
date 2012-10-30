//
//  ControllerCursor.h
//  emptyExample
//
//  Created by Ben McChesney on 9/24/12.
//
//

#ifndef emptyExample_ControllerCursor_h
#define emptyExample_ControllerCursor_h


#include "ofMain.h"

class ControllerCursor
{
public:
    ControllerCursor( ) { }
    ~ControllerCursor( ) { }
    
    
    void setup ( ofColor _color , ofPoint _nPosition ) ;
    void update ( ofPoint position ) ;
    void draw( ) ;
    
    ofColor color ;
    ofPoint position ; 
    ofPoint nPosition ;
    ofPoint scale ; 
};
#endif
