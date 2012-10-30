//
//  ControllerCursor.cpp
//  emptyExample
//
//  Created by Ben McChesney on 9/24/12.
//
//

#include "ControllerCursor.h"

void ControllerCursor::setup ( ofColor _color , ofPoint _nPosition )
{
    color = _color ;
    nPosition = _nPosition ; 
}

void ControllerCursor::update ( ofPoint _nPosition )
{
    nPosition = _nPosition ; 
    position = ofPoint( _nPosition.x * ofGetWidth() , _nPosition.y * ofGetHeight() )  ;
}

void ControllerCursor::draw( )
{
    ofPushStyle() ;
        ofSetColor( color ) ;
        ofNoFill( ) ;
        ofSetLineWidth( 3 ) ;
        ofCircle( position.x , position.y , 35 ) ;
    ofPopStyle() ;
    
}

//ofPoint nPosition ;
//ofPoint scale ;