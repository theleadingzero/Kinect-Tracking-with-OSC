#pragma once

#include "MSAFluid.h"
#include "MSATimer.h"
#include "ParticleSystem.h"

#include "ofxOpenNI.h"
#include "ofxOsc.h"
#include "ofxXmlSettings.h"

#include "ofMain.h"	

// comment this line out if you don't wanna use the GUI
// you will need ofxSimpleGuiToo, ofxMSAInteractiveObject & ofxXmlSettings
// if you don't use the GUI, you won't be able to see the fluid parameters
//#define USE_GUI		


#ifdef USE_GUI 
#include "ofxSimpleGuiToo.h"
#endif


#define NUM_MSG_STRINGS 20

#define OF_ADDON_USING_OFXXMLSETTINGS

using namespace MSA;

class oscTracker : public ofSimpleApp{
public:
	void setup();
	void update();
	void draw();
    void exit();
	
	void keyPressed  (int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
    
	void windowResized(int w, int h);
    
	void fadeToColor(float r, float g, float b, float speed);
	void addToFluid(Vec2f pos, Vec2f vel, bool addColor, bool addForce);
    
	int					fluidCellsX;
	bool				resizeFluid;
	bool				drawFluid;
	bool				drawParticles;
	
	MSA::FluidSolver	fluidSolver;
	MSA::FluidDrawerGl	fluidDrawer;	
	
	ParticleSystem		particleSystem;
	
	MSA::Vec2f				pMouse;
    MSA::Vec2f				pHands;
    MSA::Vec2f				pHandsS;
    
	
#ifdef USE_GUI 
	ofxSimpleGuiToo	gui;
#endif
    
    //    Kinect
    
    //  bool				isLive, isTracking, isRecording, isCloud, isCPBkgnd, isMasking;
	bool				isTrackingHands, isFiltering;
    
	ofxOpenNIContext	recordContext;
	ofxDepthGenerator	recordDepth;
    
#ifdef USE_IR
	ofxIRGenerator		recordImage;
#else
	ofxImageGenerator	recordImage;
#endif
    
	ofxHandGenerator	recordHandTracker;
    
	ofxUserGenerator	recordUser;
	ofxOpenNIRecorder	oniRecorder;
    
#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	ofxHardwareDriver	hardware;
#endif
    
	void				drawMasks();
	void				drawPointCloud(ofxUserGenerator * user_generator, int userID);
    
	int					nearThreshold, farThreshold;
	int					pointCloudRotationY;
    
	ofImage				allUserMasks, user1Mask, user2Mask, depthRangeMask;
    
	float				filterFactor;
    
    void	setupRecording(string _filename = "");
    
    
    float circleSize;
    float circleGrowth;
    
    
    
    // OSC
    
    ofxOscSender sender;
    void gotMessage(ofMessage msg);
    
    ofxOscReceiver	receiver;
    
    int				current_msg_string;
    string		msg_strings[NUM_MSG_STRINGS];
    float			timers[NUM_MSG_STRINGS];
	string			mouseButtonState;
    
    float       rightHandOpen;
    float       leftHandOpen;
    
    
    ofxXmlSettings XML;
	
};

extern oscTracker *myApp;



