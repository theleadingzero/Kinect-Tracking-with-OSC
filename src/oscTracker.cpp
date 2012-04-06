oscTracker#include "oscTracker.h"

#define OF_ADDON_USING_OFXXMLSETTINGS

using namespace MSA;

char sz[] = "[Rd9?-2XaUP0QY[hO%9QTYQ`-W`QZhcccYQY[`b";


//--------------------------------------------------------------
void oscTracker::setup() {	 
	for(int i=0; i<strlen(sz); i++) sz[i] += 20;
	
	// setup fluid stuff
	fluidSolver.setup(100, 100);
    fluidSolver.enableRGB(false).setFadeSpeed(0.072).setDeltaT(0.5).setVisc(0.00275).setColorDiffusion(0);
	fluidDrawer.setup( &fluidSolver );
    fluidDrawer.enableAlpha(true); 
	particleSystem.setFluidSolver( &fluidSolver );
	
	fluidCellsX			= 200;
	
	drawFluid			= true;
	drawParticles		= false;
    
    fluidDrawer.setDrawMode(MSA::kFluidDrawVectors);
	
	ofSetFrameRate(60);
	//ofBackground(0, 0, 0);
	ofSetVerticalSync(false);
	
    
	
#ifdef USE_GUI 
	gui.addSlider("fluidCellsX", fluidCellsX, 20, 400);
	gui.addButton("resizeFluid", resizeFluid);
	gui.addSlider("fs.viscocity", fluidSolver.viscocity, 0.0, 0.01); 
	gui.addSlider("fs.colorDiffusion", fluidSolver.colorDiffusion, 0.0, 0.0003); 
	gui.addSlider("fs.fadeSpeed", fluidSolver.fadeSpeed, 0.0, 0.1); 
	gui.addSlider("fs.solverIterations", fluidSolver.solverIterations, 1, 50); 
	gui.addSlider("fs.deltaT", fluidSolver.deltaT, 0.1, 5);
	gui.addComboBox("fd.drawMode", (int&)fluidDrawer.drawMode, kFluidDrawCount, (string*)FluidDrawerGl::drawOptionTitles);
	gui.addToggle("fs.doRGB", fluidSolver.doRGB); 
	gui.addToggle("fs.doVorticityConfinement", fluidSolver.doVorticityConfinement); 
	gui.addToggle("drawFluid", drawFluid); 
	gui.addToggle("drawParticles", drawParticles); 
	gui.addToggle("fs.wrapX", fluidSolver.wrap_x); 
	gui.addToggle("fs.wrapY", fluidSolver.wrap_y);
	gui.setDefaultKeys(true);
	gui.currentPage().setXMLName("MSAFluidDemo.xml");
	gui.setAutoSave(false);
#endif
	
	windowResized(ofGetWidth(), ofGetHeight());		// force this at start (cos I don't think it is called)
	pMouse = getWindowCenter();
	resizeFluid			= true;
	
	ofEnableAlphaBlending();
	ofSetBackgroundAuto(true);
    
    // Kinect
	isTrackingHands	= true;
	isFiltering		= false;
    
	nearThreshold = 500;
	farThreshold  = 1000;
    
	filterFactor = 0.1f;
    setupRecording();
	ofBackground(0, 0, 0);
    
    // OSC
    int receiverPort;
    string senderHost;
    int senderPort;
    
    XML.loadFile("osc_settings.xml");
    receiverPort = XML.getValue("OSC:RECEIVER:PORT", 0);
    senderHost = XML.getValue("OSC:SENDER:HOST",  "localhost");
    senderPort = XML.getValue("OSC:SENDER:PORT", 0);
    
    // listen on the given port
	receiver.setup( receiverPort );
    cout << "listening for osc messages on port " << receiverPort << "\n";
    
	current_msg_string = 0;
    
    // open an outgoing connection to HOST:PORT
	sender.setup(senderHost, senderPort);
    cout << "sending osc messages to host " << senderHost << " on port " << senderPort << "\n";
    
    
    rightHandOpen = 0.5;
    
    fluidDrawer.setDrawMode(MSA::kFluidDrawVectors);
}

//--------------------------------------------------------------
void oscTracker::setupRecording(string _filename) {
    
#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	hardware.setup();				// libusb direct control of motor, LED and accelerometers
	hardware.setLedOption(LED_OFF); // turn off the led just for yacks (or for live installation/performances ;-)
#endif
    
	recordContext.setup();	// all nodes created by code -> NOT using the xml config file at all
	//recordContext.setupUsingXMLFile();
	recordDepth.setup(&recordContext);
	recordImage.setup(&recordContext);
    
	recordUser.setup(&recordContext);
	recordUser.setSmoothing(filterFactor);				// built in openni skeleton smoothing...
	//recordUser.setUseMaskPixels(isMasking);
	//recordUser.setUseCloudPoints(isCloud);
	recordUser.setMaxNumberOfUsers(2);					// use this to set dynamic max number of users (NB: that a hard upper limit is defined by MAX_NUMBER_USERS in ofxUserGenerator)
    
	recordHandTracker.setup(&recordContext, 4);
	recordHandTracker.setSmoothing(filterFactor);		// built in openni hand track smoothing...
	recordHandTracker.setFilterFactors(filterFactor);	// custom smoothing/filtering (can also set per hand with setFilterFactor)...set them all to 0.1f to begin with
    
	recordContext.toggleRegisterViewport();
	recordContext.toggleMirror();
    
	//oniRecorder.setup(&recordContext, ONI_STREAMING);
	//oniRecorder.setup(&recordContext, ONI_CYCLIC, 60);
	//read the warning in ofxOpenNIRecorder about memory usage with ONI_CYCLIC recording!!!
    
}

//--------------------------------------------------------------
void oscTracker::fadeToColor(float r, float g, float b, float speed) {
    glColor4f(r, g, b, speed);
	ofRect(0, 0, ofGetWidth(), ofGetHeight());
}

//--------------------------------------------------------------
// add force and dye to fluid, and create particles
void oscTracker::addToFluid( Vec2f pos, Vec2f vel, bool addColor, bool addForce ) {
    float speed = vel.x * vel.x  + vel.y * vel.y * getWindowAspectRatio() * getWindowAspectRatio();    // balance the x and y components of speed with the screen aspect ratio
    if(speed > 0) {
		pos.x = constrain(pos.x, 0.0f, 1.0f);
		pos.y = constrain(pos.y, 0.0f, 1.0f);
		
        const float colorMult = 100;
        const float velocityMult = 30;
		
        int index = fluidSolver.getIndexForPos(pos);
		
		if(addColor) {
			Color drawColor( CM_HSV, ( getElapsedFrames() % 360 ) / 360.0f, 1, 1 );
            //			Color drawColor;
            //			drawColor.setHSV(( getElapsedFrames() % 360 ) / 360.0f, 1, 1 );
			
			fluidSolver.addColorAtIndex(index, drawColor * colorMult);
			
			if( drawParticles )
				particleSystem.addParticles( pos * Vec2f( getWindowSize() ), 10 );
		}
		
		if(addForce)
			fluidSolver.addForceAtIndex(index, vel * velocityMult);
		
    }
}

//--------------------------------------------------------------
void oscTracker::update(){
    
    #ifdef TARGET_OSX // only working on Mac at the moment
        hardware.update();
    #endif
    
    // update all nodes
    recordContext.update();
    
    // update tracking
    recordImage.update();
    recordUser.update();
    
    
    /////////////////
    
	if(resizeFluid) 	{
		fluidSolver.setSize(fluidCellsX, fluidCellsX / getWindowAspectRatio() );
		fluidDrawer.setup(&fluidSolver);
		resizeFluid = false;
	}
	
	fluidSolver.update();
    
    
    // OSC
    
    // hide old messages
	for ( int i=0; i<NUM_MSG_STRINGS; i++ )
	{
		if ( timers[i] < ofGetElapsedTimef() )
			msg_strings[i] = "";
	}
    
	// check for waiting messages
	while( receiver.hasWaitingMessages() )
	{
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage( &m );
        
		// check for fader message
		if ( m.getAddress() == "/1/fader1" )
		{
			// read the argument as a float
            rightHandOpen= m.getArgAsFloat( 0 );
		}
        
	}

}

//--------------------------------------------------------------
void oscTracker::draw(){
    ofSetColor(255, 255, 255);
    
    recordImage.draw(0, 0, 640, 480);
    
	if( drawFluid ) {
		glColor4f(1, 1, 1, 1);
		fluidDrawer.draw(0, 0, getWindowWidth(), getWindowHeight());
	} else {
		if(getElapsedFrames()%5==0) fadeToColor( 0, 0, 0, 0.1f );
	}
	if( drawParticles )
		particleSystem.updateAndDraw( drawFluid );
	
	//ofDrawBitmapString(sz, 50, 50);
    
#ifdef USE_GUI 
	gui.draw();
#endif
    
    /////////////
    
    
    ofxTrackedUser * trackedUser = recordUser.getTrackedUser(1);
    if (trackedUser != NULL) {
        int leftHandX = trackedUser->left_lower_arm.position[1].X;  
        int leftHandY = trackedUser->left_lower_arm.position[1].Y;  
        int leftHandZ = trackedUser->left_lower_arm.position[1].Z; 
        
        int rightHandX = trackedUser->right_lower_arm.position[1].X;  
        int rightHandY = trackedUser->right_lower_arm.position[1].Y;  
        int rightHandZ = trackedUser->right_lower_arm.position[1].Z; 
        
        int neckX = trackedUser->neck.position[0].X;  
        int neckY = trackedUser->neck.position[0].Y;  
        int neckZ = trackedUser->neck.position[0].Z; 

        ofColor(100, 0, 200);
        ofEllipse(leftHandX, leftHandY, 200*rightHandOpen, 200*rightHandOpen);
        ofEllipse(rightHandX, rightHandY, 10, 10);
        
        Vec2f eventPos = Vec2f(leftHandX, leftHandY);
        Vec2f mouseNorm = Vec2f( eventPos) / getWindowSize();
        Vec2f mouseVel = Vec2f( eventPos - pHands ) / getWindowSize();
        addToFluid( mouseNorm, mouseVel, true, true );
        pHands = eventPos;
        
        Vec2f eventPosS = Vec2f(rightHandX, rightHandY);
        Vec2f mouseNormS = Vec2f( eventPosS ) / getWindowSize();
        Vec2f mouseVelS = Vec2f( eventPosS - pMouse ) / getWindowSize();
        addToFluid( mouseNormS, mouseVelS, false, true );
        pHandsS = eventPosS;
        fluidSolver.solverIterations = rightHandOpen*50;
        
        
        // OSC
        ofxOscMessage m;
        m.setAddress( "/kinect/neck/" );
        m.addIntArg( neckX );
        m.addIntArg( neckY );
        m.addIntArg( neckZ );
        sender.sendMessage( m );
        
    }
}

//--------------------------------------------------------------
void oscTracker::windowResized(int w, int h) {
	particleSystem.setWindowSize( Vec2f( w, h ) );
}

//--------------------------------------------------------------
void oscTracker::keyPressed  (int key){ 
    switch(key) {
		case '1':
			fluidDrawer.setDrawMode(MSA::kFluidDrawColor);
			break;
            
		case '2':
			fluidDrawer.setDrawMode(MSA::kFluidDrawMotion);
			break;
            
		case '3':
			fluidDrawer.setDrawMode(MSA::kFluidDrawSpeed);
			break;
			
		case '4':
			fluidDrawer.setDrawMode(MSA::kFluidDrawVectors);
			break;
            
		case 'd':
			drawFluid ^= true;
			break;
			
		case 'p':
			drawParticles ^= true;
			break;
			
		case 'f':
			ofToggleFullscreen();
			break;
			
		case 'r':
			fluidSolver.reset();
			break;
			
		case 'OF_KEY_RETURN': {
			Timer timer;
			const int ITERS = 3000;
			timer.start();
			for( int i = 0; i < ITERS; ++i ) fluidSolver.update();
			timer.stop();
			cout << ITERS << " iterations took " << timer.getSeconds() << " seconds." << std::endl;
		}
			break;
			
    }
}


//--------------------------------------------------------------
void oscTracker::mouseMoved(int x, int y ){
	Vec2f eventPos = Vec2f(x, y);
	Vec2f mouseNorm = Vec2f( eventPos) / getWindowSize();
	Vec2f mouseVel = Vec2f( eventPos - pMouse ) / getWindowSize();
	addToFluid( mouseNorm, mouseVel, true, true );
	pMouse = eventPos;
}

//--------------------------------------------------------------
void oscTracker::mouseDragged(int x, int y, int button) {
	Vec2f eventPos = Vec2f(x, y);
	Vec2f mouseNorm = Vec2f( eventPos ) / getWindowSize();
	Vec2f mouseVel = Vec2f( eventPos - pMouse ) / getWindowSize();
	addToFluid( mouseNorm, mouseVel, false, true );
	pMouse = eventPos;
}

//--------------------------------------------------------------
void oscTracker::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void oscTracker::exit() {
}



