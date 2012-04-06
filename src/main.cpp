
#include "oscTracker.h"

oscTracker *myApp;
int main( ){
	ofSetupOpenGL(640, 480, OF_WINDOW);			// <-------- setup the GL context
	myApp = new oscTracker;
	ofRunApp(myApp);
}
