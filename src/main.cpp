
#include "heapApp.h"

heapApp *myApp;
int main( ){
	ofSetupOpenGL(640, 480, OF_WINDOW);			// <-------- setup the GL context
	myApp = new heapApp;
	ofRunApp(myApp);
}
