#include "ofApp.h"
#define LENGTH 0.5      // chassis length
#define WIDTH 0.4       // chassis width
#define HEIGHT 0.1      // chassis height
#define RADIUS 0.10     // wheel radius
#define STARTZ 0.5      // starting height of chassis
#define CMASS 1         // chassis mass
#define WMASS 0.2       // wheel mass

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };
static const dReal *p;

//--------------------------------------------------------------
void ofApp::setup(){
    int i;
    dMass m;
    speed=0; steer=0;

    // loads sound
    mySound.load("race.mp3");
    mySound.setLoop(true);
    mySound.play();

    ofEnableDepthTest();
    light.enable();
    light.setPosition(ofVec3f(100,100,200));
    light.lookAt(ofVec3f(0,0,0));

    ofDisableArbTex();
    ofLoadImage(mTex,"wall.jpg");
    ofLoadImage(cTex,"car.jpg");
    ofLoadImage(fTex,"fin.jpg");


    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-0.5);
    ground = dCreatePlane (space,0,0,1,0);

    // create maze
    int b = 0;
    for (int i = 0; i < 35; i++) {
        for (int j = 0; j < 30; j++) {
            // creates a wall
            if (maze[i][j] == '#') {
                walls[b] = dCreateBox (space,scaling,scaling,1);

                dReal _x = (i * scaling) + positionx;
                dReal _y = (j * scaling) + positiony;

                dMatrix3 R;
                dRFromAxisAndAngle (R,0,1,0,0);
                dGeomSetPosition (walls[b],_x,_y,0);
                dGeomSetRotation (walls[b],R);
            }

            // creates hidden ramp(s)
            if (maze[i][j] == '@') {
                walls[b] = dCreateBox (space,scaling,scaling,1);

                dReal _x = (i * scaling) + positionx;
                dReal _y = (j * scaling) + positiony;

                dMatrix3 R;
                dRFromAxisAndAngle (R,0,1,0,-.45);
                dGeomSetPosition (walls[b],_x,_y,0);
                dGeomSetRotation (walls[b],R);
            }
            b++;
        }};

    // chassis body
    body[0] = dBodyCreate (world);
    dBodySetPosition (body[0],0,0,STARTZ);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS);
    dBodySetMass (body[0],&m);
    box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (box[0],body[0]);

    // wheel bodies
    for (i=1; i<=4; i++) {
        body[i] = dBodyCreate (world);
        dQuaternion q;
        dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
        dBodySetQuaternion (body[i],q);
        dMassSetSphere (&m,1,RADIUS);
        dMassAdjust (&m,WMASS);
        dBodySetMass (body[i],&m);
        sphere[i-1] = dCreateSphere (0,RADIUS);
        dGeomSetBody (sphere[i-1],body[i]);
    }
    dBodySetPosition (body[1],0.5*LENGTH,WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[4],0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);


    // front and back wheel hinges
    for (i=0; i<4; i++) {
        joint[i] = dJointCreateHinge2 (world,0);
        dJointAttach (joint[i],body[0],body[i+1]);
        const dReal *a = dBodyGetPosition (body[i+1]);
        dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
        dJointSetHinge2Axis1 (joint[i],0,0,1);
        dJointSetHinge2Axis2 (joint[i],0,1,0);
    }
    // set joint suspension
    for (i=0; i<4; i++) {
        dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
        dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
    }

    // lock back wheels along the steering axis
    for (i=1; i<4; i++) {
        // set stops to make sure wheels always stay in alignment
        dJointSetHinge2Param (joint[i],dParamLoStop,0);
        dJointSetHinge2Param (joint[i],dParamHiStop,0);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,sphere[0]);
    dSpaceAdd (car_space,sphere[1]);
    dSpaceAdd (car_space,sphere[2]);
    dSpaceAdd (car_space,sphere[3]);

}

//--------------------------------------------------------------
void ofApp::update(){
    // Set up the OpenFrameworks camera
    ofVec3f upVector;
    upVector.set(0, 0, 1);
    cam.setAutoDistance(false);
    p = dBodyGetPosition(body[0]);
    cam.setPosition(p[0]-15,p[1],p[2]+7.5);
    cam.lookAt({p[0],p[1],p[2]},upVector);
    cam.setUpAxis(upVector);

    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);
    dJointSetHinge2Param (joint[3],dParamVel,v);
    dJointSetHinge2Param (joint[3],dParamFMax,0.2);
    dJointSetHinge2Param (joint[3],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[3],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[3],dParamFudgeFactor,0.1);

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);

}
//--------------------------------------------------------------
void ofApp::draw(){

    // draw the scene
    ofBackground(120,245,220);

    cam.begin();

    ofEnableDepthTest();

    ofPushMatrix();
    ofSetColor(ofColor::lightPink);
    ofDrawPlane(0,0,150.0,150.0);

    stringstream t;
    // timer which is bound to the car
    const unsigned int smallvalue = (unsigned int) ofGetElapsedTimeMillis();

    int minutes = smallvalue/(60000);
    int seconds = (smallvalue/1000);
    seconds = seconds - (minutes*60);
    if(seconds>59) seconds = 0;
    int milliseconds = smallvalue%1000;
    t<<minutes;
    string mins = t.str();
    t<<seconds;
    string secs = t.str();
    if(seconds<10){
        secs = secs.substr(secs.length()-1,1);
    }else{
        secs = secs.substr(secs.length()-2,2);
    }
    t<<milliseconds;

    string msecs = t.str();
    msecs = msecs.substr(msecs.length()-3,3);
    string fulltime = mins + ":" + secs + ":" + msecs;


    // draws the timer above the car in white
    ofSetColor(ofColor::white);
    ofDrawBitmapString(fulltime, p[0]+.5,p[1]+.5,p[2]-1);


    // Set up the OpenFrameworks camera
    ofVec3f upVector;
    upVector.set(0, 0, 1);
    cam.setAutoDistance(false);
    p = dBodyGetPosition(body[0]);
    cam.setPosition(p[0]-15,p[1],p[2]+7.5);
    cam.lookAt({p[0],p[1],p[2]},upVector);
    cam.setUpAxis(upVector);

    // chassis
    ofSetColor(ofColor::white);
    cTex.bind();
    const dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    const dReal* pos_ode = dBodyGetPosition(body[0]);
    const dReal* rot_ode = dBodyGetQuaternion(body[0]);
    drawODEDemoBox(pos_ode, rot_ode, sides);
    cTex.unbind();

    // wheels
    ofSetColor(ofColor::purple);
    for (int i=1; i<=4; i++) {
        drawODEDemoCylinder(dBodyGetPosition(body[i]),
                            dBodyGetQuaternion(body[i]),0.02f,RADIUS);
    }
    //Draw maze walls
    int b = 0;
    ofSetColor(ofColor::white);
    for (int i = 0; i < 35; i++) {
        for (int j = 0; j < 30; j++) {
            if (maze[i][j] == '#') {
                mTex.bind();
                dVector3 ss; dQuaternion r;
                dGeomBoxGetLengths (walls[b],ss);
                dGeomGetQuaternion(walls[b],r);
                drawODEDemoBox(dGeomGetPosition(walls[b]),r,ss);
                mTex.unbind();

            }
            //Draw maze ramps
            if (maze[i][j] == '@') {
                fTex.bind();
                dVector3 ss; dQuaternion r;
                dGeomBoxGetLengths (walls[b],ss);
                dGeomGetQuaternion(walls[b],r);
                drawODEDemoBox(dGeomGetPosition(walls[b]),r,ss);
                fTex.unbind();
            }
            b++;
        } }


    ofDisableDepthTest();
    cam.end();

    ofPopMatrix();
}
//--------------------------------------------------------------
void ofApp::exit() {
    dGeomDestroy (box[0]);
    dGeomDestroy (sphere[0]);
    dGeomDestroy (sphere[1]);
    dGeomDestroy (sphere[2]);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
}
//--------------------------------------------------------------
static void nearCallback (void *, dGeomID o1, dGeomID o2) {

    myApp->collide(o1,o2);
}

//--------------------------------------------------------------
void ofApp::drawODEDemoCylinder(const dReal*pos_ode, const dQuaternion rot_ode, dReal len, dReal rad) {

    ofVec3f position(pos_ode[0], pos_ode[1], pos_ode[2]);

    // ODEs quaternions are stored in a different order to openFrameworks:
    ofQuaternion rotation(rot_ode[1], rot_ode[2], rot_ode[3], rot_ode[0]);

    float rotationAmount;
    ofVec3f rotationAngle;
    rotation.getRotate(rotationAmount, rotationAngle);

    ofPushMatrix();
    ofTranslate(position);

    ofRotateDeg(rotationAmount, rotationAngle.x, rotationAngle.y, rotationAngle.z);

    // ODE Drawstuff's cylinder defaults to standing on its end (along Z axis).
    // OpenFramework's cylinder lies on its side (along Y axis). For drawing purposes,
    // we need another rotation of 90 degrees along X axis to make it look right.
    ofRotateDeg(90.0,1,0,0);

    ofDrawCylinder(rad,len);
    ofPopMatrix();
}

void ofApp::drawODEDemoBox(const dReal*pos_ode, const dQuaternion rot_ode, const dReal*sides_ode){


    dReal siz[3] = {sides_ode[0],sides_ode[1],sides_ode[2]};

    ofVec3f      position(pos_ode[0], pos_ode[1], pos_ode[2]);

    // ODEs quaternions are stored in a different order to openFrameworks:
    ofQuaternion rotation(rot_ode[1], rot_ode[2], rot_ode[3], rot_ode[0]);

    float rotationAmount;
    ofVec3f rotationAngle;
    rotation.getRotate(rotationAmount, rotationAngle);

    ofPushMatrix();
    ofTranslate(position);

    ofRotateDeg(rotationAmount, rotationAngle.x, rotationAngle.y, rotationAngle.z);
    ofDrawBox(0,0,0, siz[0],siz[1],siz[2]);
    ofPopMatrix();
}

void ofApp::collide(dGeomID o1, dGeomID o2)
{
    int i,n;
    // buggy will collide with everything
    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0) {
        for (i=0; i<n; i++) {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.3;
            dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
            dJointAttach (c,
                          dGeomGetBody(contact[i].geom.g1),
                          dGeomGetBody(contact[i].geom.g2));
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    // controls are WASD, space for break
    switch(key) {
    case 'w': case 'W':
        speed += 0.5;
        break;
    case 's': case 'S':
        speed -= 0.5;
        break;
    case 'a': case 'A':
        steer -= 0.25;
        break;
    case 'd': case 'D':
        steer += 0.25;
        break;
    case ' ':
        speed = 0;
        steer = 0;
        break;
    case 'q':
        ofExit();
        break;
    case 'r':
        ofApp::setup();
        ofResetElapsedTimeCounter();
        break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
