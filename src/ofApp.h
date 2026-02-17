#pragma once

#include "ofMain.h"
#include "SwarmOSCReceiver.h"
#include "BoidSoundEngine.h"

class ofApp : public ofBaseApp
{
public:
	void setup() override;
	void update() override;
	void draw() override;
	void keyPressed(int key) override;
	
	ofSoundStream soundStream;
	SwarmOSCReceiver swarm;
	AdaptiveBoidSoundEngine engine;
	ofEasyCam cam;
};
