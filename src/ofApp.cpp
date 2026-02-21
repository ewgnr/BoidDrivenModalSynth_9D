#include "ofApp.h"

std::vector<BoidState9D> convertTo9D(const std::vector<Boid>& swarmBoids)
{
    const float POSITION_SCALE = 70.0f;
    const float VELOCITY_SCALE = 1.0f;

    std::vector<BoidState9D> boids9D;
    boids9D.reserve(swarmBoids.size());

    for (const auto& b : swarmBoids)
    {
        BoidState9D b9;
        b9.index = b.index;

        for (int d = 0; d < 3; d++)
            b9.dims[d] = b.position[d] * POSITION_SCALE;

        for (int d = 3; d < DIMS_PER_BOID; d++)
            b9.dims[d] = b.position[d];

        for (int d = 0; d < 3; d++)
            b9.velocity[d] = b.velocity[d] * VELOCITY_SCALE;

        for (int d = 3; d < DIMS_PER_BOID; d++)
            b9.velocity[d] = b.velocity[d];

        boids9D.push_back(b9);
    }

    return boids9D;
}

//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetFrameRate(60);
	swarm.setup(9005);
	engine.setup(16);

    ofSoundStreamSettings settings;
    settings.setOutListener(&engine);
    settings.numOutputChannels = 2;
    settings.numInputChannels = 0;
    settings.sampleRate = SAMPLERATE;
    settings.bufferSize = BUFFERSIZE;
    settings.numBuffers = 4;
	settings.setApi(ofSoundDevice::Api::MS_WASAPI);
    soundStream.setup(settings);
}

//--------------------------------------------------------------
void ofApp::update()
{
	swarm.update();
	// swarm.printMinMax();
	auto boids9D = convertTo9D(swarm.getBoids());
	engine.updateBoids(boids9D);
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);
    cam.begin();
    ofSetColor(80, 200, 255);
    for (const auto& b : engine.getBoids())  // you need a getter in the engine
    {
        glm::vec3 pos = engine.getBoidPosition(b.index); // getter for spatialPos
        ofDrawSphere(pos, 3.0f);
    }
    cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
}

