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
	auto boids9D = convertTo9D(swarm.getBoids());
	engine.updateBoids(boids9D);
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);
	engine.drawDebug(cam);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    // Press 1,2,3,4 to switch between swarm presets
    switch(key)
    {
        case '1':
            engine.applyPreset(preset1);
            ofLog() << "Preset 1 applied: bright, smooth, low density trigger";
            break;
        case '2':
            engine.applyPreset(preset2);
            ofLog() << "Preset 2 applied: medium flow, medium trigger probability";
            break;
        case '3':
            engine.applyPreset(preset3);
            ofLog() << "Preset 3 applied: dense timbre, strong amplitude response";
            break;
        case '4':
            engine.applyPreset(preset4);
            ofLog() << "Preset 4 applied: slow, sparse, long envelopes";
            break;
    }
}

