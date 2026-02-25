#include "ofApp.h"

//--------------------------------------------------------------
ofxDatGuiSlider* ofApp::addBoundSlider(ofxDatGuiFolder* folder,
                                       const std::string& label,
                                       float min,
                                       float max,
                                       float value,
                                       std::atomic<double>& param)
{
    auto* s = folder->addSlider(label, min, max, value);
    sliderBindings.push_back({s, &param});
    return s;
}

//--------------------------------------------------------------
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

	gui = new ofxDatGui(ofxDatGuiAnchor::TOP_LEFT);
	gui->setWidth(1024);
	gui->addHeader("BOID SOUND ENGINE");
	gui->addBreak();

	int totalWidth = 824;
	float labelWidth = 200;

	// --- TRIGGER ---
	fTrigger = gui->addFolder("TRIGGER");
	addBoundSlider(fTrigger, "MIX", 0.0, 1.0, TRIGGER_MIX.load(), TRIGGER_MIX)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fTrigger, "MIN RATE", 1.0, 200.0, TRIGGER_MIN_RATE.load(), TRIGGER_MIN_RATE)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fTrigger, "MAX RATE", 50.0, 1000.0, TRIGGER_MAX_RATE.load(), TRIGGER_MAX_RATE)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fTrigger, "CURVE EXP", 0.1, 2.0, TRIGGER_CURVE_EXPONENT.load(), TRIGGER_CURVE_EXPONENT)->setWidth(totalWidth, labelWidth);
	fTrigger->expand();

	// --- FREQUENCY ---
	fFrequency = gui->addFolder("FREQUENCY");
	addBoundSlider(fFrequency, "FREQ MIN", 20.0, 500.0, FREQ_MIN.load(), FREQ_MIN)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fFrequency, "FREQ MAX", 200.0, 5000.0, FREQ_MAX.load(), FREQ_MAX)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fFrequency, "DENSITY INFLUENCE", 0.0, 3.0, FREQ_DENSITY_INFLUENCE.load(), FREQ_DENSITY_INFLUENCE)->setWidth(totalWidth, labelWidth);
	fFrequency->expand();

	// --- MODAL STRUCTURE ---
	fModal = gui->addFolder("MODAL STRUCTURE");
	addBoundSlider(fModal, "FREQ SPREAD", 0.0, 0.1, MODE_FREQ_SPREAD.load(), MODE_FREQ_SPREAD)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fModal, "BW BASE", 10.0, 300.0, MODE_BW_BASE.load(), MODE_BW_BASE)->setWidth(totalWidth, labelWidth);
	addBoundSlider(fModal, "AMP DECAY", 0.0, 1.0, MODE_AMP_DECAY.load(), MODE_AMP_DECAY)->setWidth(totalWidth, labelWidth);
	fModal->expand();

	// --- SPATIAL ---
	fSpatial = gui->addFolder("SPATIAL");
	addBoundSlider(fSpatial, "GAIN EXP", 0.1, 6.0, SPATIAL_GAIN_EXP.load(), SPATIAL_GAIN_EXP)->setWidth(totalWidth, labelWidth);
	fSpatial->expand();

	// --- OUTPUT ---
	fOutput = gui->addFolder("OUTPUT");
	addBoundSlider(fOutput, "TANH DRIVE", 0.5, 10.0, OUTPUT_TANH_DRIVE.load(), OUTPUT_TANH_DRIVE)->setWidth(totalWidth, labelWidth);
	fOutput->expand();

    gui->onSliderEvent(this, &ofApp::onSliderEvent);
}

//--------------------------------------------------------------
void ofApp::update()
{
    swarm.update();
    auto boids9D = convertTo9D(swarm.getBoids());
    engine.updateBoids(boids9D);

    if (gui) gui->update();
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackground(0);

    int guiWidth   = 1024;
    int rightWidth = ofGetWidth() - guiWidth;
    int height     = ofGetHeight();

    ofPushStyle();
    ofSetColor(30);
    ofDrawRectangle(0, 0, guiWidth, height);
    ofPopStyle();

    ofViewport(guiWidth, 0, rightWidth, height);

    cam.begin();

    ofSetColor(80, 200, 255);
    for (const auto& b : engine.getBoids())
    {
        glm::vec3 pos = engine.getBoidPosition(b.index);
        ofDrawSphere(pos, 3.0f);
    }

    cam.end();

	ofViewport(0, 0, ofGetWidth(), ofGetHeight());

    gui->draw();
}

//--------------------------------------------------------------
void ofApp::onSliderEvent(ofxDatGuiSliderEvent e)
{
    for (auto& binding : sliderBindings)
    {
        if (e.target == binding.slider)
        {
            binding.parameter->store(e.value);
            break;
        }
    }
}
