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
void ofApp::setup()
{
    ofSetFrameRate(60);

    swarm.setup(9005);
    engine.setup(48);

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
	fTrigger = gui->addFolder("TRIGGER / EXCITATION");

	addBoundSlider(fTrigger, "MIX", 0.0, 1.0, TRIGGER_MIX.load(), TRIGGER_MIX);
	addBoundSlider(fTrigger, "MIN RATE", 1.0, 200.0, TRIGGER_MIN_RATE.load(), TRIGGER_MIN_RATE);
	addBoundSlider(fTrigger, "MAX RATE", 50.0, 1000.0, TRIGGER_MAX_RATE.load(), TRIGGER_MAX_RATE);
	addBoundSlider(fTrigger, "CURVE", 0.1, 2.0, TRIGGER_CURVE_EXPONENT.load(), TRIGGER_CURVE_EXPONENT);
	addBoundSlider(fTrigger, "ACTIVITY GAIN", 0.0, 3.0, TRIGGER_ACTIVITY_GAIN.load(), TRIGGER_ACTIVITY_GAIN);
	addBoundSlider(fTrigger, "ABS DIST SCALE", 50.0, 2000.0, ABSOLUTE_DISTANCE_SCALE.load(), ABSOLUTE_DISTANCE_SCALE);
	fTrigger->expand();

	// --- SPATIAL FIELD ---
	fSpatial = gui->addFolder("SPATIAL FIELD");
	addBoundSlider(fSpatial, "GAIN EXP", 0.1, 6.0, SPATIAL_GAIN_EXP.load(), SPATIAL_GAIN_EXP);
	addBoundSlider(fSpatial, "DIST MIN", 0.01, 1.0, SPATIAL_DISTANCE_MIN.load(), SPATIAL_DISTANCE_MIN);
	addBoundSlider(fSpatial, "DIST MAX", 0.1, 5.0, SPATIAL_DISTANCE_MAX.load(), SPATIAL_DISTANCE_MAX);
	addBoundSlider(fSpatial, "WIDTH", 0.2, 3.0, SPATIAL_WIDTH.load(), SPATIAL_WIDTH);
	addBoundSlider(fSpatial, "DENSITY SCALE", 0.1, 5.0, SPATIAL_DENSITY_EXP_SCALE.load(), SPATIAL_DENSITY_EXP_SCALE);
	fSpatial->expand();

	// --- FREQUENCY FIELD ---
	fFrequency = gui->addFolder("FREQUENCY FIELD");
	addBoundSlider(fFrequency, "FREQ MIN", 20.0, 500.0, FREQ_MIN.load(), FREQ_MIN);
	addBoundSlider(fFrequency, "FREQ MAX", 200.0, 8000.0, FREQ_MAX.load(), FREQ_MAX);
	addBoundSlider(fFrequency, "DENSITY INFL", 0.0, 3.0, FREQ_DENSITY_INFLUENCE.load(), FREQ_DENSITY_INFLUENCE);
	addBoundSlider(fFrequency, "TRANSPOSE", 0.25, 4.0, FREQ_TRANSPOSE.load(), FREQ_TRANSPOSE);
	addBoundSlider(fFrequency, "RADIAL EXP", 0.1, 2.0, FREQ_RADIAL_EXPONENT.load(), FREQ_RADIAL_EXPONENT);
	addBoundSlider(fFrequency, "SMOOTHING", 0.8, 0.999, FREQ_SMOOTHING.load(), FREQ_SMOOTHING);
	fFrequency->expand();

	// --- MODAL ENGINE ---
	fModal = gui->addFolder("MODAL ENGINE");
	addBoundSlider(fModal, "FREQ SPREAD", 0.0, 0.1, MODE_FREQ_SPREAD.load(), MODE_FREQ_SPREAD);
	addBoundSlider(fModal, "BW BASE", 10.0, 300.0, MODE_BW_BASE.load(), MODE_BW_BASE);
	addBoundSlider(fModal, "AMP DECAY", 0.0, 1.0, MODE_AMP_DECAY.load(), MODE_AMP_DECAY);
	addBoundSlider(fModal, "BRIGHTNESS", 0.1, 3.0, MODE_BRIGHTNESS.load(), MODE_BRIGHTNESS);
	addBoundSlider(fModal, "AMP BASE", 0.0, 1.0, MODE_AMP_BASE.load(), MODE_AMP_BASE);
	fModal->expand();

	// --- OUTPUT STAGE ---
	fOutput = gui->addFolder("OUTPUT STAGE");
	addBoundSlider(fOutput, "TANH DRIVE", 0.5, 10.0, OUTPUT_TANH_DRIVE.load(), OUTPUT_TANH_DRIVE);
	addBoundSlider(fOutput, "MASTER GAIN", 0.0, 2.0, OUTPUT_MASTER_GAIN.load(), OUTPUT_MASTER_GAIN);
	fOutput->expand();

	gui->onSliderEvent(this, &ofApp::onSliderEvent);
}

//--------------------------------------------------------------
void ofApp::update()
{
    swarm.update();
    engine.updateBoids(swarm.getBoids());

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
		glm::vec3 pos(
			b.position[0],
			b.position[1],
			b.position[2]
		);

		ofDrawSphere(pos * 50.0f, 1.0f);
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
