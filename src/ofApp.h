#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"

#include "SwarmOSCReceiver.h"
#include "BoidSoundEngine.h"

#include "ParamRegistry.h"

class ofApp : public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void draw() override;

    void onSliderEvent(ofxDatGuiSliderEvent e);

	void saveAudioPreset(const std::string& presetName);
	void loadAudioPreset(const std::string& presetName);
	void syncGuiToParams();

	void keyPressed(int key);

    ofSoundStream soundStream;
    SwarmOSCReceiver swarm;
    AdaptiveBoidSoundEngine engine;
    ofEasyCam cam;

    ofxDatGui* gui = nullptr;

    ofxDatGuiFolder* fTrigger = nullptr;
    ofxDatGuiFolder* fFrequency = nullptr;
    ofxDatGuiFolder* fModal = nullptr;
    ofxDatGuiFolder* fSpatial = nullptr;
    ofxDatGuiFolder* fOutput = nullptr;

    struct SliderBinding
    {
		std::string name;
		ofxDatGuiSlider* slider = nullptr;
		std::atomic<double>* parameter = nullptr;
    };

    std::vector<SliderBinding> sliderBindings;

    ofxDatGuiSlider* addBoundSlider(ofxDatGuiFolder* folder,
                                    const std::string& label,
                                    float min,
                                    float max,
                                    float value,
                                    std::atomic<double>& param);

	enum PresetMode
    {
        NONE,
        SAVE_MODE,
        LOAD_MODE
    };

    PresetMode presetMode = NONE;

    int selectedPreset = 1;
    const int maxPresets = 20;
};
