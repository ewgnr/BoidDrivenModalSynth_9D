#pragma once

#include "ofMain.h"
#include "ofxDatGui.h"

#include "SwarmOSCReceiver.h"
#include "BoidSoundEngine.h"

class ofApp : public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void draw() override;

    void onSliderEvent(ofxDatGuiSliderEvent e);

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
        ofxDatGuiSlider* slider;
        std::atomic<double>* parameter;
    };

    std::vector<SliderBinding> sliderBindings;

    ofxDatGuiSlider* addBoundSlider(ofxDatGuiFolder* folder,
                                    const std::string& label,
                                    float min,
                                    float max,
                                    float value,
                                    std::atomic<double>& param);
};
