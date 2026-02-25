#pragma once
#include <vector>
#include "ModalFilter.h"
#include "ofMain.h"

class ModalBank2D
{
public:
    // Setup with N sources (boids) and M modes per source
    void setup(size_t numSources, size_t numModesPerSource, double sampleRate)
    {
        sources = numSources;
        modesPerSource = numModesPerSource;

        nodes.resize(numSources);
        weights.resize(numSources);
        excitation.resize(numSources);
        output.resize(numSources);

        for (size_t i = 0; i < numSources; i++)
        {
            nodes[i].resize(numModesPerSource);
            weights[i].assign(numModesPerSource, 1.0 / numModesPerSource);
            excitation[i].assign(numModesPerSource, 0.0);
            output[i].assign(numModesPerSource, 0.0);

            for (auto & n : nodes[i])
                n.setSampleRate(sampleRate);
        }
    }

    // Initialize all modes with random parameters
    void initRandom(double baseFreq = 200.0, double freqRange = 800.0)
    {
        for (size_t i = 0; i < nodes.size(); i++)
        {
            for (auto & n : nodes[i])
            {
                double frq = baseFreq + freqRange * ofRandom(1.0);
                double q = 5.0 + 10.0 * ofRandom(1.0);
                n.init(frq, q, 0.05);
            }
        }
    }

    // Set parameters for a specific mode of a specific source
    void setParams(size_t sourceIdx, size_t modeIdx, double frq, double q, double amp)
    {
        if (sourceIdx < nodes.size() && modeIdx < nodes[sourceIdx].size())
            nodes[sourceIdx][modeIdx].setParams(frq, q, amp);
    }

    // Excite a specific mode of a specific source
    void excite(size_t sourceIdx, size_t modeIdx, double value)
    {
        if (sourceIdx < excitation.size() && modeIdx < excitation[sourceIdx].size())
            excitation[sourceIdx][modeIdx] += value;
    }

    // Excite all modes of a specific source (useful for boids)
    void exciteSource(size_t sourceIdx, double value)
    {
        if (sourceIdx >= excitation.size()) return;

        for (size_t j = 0; j < excitation[sourceIdx].size(); j++)
            excitation[sourceIdx][j] += value;
    }

    // Play all modes and return summed outputs per source
    void playMulti(std::vector<double>& sums)
    {
    const size_t numSources = nodes.size();

    for (size_t i = 0; i < numSources; i++)
        sums[i] = 0.0;

	    for (size_t i = 0; i < numSources; i++) // per source
        {
            for (size_t j = 0; j < nodes[i].size(); j++) // per mode
            {
            output[i][j] = nodes[i][j].play(excitation[i][j]);
            sums[i] += output[i][j] * weights[i][j];
            excitation[i][j] = 0.0;
            }
        }
    }

	size_t getNumModes(size_t sourceIdx) const
	{
		if (sourceIdx >= nodes.size()) return 0;
		return nodes[sourceIdx].size();
	}

    // Access per-mode outputs if needed
    const std::vector<std::vector<double>> & getOutputs() const { return output; }

private:
    size_t sources = 0;
    size_t modesPerSource = 0;

    std::vector<std::vector<ModalFilter>> nodes;
    std::vector<std::vector<double>> weights;
    std::vector<std::vector<double>> excitation;
    std::vector<std::vector<double>> output;
};
