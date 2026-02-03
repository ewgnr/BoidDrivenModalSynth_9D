#pragma once
#include "ofMath.h"
#include "ofSoundStream.h"
#include "ModalBank2D.h"
#include "LinearEnvelope.h"
#include "PinkNoiseGenerator.h"
#include "ambiEncode2DThirdOrder.h"
#include "ambiDecode2DThirdOrder.h"
#include <array>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

constexpr double SAMPLERATE = 44100;
constexpr double BUFFERSIZE = 1024;
constexpr double CHANNELS = 2;

#define DIMS_PER_BOID 9

struct BoidState9D
{
    int index = 0;
    std::array<float, DIMS_PER_BOID> dims{};
    std::array<float, DIMS_PER_BOID> velocity{};
};

struct Subspace
{
    std::array<int,3> dims;   // dimension indices
    float weight = 1.0f;      // aggregation weight
    float radius = 0.6f;      // density neighborhood
};

class BoidAggregator
{
public:
    void setup(int maxBoids)
    {
        spatialPos.resize(maxBoids, glm::vec3(0.0f));
        densities.resize(maxBoids, 0.0f);
    }

    void setSubspaces(const std::vector<Subspace>& s)
    {
        subspaces = s;
        normalizeWeights();
    }

    void setSpatialSmoothing(float smooth) { spatialSmooth = smooth; }

    void update(const std::vector<BoidState9D>& boids)
    {
        if(subspaces.empty()) return;
        N = static_cast<int>(boids.size());
        computeSpatialPositions(boids);
        computeDensities(boids);
    }

    const glm::vec3& getSpatialPos(int i) const { return spatialPos[i]; }
    float getDensity(int i) const { return densities[i]; }

private:
    int N = 0;
    std::vector<Subspace> subspaces;
    std::vector<glm::vec3> spatialPos;
    std::vector<float> densities;

    float spatialSmooth = 0.98f;

    glm::vec3 extractPos(const BoidState9D& b, const Subspace& s) const
    {
        return {
            b.dims[s.dims[0]],
            b.dims[s.dims[1]],
            b.dims[s.dims[2]]
        };
    }

    void normalizeWeights()
    {
        float sum = 0.0f;
        for(const auto& s : subspaces) sum += s.weight;
        if(sum > 0.0f)
            for(auto& s : subspaces) s.weight /= sum;
    }

    void computeSpatialPositions(const std::vector<BoidState9D>& boids)
    {
        for(int i = 0; i < N; i++)
        {
            glm::vec3 agg(0.0f);
            for(const auto& s : subspaces)
                agg += s.weight * extractPos(boids[i], s);

            spatialPos[i] = spatialSmooth * spatialPos[i]
                          + (1.0f - spatialSmooth) * agg;
        }
    }

    void computeDensities(const std::vector<BoidState9D>& boids)
    {
        for(int i = 0; i < N; i++)
        {
            float d = 0.0f;
            for(const auto& s : subspaces)
            {
                int neighbors = 0;
                glm::vec3 pi = extractPos(boids[i], s);
                for(int j = 0; j < N; j++)
                {
                    if(i == j) continue;
                    glm::vec3 pj = extractPos(boids[j], s);
                    if(glm::length(pi - pj) < s.radius)
                        neighbors++;
                }
                float local = std::min(neighbors / 8.0f, 1.0f);
                d += s.weight * local;
            }
            densities[i] = d; // unsmoothed
        }
    }
};

// --- Swarm Presets ---
struct SwarmPreset
{
    std::vector<Subspace> subspaces;
    float spatialSmooth;
    float densityThreshold;
    float triggerProbability;
    std::vector<std::pair<double,double>> envelopeShape;
    double bandwidthScale;
    double detuneScale;
};

// Example presets
const SwarmPreset preset1 {
    { { {0,1,2}, 0.7f, 0.8f }, { {3,4,5}, 0.2f, 0.6f }, { {6,7,8}, 0.1f, 0.5f } },
    0.99f, 0.3f, 0.01f,
    { {0.5,1.5}, {2.0,0.3} },
    1.0, 0.02
};
const SwarmPreset preset2 {
    { { {0,1,2}, 0.3f, 0.5f }, { {3,4,5}, 0.6f, 0.4f }, { {6,7,8}, 0.1f, 0.3f } },
    0.92f, 0.2f, 0.1f,
    { {0.05,0.1}, {0.2,0.05} },
    1.2, 0.05
};
const SwarmPreset preset3 {
    { { {0,1,2}, 0.2f, 0.6f }, { {3,4,5}, 0.2f, 0.6f }, { {6,7,8}, 0.6f, 0.7f } },
    0.96f, 0.4f, 0.05f,
    { {1.0,3.0}, {4.0,1.0} },
    1.5, 0.08
};
const SwarmPreset preset4 {
    { { {0,1,2}, 0.4f, 0.5f }, { {3,4,5}, 0.3f, 0.5f }, { {6,7,8}, 0.3f, 0.5f } },
    0.95f, 0.6f, 0.005f,
    { {0.5,5.0}, {0.1,0.2} },
    1.0, 0.02
};

class BoidSoundEngine : public ofBaseSoundOutput
{
public:
    BoidSoundEngine() = default;

    void setup(int maxBoids)
    {
        N = maxBoids;

        aggregator.setup(N);
        applyPreset(preset1); // default

        modalBank2D.setup(N, 4, SAMPLERATE);
        modalBank2D.initRandom();

        smoothedFreq.resize(N, 0.0);
        envelopes.resize(N);
        densities.resize(N, 0.0);

        for(auto & env : envelopes) env.set(currentPreset.envelopeShape);

        speakerAz = { -0.5 * PI, 0.5 * PI };
    }

    void applyPreset(const SwarmPreset& p)
    {
        currentPreset = p;
        aggregator.setSubspaces(p.subspaces);
        aggregator.setSpatialSmoothing(p.spatialSmooth);

        densityThreshold = p.densityThreshold;
        triggerProbability = p.triggerProbability;

        bandwidthScale = p.bandwidthScale;
        detuneScale = p.detuneScale;

        for(auto& env : envelopes)
            env.set(p.envelopeShape);
    }

    void updateBoids(const std::vector<BoidState9D> & boidsIn)
    {
        boids = boidsIn;
        N = static_cast<int>(boids.size());

        if(smoothedFreq.size() != N) smoothedFreq.resize(N, 0.0);
        if(envelopes.size() != N) envelopes.resize(N);
        if(densities.size() != N) densities.resize(N, 0.0);

        aggregator.update(boids);

        for(int i = 0; i < N; i++)
        {
            float d = aggregator.getDensity(i);
            densities[i] = d;

            // use densityThreshold and triggerProbability
            if(d > densityThreshold && ofRandom(1.0f) < triggerProbability)
                envelopes[i].reset();
        }
    }

    void processModalParams()
    {
        const double smooth = 0.995;

        for(int j=0; j<N; j++)
        {
            const auto & b = boids[j];

            glm::vec3 r(b.dims[0], b.dims[1], b.dims[2]);
            if(glm::length(r) > 1e-6) r = glm::normalize(r);

            glm::vec3 t1(b.dims[3], b.dims[4], b.dims[5]);
            t1 = t1 - glm::dot(t1,r)*r;
            double flow = glm::length(t1);
            if(flow > 1e-6) t1 = glm::normalize(t1);
            else t1 = glm::vec3(0.0f);

            glm::vec3 t2 = glm::cross(r,t1);
            double curvature = glm::length(t2);
            if(curvature > 1e-6) t2 = glm::normalize(t2);

            double radialHeight = 0.5*(r.y + 1.0);
            double speed = glm::length(glm::vec3(b.velocity[0], b.velocity[1], b.velocity[2]));
            double targetFreq = 140.0 + 320.0*radialHeight + 120.0*speed;
            smoothedFreq[j] = smooth*smoothedFreq[j] + (1.0 - smooth)*targetFreq;

            double ampBase = 0.05 + 0.5*flow;
            double d = densities[j];
            ampBase *= 0.5 + 0.5*d;
            ampBase = std::clamp(ampBase, 0.01, 1.0);

            double baseBandwidth = (180.0 + 150.0*flow) * bandwidthScale;
            double detuneAmt = (0.05*curvature + 0.02*speed) * detuneScale;

            static const double harmonicsSparse[4] = {1.0, 1.01, 1.02, 1.03};
            static const double harmonicsMedium[4] = {1.0, 1.125, 1.25, 1.3333};
            static const double harmonicsDense[4] = {1.0, 1.02, 1.04, 1.06};
            const double* harmonics = (d<0.33 ? harmonicsSparse : (d<0.66 ? harmonicsMedium : harmonicsDense));

            const size_t numModes = modalBank2D.getOutputs()[j].size();
            for(size_t m=0; m<numModes; m++)
            {
                double frq = smoothedFreq[j] * harmonics[m%4] * (1.0 + detuneAmt*double(m));
                double bandwidth = baseBandwidth*(1.0 + 0.25*m);
                double amp = ampBase*(1.0 / (1.0 + 0.6*m));
                modalBank2D.setParams(j,m,frq,bandwidth,amp);
            }
        }
    }

    void audioOut(ofSoundBuffer& buffer) override
    {
        processModalParams();

        for(size_t i=0; i<buffer.getNumFrames(); i++)
        {
            for(int j=0; j<N; j++)
            {
                double env = envelopes[j].play(1.0/SAMPLERATE);
                if(env > 0.0)
                {
                    double x = env * noiseGen.play() * 0.2;
                    modalBank2D.exciteSource(j,x);
                }
            }

            auto boidOutputs = modalBank2D.playMulti();

            std::array<double,7> ambiFrame{};
            ambiFrame.fill(0.0);

            for(int j=0; j<N; j++)
            {
                glm::vec3 p = aggregator.getSpatialPos(j);
                double az = std::atan2(p.z, p.x);
                double dist = glm::length(p);
                dist = std::clamp(dist,0.05,1.0);
                double distanceGain = std::exp(-3.0*dist);
                auto boidFrame = ambiEnc.play(boidOutputs[j] * distanceGain, az, dist);
                for(size_t k=0; k<7; k++) ambiFrame[k] += boidFrame[k];
            }

            std::array<double,2> stereo{};
            for(size_t ch=0; ch<2; ch++)
            {
                double sOut = ambiDec.play(ambiFrame, speakerAz[ch]);
                stereo[ch] = std::tanh(sOut);
            }

            for(size_t ch=0; ch<2; ch++)
                buffer[i*2 + ch] = float(stereo[ch]);
        }
    }

    void drawDebug(ofCamera& cam)
    {
        cam.begin();
        ofSetColor(80, 200, 255);
        for(const auto & b : boids)
        {
            glm::vec3 pos = aggregator.getSpatialPos(b.index);
            ofDrawSphere(pos, 3.0f);
        }
        cam.end();
    }

private:
    int N = 0;
    std::vector<BoidState9D> boids;
    std::vector<double> smoothedFreq;
    std::vector<double> densities;

    BoidAggregator aggregator;
    ModalBank2D modalBank2D;
    PinkNoiseGenerator noiseGen;
    AmbiEncode2D ambiEnc;
    AmbiDecode2D ambiDec;
    std::vector<LinearEnvelope> envelopes;
    std::array<double,2> speakerAz;

    // --- preset control ---
    SwarmPreset currentPreset;
    float densityThreshold = 0.3f;
    float triggerProbability = 0.01f;
    double bandwidthScale = 1.0;
    double detuneScale = 0.02;
};
