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

// 9D Boid state
struct BoidState9D
{
    int index = 0;
    std::array<float,DIMS_PER_BOID> dims{};
    std::array<float,DIMS_PER_BOID> velocity{};
};

// Subspace for aggregation
struct Subspace
{
    std::array<int,3> dims;
    float weight = 1.0f;
    float radius = 0.6f;
};

// Aggregator for spatial position & density
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
        return { b.dims[s.dims[0]], b.dims[s.dims[1]], b.dims[s.dims[2]] };
    }

    void normalizeWeights()
    {
        float sum = 0.0f;
        for(const auto& s : subspaces) sum += s.weight;
        if(sum>0.0f) for(auto& s : subspaces) s.weight /= sum;
    }

    void computeSpatialPositions(const std::vector<BoidState9D>& boids)
    {
        for(int i=0;i<N;i++)
        {
            glm::vec3 agg(0.0f);
            for(const auto& s : subspaces)
                agg += s.weight * extractPos(boids[i],s);
            spatialPos[i] = spatialSmooth*spatialPos[i] + (1.0f - spatialSmooth)*agg;
        }
    }

    void computeDensities(const std::vector<BoidState9D>& boids)
    {
        for(int i=0;i<N;i++)
        {
            float d = 0.0f;
            for(const auto& s : subspaces)
            {
                int neighbors=0;
                glm::vec3 pi = extractPos(boids[i],s);
                for(int j=0;j<N;j++)
                {
                    if(i==j) continue;
                    glm::vec3 pj = extractPos(boids[j],s);
                    if(glm::length(pi - pj) < s.radius) neighbors++;
                }
                float local = std::min(neighbors / 8.0f,1.0f);
                d += s.weight*local;
            }
            densities[i] = d;
        }
    }
};

// --- Adaptive preset ---
struct AdaptivePreset
{
    std::vector<Subspace> subspaces;
    float spatialSmooth = 0.96f;
    std::vector<std::pair<double,double>> envelopeShape = { {0.5,1.0}, {1.0,0.5} };
    double bandwidthScale = 1.0;
    double detuneScale = 0.02;
    double minFreq = 100.0;
    double maxFreq = 1200.0;      // größere Range für dramatische Wechsel
    double minAmp  = 0.01;
    double maxAmp  = 0.7;         // höhere Maximalamplitude für sparse Boids
};

// --- Adaptive single instrument engine ---
// --- Single adaptive instrument mode ---
class AdaptiveBoidSoundEngine : public ofBaseSoundOutput
{
public:
    void setup(int maxBoids)
    {
        N = maxBoids;
        aggregator.setup(N);

        // Only one adaptive preset
        preset.subspaces = {
            { {0,1,2}, 0.5f, 0.6f },
            { {3,4,5}, 0.3f, 0.5f },
            { {6,7,8}, 0.2f, 0.4f }
        };

        aggregator.setSubspaces(preset.subspaces);
        aggregator.setSpatialSmoothing(preset.spatialSmooth);

        modalBank2D.setup(N, 4, SAMPLERATE);
        modalBank2D.initRandom();

        smoothedFreq.resize(N, 0.0);
        envelopes.resize(N);
        densities.resize(N, 0.0);

        // Feste Envelope: nicht skaliert
        for(auto & env : envelopes) 
            env.set(preset.envelopeShape);

        speakerAz = { -0.5*PI, 0.5*PI };
    }

    void updateBoids(const std::vector<BoidState9D> & boidsIn)
    {
        boids = boidsIn;
        N = static_cast<int>(boids.size());

        if(smoothedFreq.size() != N) smoothedFreq.resize(N,0.0);
        if(envelopes.size() != N) envelopes.resize(N);
        if(densities.size() != N) densities.resize(N,0.0);

        aggregator.update(boids);

        for(int i=0; i<N; i++)
        {
            float density = densities[i];
            float speed = glm::length(glm::vec3(
                boids[i].velocity[0],
                boids[i].velocity[1],
                boids[i].velocity[2]));

            // Feste Trigger-Wahrscheinlichkeit abhängig von Density/Speed
            float triggerProb = 0.1f + 0.2f * speed; // einfach + minimal
            if(ofRandom(1.0f) < triggerProb)
                envelopes[i].reset(); // feste Envelope
        }
    }

    void processModalParams()
    {
        const double smooth = 0.995;
        for(int j=0;j<N;j++)
        {
            const auto & b = boids[j];
            glm::vec3 r(b.dims[0], b.dims[1], b.dims[2]);
            if(glm::length(r) > 1e-6) r = glm::normalize(r);

            glm::vec3 t1(b.dims[3], b.dims[4], b.dims[5]);
            t1 = t1 - glm::dot(t1,r)*r;
            double flow = glm::length(t1);
            if(flow > 1e-6) t1 = glm::normalize(t1); else t1 = glm::vec3(0.0f);

            glm::vec3 t2 = glm::cross(r,t1);
            double curvature = glm::length(t2);
            if(curvature > 1e-6) t2 = glm::normalize(t2);

            double radialHeight = 0.5*(r.y + 1.0);
            double targetFreq = preset.minFreq + (preset.maxFreq - preset.minFreq)*radialHeight;
            smoothedFreq[j] = smooth*smoothedFreq[j] + (1.0 - smooth)*targetFreq;

            double ampBase = (preset.minAmp + preset.maxAmp)/2.0; // feste Amplitude

            double baseBandwidth = 100.0 + 150.0*flow;
            baseBandwidth *= preset.bandwidthScale;
            baseBandwidth = std::clamp(baseBandwidth,80.0,400.0);

            double detuneAmt = (0.05*curvature + 0.02*glm::length(glm::vec3(b.velocity[0],b.velocity[1],b.velocity[2])))*preset.detuneScale;

            const size_t modes = modalBank2D.getOutputs()[j].size();
            for(size_t m=0;m<modes;m++)
            {
                double frq = smoothedFreq[j]*(1.0 + detuneAmt*m);
                double bw  = baseBandwidth*(1.0 + 0.2*m);
                double a   = ampBase / (1.0 + 0.5*m);
                modalBank2D.setParams(j,m,frq,bw,a);
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
                    double x = env*noiseGen.play()*0.5; // feste Excitation
                    modalBank2D.exciteSource(j,x);
                }
            }

            auto boidOutputs = modalBank2D.playMulti();
            std::array<double,7> ambiFrame{}; ambiFrame.fill(0.0);

            for(int j=0;j<N;j++)
            {
                glm::vec3 p = aggregator.getSpatialPos(j);
                double az = std::atan2(p.z,p.x);
                double dist = glm::length(p);
                dist = std::clamp(dist,0.05,1.0);
                double distanceGain = std::exp(-3.0*dist);
                auto boidFrame = ambiEnc.play(boidOutputs[j]*distanceGain, az, dist);
                for(size_t k=0;k<7;k++) ambiFrame[k] += boidFrame[k];
            }

            std::array<double,2> stereo{};
            for(size_t ch=0; ch<2; ch++)
            {
                double sOut = ambiDec.play(ambiFrame,speakerAz[ch]);
                stereo[ch] = std::tanh(sOut);
            }

            for(size_t ch=0; ch<2; ch++)
                buffer[i*2 + ch] = float(stereo[ch]);
        }
    }

	const std::vector<BoidState9D>& getBoids() const { return boids; }
	glm::vec3 getBoidPosition(int i) const { return aggregator.getSpatialPos(i); }

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

    AdaptivePreset preset;
};
