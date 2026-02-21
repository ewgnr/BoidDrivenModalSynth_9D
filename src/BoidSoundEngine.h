#pragma once

#include "ofMath.h"
#include "ofSoundStream.h"
#include "ModalBank2D.h"
#include "ambiEncode2DThirdOrder.h"
#include "ambiDecode2DThirdOrder.h"

#include <array>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <atomic>

constexpr double CHANNELS   = 2;
constexpr double SAMPLERATE = 44100;
constexpr double BUFFERSIZE = 1024;

#define DIMS_PER_BOID 9

struct BoidState9D
{
    int index = 0;
    std::array<float, DIMS_PER_BOID> dims{};
    std::array<float, DIMS_PER_BOID> velocity{};
};

struct Subspace
{
    std::array<int,3> dims;
    float weight = 1.f;
};

class BoidAggregator
{
public:
    void setup(int maxBoids)
    {
        spatialPos.resize(maxBoids, glm::vec3(0.f));
        densities.resize(maxBoids, 0.f);
        meanDistances.resize(maxBoids, 1.f);
    }

    void setSubspaces(const std::vector<Subspace>& s)
    {
        subspaces = s;
        normalizeWeights();
    }

    void update(const std::vector<BoidState9D>& boids)
    {
        const int N = static_cast<int>(boids.size());
        if(N == 0) return;

        computeSpatialPositions(boids);
        computeDistancesAndDensity(N);
        globalMeanDistance = computeGlobalMean(N);
    }

    const glm::vec3& getSpatialPos(int i) const { return spatialPos[i]; }
    float getDensity(int i) const { return densities[i]; }
    float getMeanDistance(int i) const { return meanDistances[i]; }
    double getGlobalMeanDistance() const { return globalMeanDistance; }

private:
    std::vector<Subspace> subspaces;
    std::vector<glm::vec3> spatialPos;
    std::vector<float> densities;
    std::vector<float> meanDistances;
    double globalMeanDistance = 1.0;
    float spatialSmooth = 0.96f;

    void normalizeWeights()
    {
        float sum = 0.f;
        for(const auto& s : subspaces) sum += s.weight;
        if(sum > 0.f)
            for(auto& s : subspaces) s.weight /= sum;
    }

    glm::vec3 extractPos(const BoidState9D& b, const Subspace& s) const
    {
        return {b.dims[s.dims[0]], b.dims[s.dims[1]], b.dims[s.dims[2]]};
    }

    void computeSpatialPositions(const std::vector<BoidState9D>& boids)
    {
        for(size_t i=0;i<boids.size();i++)
        {
            glm::vec3 agg(0.f);
            for(const auto& s : subspaces)
                agg += s.weight * extractPos(boids[i], s);

            spatialPos[i] = spatialSmooth * spatialPos[i] + (1.f - spatialSmooth) * agg;
        }
    }

    void computeDistancesAndDensity(int N)
    {
        for(int i=0;i<N;i++)
        {
            float sum = 0.f;
            int count = 0;

            for(int j=0;j<N;j++)
            {
                if(i==j) continue;
                sum += glm::length(spatialPos[i] - spatialPos[j]);
                count++;
            }

            float meanDist = (count>0) ? sum / count : 1.f;
            meanDistances[i] = meanDist;
            densities[i] = std::exp(-meanDist * 2.f);
        }
    }

    double computeGlobalMean(int N)
    {
        double sum = 0.0;
        int count = 0;

        for(int i=0;i<N;i++)
		{
            for(int j=i+1;j<N;j++)
            {
                sum += glm::length(spatialPos[i] - spatialPos[j]);
                count++;
            }
		}

        return (count>0) ? sum / count : 1.0;
    }
};

class AdaptiveBoidSoundEngine : public ofBaseSoundOutput
{
public:
    void setup(int maxBoids)
    {
        Nmax = maxBoids;
        aggregator.setup(Nmax);

        presetSubspaces = {
            { {0,1,2}, 1.f/3.f },
            { {3,4,5}, 1.f/3.f },
            { {6,7,8}, 1.f/3.f }
        };

        aggregator.setSubspaces(presetSubspaces);

        modalBank2D.setup(Nmax, 8, SAMPLERATE);
        modalBank2D.initRandom();

        smoothedFreq.resize(Nmax, 0.0);
        densities.resize(Nmax, 0.0);
        triggerAccumulator.resize(Nmax, 0.0);

        allocateBuffers(Nmax, 8);

        computeModalParamsControl();
        bufferA = workingBuffer;
        bufferB = workingBuffer;

        activeBuffer.store(&bufferA);

        speakerAz = { -0.5*PI, 0.5*PI };
    }

    void updateBoids(const std::vector<BoidState9D>& boidsIn)
    {
        boids = boidsIn;
        N = static_cast<int>(boids.size());

        if(N == 0) return;

        aggregator.update(boids);

        for(int i=0;i<N;i++)
            densities[i] = aggregator.getDensity(i);

        computeModalParamsControl();

        ModalParamsBuffer* inactive = (activeBuffer.load() == &bufferA) ? &bufferB : &bufferA;

        *inactive = workingBuffer;
        activeBuffer.store(inactive, std::memory_order_release);

        globalMeanDistance = aggregator.getGlobalMeanDistance();
    }

    void audioOut(ofSoundBuffer& buffer) override
    {
        ModalParamsBuffer* params = activeBuffer.load(std::memory_order_acquire);

        applyModalParams(*params);

        double globalDist = globalMeanDistance;

        for(size_t i=0;i<buffer.getNumFrames();i++)
        {
			for (int j = 0; j < N; j++)
			{
				double meanDist = aggregator.getMeanDistance(j);

			    double adaptive = meanDist / (globalDist + 1e-9);

			    double absolute = meanDist / 600.0;

			    double mix = 0.5;  

			    double relative = mix * adaptive + (1.0 - mix) * absolute;

			    double compression = std::clamp(1.0 - relative, 0.0, 1.0);

			    double rate = 10.0 + (500.0 - 10.0) * pow(compression, 0.7);

			    triggerAccumulator[j] += rate / SAMPLERATE;

			    if (triggerAccumulator[j] >= 1.0)
			    {
				    triggerAccumulator[j] -= 1.0;

				    modalBank2D.exciteSource(j, 0.1 + compression);
			    }
		    }

            auto boidOutputs = modalBank2D.playMulti();

            std::array<double,7> ambiFrame{};
            ambiFrame.fill(0.0);

            for(int j=0;j<N;j++)
            {
                glm::vec3 p = aggregator.getSpatialPos(j);
                double az   = std::atan2(p.z,p.x);
                double dist = std::clamp((double)glm::length(p),0.05,1.0);

                double gain = std::exp(-3.0*dist);

                auto frame = ambiEnc.play(boidOutputs[j]*gain, az, dist);

                for(int k=0;k<7;k++)
                    ambiFrame[k] += frame[k];
            }

            for(int ch=0;ch<2;ch++)
            {
                 double sOut = ambiDec.play(ambiFrame, speakerAz[ch]);

                 buffer[i*2+ch] = std::tanh(sOut) * 4.0;
            }
        }
    }

    const std::vector<BoidState9D>& getBoids() const
    {
        return boids;
    }

    glm::vec3 getBoidPosition(int i) const
    {
        return aggregator.getSpatialPos(i);
    }

private:

    struct ModalParams
    {
        std::vector<double> freq;
        std::vector<double> bw;
        std::vector<double> amp;
    };

    struct ModalParamsBuffer
    {
        std::vector<ModalParams> perBoid;
    };

    void allocateBuffers(int boids, int modes)
    {
        bufferA.perBoid.resize(boids);
        bufferB.perBoid.resize(boids);
        workingBuffer.perBoid.resize(boids);

        for(int j=0;j<boids;j++)
        {
            bufferA.perBoid[j].freq.resize(modes);
            bufferA.perBoid[j].bw.resize(modes);
            bufferA.perBoid[j].amp.resize(modes);

            bufferB.perBoid[j] = bufferA.perBoid[j];
            workingBuffer.perBoid[j] = bufferA.perBoid[j];
        }
    }

    void computeModalParamsControl()
    {
        const double smooth = 0.995;

        for(int j=0;j<N;j++)
        {
            glm::vec3 r(boids[j].dims[0], boids[j].dims[1], boids[j].dims[2]);

            if(glm::length(r)>1e-6)
                r = glm::normalize(r);

            double radialHeight = 0.5*(r.y+1.0);

            double radialCurve = pow(radialHeight,0.6);

            double targetFreq = 100.0 + (1000.0-100.0) * radialCurve * (1.0 + densities[j]*1.5);

            targetFreq = std::clamp(targetFreq,100.0,1000.0);

            if(smoothedFreq[j] <= 0.0)
                smoothedFreq[j] = targetFreq;
            else
                smoothedFreq[j] = smooth*smoothedFreq[j] + (1.0-smooth)*targetFreq;

            for(size_t m=0; m<workingBuffer.perBoid[j].freq.size(); m++)
            {
                workingBuffer.perBoid[j].freq[m] = smoothedFreq[j]*(1.0+0.01*m);
                workingBuffer.perBoid[j].bw[m] = 80.0*(1.0+0.2*m);
                workingBuffer.perBoid[j].amp[m] = 0.3/(1.0+0.4*m);
            }
        }
    }

    void applyModalParams(const ModalParamsBuffer& params)
    {
        for(int j=0;j<N;j++)
        {
            for(size_t m=0; m<params.perBoid[j].freq.size(); m++)
            {
                modalBank2D.setParams(j, m, params.perBoid[j].freq[m], params.perBoid[j].bw[m], params.perBoid[j].amp[m]);
            }
        }
    }

private:
    int N = 0;
    int Nmax = 0;

    std::vector<BoidState9D> boids;
    std::vector<double> smoothedFreq;
    std::vector<double> densities;
    std::vector<double> triggerAccumulator;

    BoidAggregator aggregator;
    ModalBank2D modalBank2D;
    AmbiEncode2D ambiEnc;
    AmbiDecode2D ambiDec;

    std::vector<Subspace> presetSubspaces;
    std::array<double,2> speakerAz;

    ModalParamsBuffer bufferA;
    ModalParamsBuffer bufferB;
    ModalParamsBuffer workingBuffer;

    std::atomic<ModalParamsBuffer*> activeBuffer;
    double globalMeanDistance = 1.0;
};
