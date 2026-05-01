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

// --- Trigger behaviour ---
std::atomic<double> TRIGGER_MIX              {0.5};
std::atomic<double> TRIGGER_MIN_RATE         {10.0};
std::atomic<double> TRIGGER_MAX_RATE         {300.0};
std::atomic<double> TRIGGER_CURVE_EXPONENT   {0.7};
std::atomic<double> ABSOLUTE_DISTANCE_SCALE  {600.0};

// --- Density behaviour ---
std::atomic<double> DENSITY_EXP_SCALE        {2.0};

// --- Frequency mapping ---
std::atomic<double> FREQ_MIN                 {100.0};
std::atomic<double> FREQ_MAX                 {1000.0};
std::atomic<double> FREQ_DENSITY_INFLUENCE   {1.5};
std::atomic<double> FREQ_RADIAL_EXPONENT     {0.6};
std::atomic<double> FREQ_SMOOTHING           {0.995};

// --- Modal structure ---
std::atomic<double> MODE_FREQ_SPREAD         {0.01};
std::atomic<double> MODE_BW_BASE             {80.0};
std::atomic<double> MODE_BW_SPREAD           {0.2};
std::atomic<double> MODE_AMP_BASE            {0.3};
std::atomic<double> MODE_AMP_DECAY           {0.4};

// --- Spatial rendering ---
std::atomic<double> SPATIAL_DISTANCE_MIN     {0.05};
std::atomic<double> SPATIAL_DISTANCE_MAX     {1.0};
std::atomic<double> SPATIAL_GAIN_EXP         {3.0};

// --- Output stage ---
std::atomic<double> OUTPUT_TANH_DRIVE        {4.0};

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
		prevSpatialPos.resize(maxBoids, glm::vec3(0.f));
        densities.resize(maxBoids, 0.f);
        meanDistances.resize(maxBoids, 1.f);
    }

    void setSubspaces(const std::vector<Subspace>& s)
    {
        subspaces = s;
        normalizeWeights();
    }

    void update(const std::vector<Boid>& boids)
    {
        const int N = static_cast<int>(boids.size());
        if(N == 0) return;

		boidsRef = &boids;

        computeSpatialPositions(boids);
        computeDistancesAndDensity(N);
        globalMeanDistance = computeGlobalMean(N);
    }


	glm::vec3 getWrappedPos(int i) const
	{
		glm::vec3 p = spatialPos[i];

		p -= glm::round(p / 2.0f) * 2.0f;

		return p;
	}

	glm::vec3 getWrappedVelocity(int i) const
	{
		glm::vec3 pNow  = getWrappedPos(i);

		glm::vec3 pPrev = prevSpatialPos[i];
		pPrev -= glm::round(pPrev / 2.0f) * 2.0f;

		glm::vec3 v = pNow - pPrev;
		v -= glm::round(v / 2.0f) * 2.0f;

		return v;
	}

    float getDensity(int i) const { return densities[i]; }
    float getMeanDistance(int i) const { return meanDistances[i]; }
    double getGlobalMeanDistance() const { return globalMeanDistance; }
	glm::vec3 getNormalizedPos(int i) const { return (spatialPos[i] - normCenter) * normScale; }

private:
    std::vector<Subspace> subspaces;
    std::vector<glm::vec3> spatialPos;
	std::vector<glm::vec3> prevSpatialPos;
    std::vector<float> densities;
    std::vector<float> meanDistances;
	glm::vec3 normCenter = glm::vec3(0.0f);
	const std::vector<Boid>* boidsRef = nullptr;
	glm::vec3 smoothedCenter = glm::vec3(0.0f);
	float smoothedScale = 1.0f;
	float normScale = 1.0f;
    double globalMeanDistance = 1.0;
    float spatialSmooth = 0.25f;

    void normalizeWeights()
    {
        float sum = 0.f;
        for(const auto& s : subspaces) sum += s.weight;

		if(sum > 0.f)
            for(auto& s : subspaces) s.weight /= sum;
    }

	glm::vec3 extractPos(const Boid& b, const Subspace& s) const 
	{
		return {
			b.position[s.dims[0]],
			b.position[s.dims[1]],
			b.position[s.dims[2]]
		};
	}

    void computeSpatialPositions(const std::vector<Boid>& boids)
    {
		prevSpatialPos = spatialPos;

		glm::vec3 minP(FLT_MAX);
		glm::vec3 maxP(-FLT_MAX);
		glm::vec3 center(0.0f);

		for (size_t i = 0; i < boids.size(); i++)
		{
			minP = glm::min(minP, spatialPos[i]);
			maxP = glm::max(maxP, spatialPos[i]);
		}

		glm::vec3 extent = maxP - minP;
		float maxExtent = std::max({extent.x, extent.y, extent.z});

		float targetScale = (maxExtent > 0.0001f) ? (1.0f / maxExtent) : 1.0f;
		glm::vec3 targetCenter = (minP + maxP) * 0.5f;

		// --- SMOOTH (critical fix)
		float alpha = 0.05f; // small = stable, big = reactive

		smoothedCenter = glm::mix(smoothedCenter, targetCenter, alpha);
		smoothedScale  = glm::mix(smoothedScale,  targetScale,  alpha);

		normCenter = smoothedCenter;
		normScale  = smoothedScale;

		center /= (float)spatialPos.size();

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

                glm::vec3 d = spatialPos[i] - spatialPos[j];
				d -= glm::round(d / 2.0f) * 2.0f;
				float dist = sqrt(glm::dot(d, d) + 0.0001f);
				sum += dist;
                count++;
            }

            float meanDist = (count>0) ? sum / count : 1.f;
            meanDistances[i] = meanDist;
            densities[i] = std::exp(-meanDist * DENSITY_EXP_SCALE.load(std::memory_order_relaxed));
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

		std::vector<Subspace> presetSubspaces =
		{
			{ {0, 1, 2}, 1.f / 3.f },
			{ {3, 4, 5}, 1.f / 3.f },
			{ {6, 7, 8}, 1.f / 3.f }
		};

		aggregator.setSubspaces(presetSubspaces);

		modalBank2D.setup(Nmax, 8, SAMPLERATE);
		modalBank2D.initRandom();

		smoothedFreq.resize(Nmax, 0.0);
		densities.resize(Nmax, 0.0);
		triggerAccumulator.resize(Nmax, 0.0);
		triggerRate.resize(Nmax, 10.0);
		boidOutputs.resize(Nmax);

		allocateBuffers(Nmax, 8);

		computeModalParamsControl();
		bufferA = workingBuffer;
		bufferB = workingBuffer;

		activeBuffer.store(&bufferA);

		speakerAz = { -0.5 * PI, 0.5 * PI };
	}

	void updateBoids(const std::vector<Boid>& boidsIn)
	{
		boids = boidsIn;
		N = static_cast<int>(boids.size());

		if (N == 0) return;

		aggregator.update(boids);

		for (int i = 0; i < N; i++)
			densities[i] = aggregator.getDensity(i);

		computeModalParamsControl();

		ModalParamsBuffer* inactive = (activeBuffer.load(std::memory_order_relaxed) == &bufferA) ? &bufferB : &bufferA;

		*inactive = workingBuffer;
		activeBuffer.store(inactive, std::memory_order_release);

		globalMeanDistance.store(aggregator.getGlobalMeanDistance(), std::memory_order_release);
	}

	void audioOut(ofSoundBuffer& buffer) override
	{
		ModalParamsBuffer* params = activeBuffer.load(std::memory_order_acquire);

		applyModalParams(*params);

		double globalDist = globalMeanDistance.load(std::memory_order_acquire);

		const double mix = TRIGGER_MIX.load(std::memory_order_relaxed);
		const double maxRate = TRIGGER_MAX_RATE.load(std::memory_order_relaxed);
		const double minRate = TRIGGER_MIN_RATE.load(std::memory_order_relaxed);

		for (int j = 0; j < N; j++)
		{
			double meanDist = aggregator.getMeanDistance(j);

			double adaptive = meanDist / (globalDist + 1e-9);
			double absolute = meanDist / ABSOLUTE_DISTANCE_SCALE.load(std::memory_order_relaxed);

			double relative = mix * adaptive + (1.0 - mix) * absolute;

			double compression = std::clamp(1.0 - relative, 0.0, 1.0);

			double rate = minRate + (maxRate - minRate) * std::pow(compression, TRIGGER_CURVE_EXPONENT.load(std::memory_order_relaxed));

			triggerRate[j] = rate;
		}

		for(size_t i = 0; i < buffer.getNumFrames(); i++)
		{
			for (int j = 0; j < N; j++)
			{
				triggerAccumulator[j] += triggerRate[j] / SAMPLERATE;

				if (triggerAccumulator[j] >= 1.0)
				{
					triggerAccumulator[j] -= 1.0;

					modalBank2D.exciteSource(
						j,
						0.1 + (triggerRate[j] / maxRate)
					);
				}
			}

			modalBank2D.playMulti(boidOutputs);

			std::array<double,7> ambiFrame{};
			ambiFrame.fill(0.0);

			for(int j = 0; j < N; j++)
			{
				glm::vec3 p = aggregator.getNormalizedPos(j);

				double az   = std::atan2(p.z, p.x);
				double dist = std::clamp((double)glm::length(p), SPATIAL_DISTANCE_MIN.load(std::memory_order_relaxed), SPATIAL_DISTANCE_MAX.load(std::memory_order_relaxed));

				double gain = 1.0 / (1.0 + SPATIAL_GAIN_EXP * dist);

				auto frame = ambiEnc.play(boidOutputs[j] * gain, az, dist);

				for(int k = 0; k < 7; k++)
					ambiFrame[k] += frame[k];
			}

			for(int ch = 0; ch < 2; ch++)
			{
				double sOut = ambiDec.play(ambiFrame, speakerAz[ch]);

				buffer[i*2 + ch] = std::tanh(sOut) * OUTPUT_TANH_DRIVE.load(std::memory_order_relaxed);
			}
		}
	}

    const std::vector<Boid>& getBoids() const
    {
        return boids;
    }

    glm::vec3 getBoidPosition(int i) const
    {
        return aggregator.getWrappedPos(i);
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
		const double smooth = FREQ_SMOOTHING.load(std::memory_order_relaxed);

		for(int j = 0; j < N; j++)
		{
			// --- POSITION (normalized torus space)
			glm::vec3 pos = aggregator.getNormalizedPos(j);

			// --- CENTER (wrapped space, as you already do)
			glm::vec3 center(0.0);
			for (int k = 0; k < N; k++)
				center += aggregator.getWrappedPos(k);
			center /= (double)N;

			// *** TORUS-CORRECT DISTANCE
			glm::vec3 dCenter = pos - center;
			dCenter -= glm::round(dCenter / 2.0f) * 2.0f;
			double dist = glm::length(dCenter);

			// --- NORMALIZED DISTANCE
			double spatialScale = 0.5; // *** smaller because space is now [-1,1]
			double normDist = glm::clamp(dist / spatialScale, 0.0, 1.0);

			// --- BASE FREQUENCY
			double baseFreq =
				FREQ_MIN.load(std::memory_order_relaxed) +
				(FREQ_MAX.load(std::memory_order_relaxed) - FREQ_MIN.load(std::memory_order_relaxed))
				* normDist;

			baseFreq *= (1.0 + densities[j] * FREQ_DENSITY_INFLUENCE.load(std::memory_order_relaxed));

			baseFreq = std::clamp(baseFreq,
								   FREQ_MIN.load(std::memory_order_relaxed),
								   FREQ_MAX.load(std::memory_order_relaxed));

			if(smoothedFreq[j] <= 0.0)
				smoothedFreq[j] = baseFreq;
			else
				smoothedFreq[j] =
					smooth * smoothedFreq[j] +
					(1.0 - smooth) * baseFreq;

			// --- TORUS-CONSISTENT VELOCITY (*** NEW)
			glm::vec3 vel = aggregator.getWrappedVelocity(j);

			// --- MOTION / ENERGY (*** FIXED)
			double energy = glm::length(vel);   // instead of pos

			double spreadMod = MODE_FREQ_SPREAD.load(std::memory_order_relaxed);
			double bwMod     = MODE_BW_BASE.load(std::memory_order_relaxed);
			double decayMod  = MODE_AMP_DECAY.load(std::memory_order_relaxed);

			double chaos      = 1.0 + energy;
			double brightness = 1.0 + dist;   // *** spatial instead of energy
			double tilt       = 1.0 + energy;

			for(size_t m = 0; m < workingBuffer.perBoid[j].freq.size(); m++)
			{
				double modeIndex = (double)m;

				workingBuffer.perBoid[j].freq[m] =
					smoothedFreq[j] *
					(1.0 + spreadMod * modeIndex) *
					(1.0 + 0.02 * chaos * sin(modeIndex));

				workingBuffer.perBoid[j].bw[m] =
					bwMod * (1.0 + 0.3 * modeIndex);

				double baseAmp =
					MODE_AMP_BASE.load(std::memory_order_relaxed) /
					(1.0 + decayMod * modeIndex);

				double tiltShape = pow(tilt, -modeIndex * 0.5);
				double brightShape = pow(brightness, modeIndex * 0.3);

				workingBuffer.perBoid[j].amp[m] =
					baseAmp * tiltShape * brightShape;
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

    std::vector<Boid> boids;
	std::vector<double> boidOutputs;
    std::vector<double> smoothedFreq;
    std::vector<double> densities;
    std::vector<double> triggerAccumulator;
	std::vector<double> triggerRate;

    BoidAggregator aggregator;
    ModalBank2D modalBank2D;
    AmbiEncode2D ambiEnc;
    AmbiDecode2D ambiDec;

    std::array<double,2> speakerAz;

    ModalParamsBuffer bufferA;
    ModalParamsBuffer bufferB;
    ModalParamsBuffer workingBuffer;

    std::atomic<ModalParamsBuffer*> activeBuffer;
    std::atomic<double> globalMeanDistance {1.0};
};
