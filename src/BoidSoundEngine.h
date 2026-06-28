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

#include "ParamRegistry.h"

constexpr std::size_t CHANNELS   = 2;
constexpr std::size_t SAMPLERATE = 44100;
constexpr std::size_t BUFFERSIZE = 1024;

#define DIMS_PER_BOID 9

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

	glm::vec3 getNormalizedWrappedPos(int i) const
	{
		glm::vec3 p = getWrappedPos(i);
		return (p - normCenter) * normScale;
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

		for(size_t i = 0; i < boids.size(); i++)
		{
			glm::vec3 agg(0.f);

			for(const auto& s : subspaces)
				agg += s.weight * extractPos(boids[i], s);

			spatialPos[i] = spatialSmooth * spatialPos[i] + (1.f - spatialSmooth) * agg;
		}

		glm::vec3 minP(FLT_MAX);
		glm::vec3 maxP(-FLT_MAX);

		for(size_t i = 0; i < boids.size(); i++)
		{
			minP = glm::min(minP, spatialPos[i]);
			maxP = glm::max(maxP, spatialPos[i]);
		}

		glm::vec3 extent = maxP - minP;

		float maxExtent = std::max({extent.x, extent.y, extent.z});

		float targetScale = (maxExtent > 1e-5f) ? (1.0f / maxExtent) : 1.0f;

		glm::vec3 targetCenter = (minP + maxP) * 0.5f;

		float motion = glm::length(targetCenter - smoothedCenter);

		float alpha = glm::clamp(0.005f + motion * 0.1f, 0.005f, 0.2f);

		smoothedCenter = glm::mix(smoothedCenter, targetCenter, alpha);

		smoothedScale = glm::mix(smoothedScale, targetScale, alpha);

		normCenter = smoothedCenter;
		normScale  = smoothedScale;
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
            densities[i] = std::exp(-meanDist * SPATIAL_DENSITY_EXP_SCALE.load(std::memory_order_relaxed));
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
                glm::vec3 d = spatialPos[i] - spatialPos[j];
				d -= glm::round(d / 2.0f) * 2.0f;

				sum += glm::length(d);

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
		smoothedTriggerRate.resize(Nmax, 10.0);
		boidOutputs.resize(Nmax);

		allocateBuffers(Nmax, 8);

		spatialA.wrappedPos.resize(Nmax);
		spatialA.normalizedPos.resize(Nmax);
		spatialA.velocity.resize(Nmax);
		spatialA.density.resize(Nmax);
		spatialA.meanDistance.resize(Nmax);

		spatialB = spatialA;

		activeSpatial.store(&spatialA, std::memory_order_release);

		computeModalParamsControl();
		bufferA = workingBuffer;
		bufferB = workingBuffer;

		activeBuffer.store(&bufferA);

		speakerAz = { -0.5 * PI, 0.5 * PI };

		auto initFrame = [&](SpatialFrame& f)
		{
			f.wrappedPos.resize(Nmax);
			f.normalizedPos.resize(Nmax);
			f.velocity.resize(Nmax);
			f.density.resize(Nmax);
			f.meanDistance.resize(Nmax);
		};

		initFrame(spatialA);
		initFrame(spatialB);
	}

	void updateBoids(const std::vector<Boid>& boidsIn)
	{
		boids = boidsIn;
		N = static_cast<int>(boids.size());

		if (N == 0) return;

		aggregator.update(boids);

		SpatialFrame* inactiveSpatial =
			(activeSpatial.load(std::memory_order_relaxed) == &spatialA)
			? &spatialB
			: &spatialA;

		for(int i = 0; i < N; i++)
		{
			inactiveSpatial->wrappedPos[i] = aggregator.getWrappedPos(i);
			inactiveSpatial->normalizedPos[i] = aggregator.getNormalizedWrappedPos(i);
			inactiveSpatial->velocity[i] = aggregator.getWrappedVelocity(i);
			inactiveSpatial->density[i] = aggregator.getDensity(i);
			inactiveSpatial->meanDistance[i] = aggregator.getMeanDistance(i);
		}

		inactiveSpatial->globalMeanDistance = aggregator.getGlobalMeanDistance();

		activeSpatial.store(inactiveSpatial, std::memory_order_release);

		for (int i = 0; i < N; i++)
			densities[i] = aggregator.getDensity(i);

		computeModalParamsControl();

		ModalParamsBuffer* inactiveModal =
			(activeBuffer.load(std::memory_order_relaxed) == &bufferA)
			? &bufferB
			: &bufferA;

		*inactiveModal = workingBuffer;

		activeBuffer.store(inactiveModal, std::memory_order_release);

		globalMeanDistance.store(aggregator.getGlobalMeanDistance(), std::memory_order_release);
	}

	void audioOut(ofSoundBuffer& buffer) override
	{
		SpatialFrame* spatial = activeSpatial.load(std::memory_order_acquire);

		ModalParamsBuffer* params = activeBuffer.load(std::memory_order_acquire);

		applyModalParams(*params);

		double globalDist = spatial->globalMeanDistance;

		const double mix = TRIGGER_MIX.load(std::memory_order_relaxed);
		const double maxRate = TRIGGER_MAX_RATE.load(std::memory_order_relaxed);
		const double minRate = TRIGGER_MIN_RATE.load(std::memory_order_relaxed);

		for (int j = 0; j < N; j++)
		{
			double meanDist = spatial->meanDistance[j];

			double adaptive = meanDist / (globalDist + 1e-9);
			double absolute = meanDist / ABSOLUTE_DISTANCE_SCALE.load(std::memory_order_relaxed);

			double relative = mix * adaptive + (1.0 - mix) * absolute;

			double compression = std::clamp(1.0 - relative, 0.0, 1.0);

			double activityGain = TRIGGER_ACTIVITY_GAIN.load(std::memory_order_relaxed);

			double rate =
				minRate +
				(maxRate - minRate) *
				std::pow(compression,
						 TRIGGER_CURVE_EXPONENT.load(std::memory_order_relaxed));

			rate *= activityGain;

			smoothedTriggerRate[j] =
				0.98 * smoothedTriggerRate[j]
				+ 0.02 * rate;

			triggerRate[j] = smoothedTriggerRate[j];
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
				glm::vec3 p = spatial->normalizedPos[j];

				double width = SPATIAL_WIDTH.load(std::memory_order_relaxed);
				double az = std::atan2(p.z, p.x);
				double widthClamped = std::clamp(width, 0.2, 3.0);
				az = std::tanh(az * widthClamped) * M_PI;

				double dist = std::clamp((double)glm::length(p), SPATIAL_DISTANCE_MIN.load(std::memory_order_relaxed), SPATIAL_DISTANCE_MAX.load(std::memory_order_relaxed));

				double gain = 1.0 / (1.0 + SPATIAL_GAIN_EXP * dist);

				auto frame = ambiEnc.play(boidOutputs[j] * gain, az, dist);

				for(int k = 0; k < 7; k++)
					ambiFrame[k] += frame[k];
			}

			for(int ch = 0; ch < 2; ch++)
			{
				double sOut = ambiDec.play(ambiFrame, speakerAz[ch]);

				double drive = OUTPUT_TANH_DRIVE.load(std::memory_order_relaxed);
				double master = OUTPUT_MASTER_GAIN.load(std::memory_order_relaxed);

				double processed = std::tanh(sOut * drive);

				buffer[i*2 + ch] = processed * master;
			}
		}
	}

    const std::vector<Boid>& getBoids() const
    {
        return boids;
    }

	glm::vec3 getBoidPosition(int i) const
	{
		SpatialFrame* spatial = activeSpatial.load(std::memory_order_acquire);

		return spatial->normalizedPos[i];
	}

private:

	struct SpatialFrame
	{
		std::vector<glm::vec3> wrappedPos;
		std::vector<glm::vec3> normalizedPos;
		std::vector<glm::vec3> velocity;

		std::vector<float> density;
		std::vector<float> meanDistance;

		double globalMeanDistance = 1.0;
	};

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

		// --- CENTER (wrapped space, as you already do)
		glm::vec3 center(0.0);
		for (int k = 0; k < N; k++)
			center += aggregator.getNormalizedWrappedPos(k);
		center /= (double)N;

		for(int j = 0; j < N; j++)
		{
			// --- POSITION (normalized torus space)
			glm::vec3 pos = aggregator.getNormalizedWrappedPos(j);

			// *** TORUS-CORRECT DISTANCE
			glm::vec3 dCenter = pos - center;
			dCenter -= glm::round(dCenter / 2.0f) * 2.0f;
			double dist = glm::length(dCenter);

			// --- NORMALIZED DISTANCE
			double spatialScale = 0.5; // *** smaller because space is now [-1,1]
			double normDist = glm::clamp(dist / spatialScale, 0.0, 1.0);

			// --- BASE FREQUENCY
			double t = pow(normDist, FREQ_RADIAL_EXPONENT.load(std::memory_order_relaxed));

			double transpose = FREQ_TRANSPOSE.load(std::memory_order_relaxed);

			double fmin = FREQ_MIN.load(std::memory_order_relaxed);
			double fmax = FREQ_MAX.load(std::memory_order_relaxed);

			// --- log space mapping (your system already lives here)
			double logMin = std::log(fmin);
			double logMax = std::log(fmax);

			// base mapping
			double baseFreq = fmin * std::pow(fmax / fmin, t);

			// --- convert to normalized log space
			double freqNorm = (std::log(baseFreq) - logMin) / (logMax - logMin);

			// --- apply transpose in perceptual domain
			double transposeShift = std::log(transpose) / (logMax - logMin);

			freqNorm += transposeShift;

			// --- keep stable bounds
			freqNorm = std::clamp(freqNorm, 0.0, 1.0);

			// --- back to Hz
			baseFreq = std::exp(logMin + freqNorm * (logMax - logMin));
			baseFreq *= (1.0 + densities[j] * FREQ_DENSITY_INFLUENCE.load(std::memory_order_relaxed));
			baseFreq = std::clamp(baseFreq, FREQ_MIN.load(std::memory_order_relaxed), FREQ_MAX.load(std::memory_order_relaxed));

			if(smoothedFreq[j] <= 0.0)
				smoothedFreq[j] = baseFreq;
			else
				smoothedFreq[j] =
					(1.0 - smooth) * smoothedFreq[j] +
					smooth * baseFreq;

			// --- TORUS-CONSISTENT VELOCITY
			glm::vec3 vel = aggregator.getWrappedVelocity(j);

			// --- MOTION / ENERGY
			double energy = glm::length(vel);

			double spreadMod = MODE_FREQ_SPREAD.load(std::memory_order_relaxed);
			double bwMod     = MODE_BW_BASE.load(std::memory_order_relaxed);
			double decayMod  = MODE_AMP_DECAY.load(std::memory_order_relaxed);

			double chaos      = 1.0 + energy;
			double brightness = 1.0 + dist;
			double tiltEnergy = 1.0 + energy;

			for(size_t m = 0; m < workingBuffer.perBoid[j].freq.size(); m++)
			{
				double modeIndex = (double)m;

				workingBuffer.perBoid[j].freq[m] = smoothedFreq[j] * (1.0 + spreadMod * modeIndex) * (1.0 + 0.02 * chaos * sin(modeIndex));

				workingBuffer.perBoid[j].bw[m] = bwMod * (1.0 + 0.3 * modeIndex);

				double baseAmp = MODE_AMP_BASE.load(std::memory_order_relaxed) / (1.0 + decayMod * modeIndex);

				double tiltShape = std::pow(tiltEnergy, -modeIndex * 0.5);

				double brightnessParam = MODE_BRIGHTNESS.load(std::memory_order_relaxed);
				double brightnessClamped = std::clamp(brightnessParam, 0.1, 3.0);

				double brightShape = std::pow(brightnessClamped, modeIndex * 0.5);

				workingBuffer.perBoid[j].amp[m] = baseAmp * tiltShape * brightShape;
			}
		}
	}

	void applyModalParams(const ModalParamsBuffer& params)
	{
		if(N <= 0) return;

		const int boidsPerBuffer = 2;

		for(int n = 0; n < boidsPerBuffer; n++)
		{
			int j = (modalUpdateOffset + n) % N;

			for(size_t m = 0; m < params.perBoid[j].freq.size(); m++)
			{
				modalBank2D.setParams(
					j,
					m,
					params.perBoid[j].freq[m],
					params.perBoid[j].bw[m],
					params.perBoid[j].amp[m]
				);
			}
		}

		modalUpdateOffset =
			(modalUpdateOffset + boidsPerBuffer) % N;
	}

private:
    int N = 0;
    int Nmax = 0;
	int modalUpdateOffset = 0;

    std::vector<Boid> boids;
	std::vector<double> boidOutputs;
    std::vector<double> smoothedFreq;
    std::vector<double> densities;
    std::vector<double> triggerAccumulator;
	std::vector<double> triggerRate;
	std::vector<double> smoothedTriggerRate;

    BoidAggregator aggregator;
    ModalBank2D modalBank2D;
    AmbiEncode2D ambiEnc;
    AmbiDecode2D ambiDec;

    std::array<double,2> speakerAz;

    ModalParamsBuffer bufferA;
    ModalParamsBuffer bufferB;
    ModalParamsBuffer workingBuffer;

	SpatialFrame spatialA;
	SpatialFrame spatialB;
	std::atomic<SpatialFrame*> activeSpatial;

    std::atomic<ModalParamsBuffer*> activeBuffer;
    std::atomic<double> globalMeanDistance {1.0};
};
