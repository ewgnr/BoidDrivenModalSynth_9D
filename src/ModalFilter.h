#pragma once
#include <cmath>

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

class ModalFilter
{
public:
    explicit ModalFilter(double pSampleRate = 44100.0)
        : sampleRate(pSampleRate) {}

    inline void init(double pFrq, double pQ, double pAmp)
    {
        frq = pFrq;
        q   = pQ;
        amp = pAmp;
        computeCoeffs();
    }

    inline void setParams(double pFrq, double pQ, double pAmp)
    {
        frq = pFrq;
        q   = pQ;
        amp = pAmp;
        computeCoeffs();
    }

    inline double play(double pIn)
    {
        const double denom = beta + alpha / (2.0 * q);
        if (denom <= 0.0)
            return 0.0;

        const double y = (-(1.0 - 2.0 * beta) * y1
                          + pIn
                          - (beta - alpha / (2.0 * q)) * y2) / denom;

        y2 = y1;
        y1 = y;

        return y * sampleRate / (4.0 * M_PI * frq) * amp;
    }

    inline void reset()
    {
        y1 = y2 = 0.0;
    }

    inline void setSampleRate(double pSampleRate)
    {
        sampleRate = pSampleRate;
        computeCoeffs();
    }

private:
    inline void computeCoeffs()
    {
        if (frq <= 0.0)
            return;

        const double omega = 2.0 * M_PI * frq;
        alpha = sampleRate / omega;
        beta  = alpha * alpha;
    }

    double y1 = 0.0;
    double y2 = 0.0;
    double alpha = 0.0;
    double beta = 0.0;
    double frq = 0.0;
    double q = 1.0;
    double amp = 0.0;
    double sampleRate;
};
