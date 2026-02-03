#pragma once
#include <array>
#include <cmath>

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

class AmbiEncode2D
{
public:
	AmbiEncode2D()
		: output {} { }

	inline std::array<double, 7> play(double pAudioIn, double pAzimuthDeg, double pDistance)
	{
		double az = M_PI * pAzimuthDeg / 180.0;
		if (pDistance < 0) az += M_PI;
		double dist = std::abs(pDistance) + 0.0001;

		double gainW = std::atan(dist * M_PI * 0.5) / (dist * M_PI * 0.5);
		double gainHO = 1.0 - std::exp(-dist);

		output[0] = gainW * pAudioIn;
		output[1] = std::cos(1 * az) * gainHO * gainW * pAudioIn;
		output[2] = std::sin(1 * az) * gainHO * gainW * pAudioIn;
		output[3] = std::cos(2 * az) * gainHO * gainW * pAudioIn;
		output[4] = std::sin(2 * az) * gainHO * gainW * pAudioIn;
		output[5] = std::cos(3 * az) * gainHO * gainW * pAudioIn;
		output[6] = std::sin(3 * az) * gainHO * gainW * pAudioIn;

		return output;
	}

private:
	std::array<double, 7> output;
};
