#pragma once
#include <cmath>
#include <algorithm>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

struct BoidSpatial2D
{
    double azimuthRad = 0.0;  // [-π, π]
    double distance   = 1.0;  // normalized [0, 1]
};

class BoidToAmbi2D
{
public:
    explicit BoidToAmbi2D(double pMaxRadius = 300.0)
        : maxRadius(pMaxRadius) {}

    inline BoidSpatial2D map(double x, double y, double z)
    {
        BoidSpatial2D out;

        // -------- azimuth (XZ plane) --------
        out.azimuthRad = std::atan2(x, z); // front = +z

        // -------- distance --------
        const double d =
            std::sqrt(x * x + y * y + z * z);

        out.distance =
            std::clamp(d / maxRadius, 0.0, 1.0);

        return out;
    }

    inline void setMaxRadius(double r)
    {
        maxRadius = std::max(r, 1.0);
    }

private:
    double maxRadius;
};
