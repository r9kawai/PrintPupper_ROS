#ifndef ComputeUnparallelLinkKnee_H
#define ComputeUnparallelLinkKnee_H
#include <cmath>

class ComputeUnparallelLinkKnee {
private:
    float PI  = std::acos(-1.0f);
    float EPS = 1.0e-6f;

public:
    bool dbgOn = false;
    float L1 = 110.0f;
    float L2 = 120.0f;
    float L2midpoint = 60.0f;
    float L3 = 50.0f;
    float L4 = 90.0f;
    float offsetX = 5.0f;
    float offsetY = -25.0f;

    bool compute(float L1Rad, float L2Rad, float* L3RadOut);
    float deg2rad(float deg);
    float rad2deg(float rad);

private:
    float normalizeRad(float rad);
    float angleAbsDiff(float a, float b);
};

#endif // ComputeUnparallelLinkKnee_H
