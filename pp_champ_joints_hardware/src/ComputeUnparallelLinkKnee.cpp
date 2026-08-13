#include "ComputeUnparallelLinkKnee.h"
#include <iostream>
#include <iomanip>

bool ComputeUnparallelLinkKnee::compute(float L1Rad, float L2Rad, float* L3RadOut)
{
    if (L3RadOut == nullptr) return false;

    // L1Rad: 0 = down
    // internal math coordinate: 0 = +X, CCW+
    float l1Abs = L1Rad - PI * 0.5f;

    // L2Rad: relative angle from L1 direction
    float l2Abs = l1Abs + L2Rad;

    // green point: L1 endpoint
    float gx = L1 * std::cos(l1Abs);
    float gy = L1 * std::sin(l1Abs);

    // yellow point: midpoint of L2
    float yx = gx + L2midpoint * std::cos(l2Abs);
    float yy = gy + L2midpoint * std::sin(l2Abs);

    // red point: drive axis
    float rx = offsetX;
    float ry = offsetY;

    // red -> yellow
    float dx = yx - rx;
    float dy = yy - ry;
    float d = std::sqrt(dx * dx + dy * dy);

    if (d < EPS) return false;
    if (d > L3 + L4 + EPS) return false;
    if (d < std::fabs(L3 - L4) - EPS) return false;

    // Circle intersection:
    // center red, radius L3
    // center yellow, radius L4
    float a = (L3 * L3 - L4 * L4 + d * d) / (2.0f * d);
    float h2 = L3 * L3 - a * a;
    if (h2 < -EPS) return false;
    if (h2 < 0.0f) h2 = 0.0f;
    float h = std::sqrt(h2);
    float ux = dx / d;
    float uy = dy / d;
    float px = rx + a * ux;
    float py = ry + a * uy;

    // two possible blue points
    float b1x = px + (-uy) * h;
    float b1y = py + ( ux) * h;
    float b2x = px - (-uy) * h;
    float b2y = py - ( ux) * h;

    // L3 candidate angles.
    // Output basis: 0 = down, CCW+
    float rad1 = normalizeRad(std::atan2(b1y - ry, b1x - rx) + PI * 0.5f);
    float rad2 = normalizeRad(std::atan2(b2y - ry, b2x - rx) + PI * 0.5f);

    // L2 actual direction, converted to the same output basis.
    float l2DirRad = normalizeRad(l2Abs + PI * 0.5f);

    // Select the solution whose L3 direction is closer to the actual L2 direction.
    float diff1 = angleAbsDiff(rad1, l2DirRad);
    float diff2 = angleAbsDiff(rad2, l2DirRad);
    float L3Rad = (diff1 <= diff2) ? rad1 : rad2;
    *L3RadOut = normalizeRad(L3Rad);

    if(dbgOn){
        std::cout << std::internal << std::fixed << std::setprecision(1) << std::setw(6) << std::right;
        std::cout << "(L1Rad = " << rad2deg(L1Rad) << ", ";
        std::cout << "L2Rad = " << rad2deg(L2Rad) << ") ";
        std::cout << "L3RadOut = " << rad2deg(*L3RadOut) << std::endl;
    }
    return true;
}

float ComputeUnparallelLinkKnee::deg2rad(float deg)
{
    return deg * PI / 180.0f;
}

float ComputeUnparallelLinkKnee::rad2deg(float rad)
{
    return rad * 180.0f / PI;
}

float ComputeUnparallelLinkKnee::normalizeRad(float rad)
{
    while (rad > PI)  rad -= 2.0f * PI;
    while (rad < -PI) rad += 2.0f * PI;
    return rad;
}

float ComputeUnparallelLinkKnee::angleAbsDiff(float a, float b)
{
    return std::fabs(normalizeRad(a - b));
}

#if 0
// for Unit test
// g++ -o main_test_ComputeUnparallelLinkKnee ComputeUnparallelLinkKnee.cpp
int main(int argc, char *argv[])
{
    ComputeUnparallelLinkKnee culk;
    culk.dbgOn = true;
    culk.L1 = 110.0f;
    culk.L2 = 120.0f;
    culk.L3 = 50.0f;
    culk.L4 = 90.0f;
    culk.offsetX = 5.0f;
    culk.offsetY = -25.0f;
    float L1rad, L2rad, L3rad;

    L1rad = culk.deg2rad(45);
    L2rad = culk.deg2rad(-90);
    L3rad = -360;
    culk.compute(L1rad, L2rad, &L3rad);

    L1rad = culk.deg2rad(50);
    L2rad = culk.deg2rad(-95);
    L3rad = -360;
    culk.compute(L1rad, L2rad, &L3rad);

    L1rad = culk.deg2rad(30);
    L2rad = culk.deg2rad(-120);
    L3rad = -360;
    culk.compute(L1rad, L2rad, &L3rad);

    L1rad = culk.deg2rad(80);
    L2rad = culk.deg2rad(-75);
    L3rad = -360;
    culk.compute(L1rad, L2rad, &L3rad);

    L1rad = culk.deg2rad(30);
    L2rad = culk.deg2rad(-120);
    L3rad = -360;
    culk.compute(L1rad, L2rad, &L3rad);

    L1rad = culk.deg2rad(25);
    L2rad = culk.deg2rad(-50);
    L3rad = -360;
    culk.compute(L1rad, L2rad, &L3rad);
    return 0;
}
#endif

