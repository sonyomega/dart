#include <iomanip>

#include "math/LieGroup.h"

namespace dart {
namespace math {

Vec3 Rotate(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v._v[0] + T._T[3] * v._v[1] + T._T[6] * v._v[2],
                T._T[1] * v._v[0] + T._T[4] * v._v[1] + T._T[7] * v._v[2],
                T._T[2] * v._v[0] + T._T[5] * v._v[1] + T._T[8] * v._v[2]);
}

Vec3 InvRotate(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

Vec3 operator*(double d, const Vec3& v)
{
    return Vec3(d * v._v[0], d * v._v[1], d * v._v[2]);
}

double Norm(const Vec3& v)
{
    return sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
}

Vec3 Normalize(const Vec3& v)
{
    double mag = sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
        return Vec3(SCALAR_0, SCALAR_0, SCALAR_1);

    mag = SCALAR_1 / mag;
    return Vec3(mag * v._v[0], mag * v._v[1], mag * v._v[2]);
}

Vec3 MinusLinearAd(const Vec3& p, const se3& s)
{
    return Vec3(p._v[2] * s._w[1] - p._v[1] * s._w[2] + s._w[3],
                p._v[0] * s._w[2] - p._v[2] * s._w[0] + s._w[4],
                p._v[1] * s._w[0] - p._v[0] * s._w[1] + s._w[5]);
}

Vec3 InvAd(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

Vec3 iEulerXYZ(const SE3& T)
{
    return Vec3(atan2(-T._T[7], T._T[8]),
                atan2( T._T[6], sqrt(T._T[7] * T._T[7] + T._T[8] * T._T[8])),
                atan2(-T._T[3], T._T[0]));
}

Vec3 iEulerZYX(const SE3& T)
{
    return Vec3(atan2( T._T[1], T._T[0]),
                atan2(-T._T[2], sqrt(T._T[0] * T._T[0] + T._T[1] * T._T[1])),
                atan2( T._T[5], T._T[8]));
}

Vec3 iEulerZYZ(const SE3& T)
{
    return Vec3(atan2(T._T[7], T._T[6]),
                atan2(sqrt(T._T[2] * T._T[2] + T._T[5] * T._T[5]), T._T[8]),
                atan2(T._T[5], -T._T[2]));
}

Vec3 iEulerZXY(const SE3& T)
{
    return Vec3(atan2(-T._T[3], T._T[4]),
                atan2( T._T[5], sqrt(T._T[3]*T._T[3]+T._T[4]*T._T[4])),
                atan2(-T._T[2], T._T[8]));
}

Vec3 ad(const Vec3& s1, const se3& s2)
{
    return Vec3(s2._w[2] * s1._v[1] - s2._w[1] * s1._v[2],
                s2._w[0] * s1._v[2] - s2._w[2] * s1._v[0],
                s2._w[1] * s1._v[0] - s2._w[0] * s1._v[1]);
}

se3 operator*(double d, const se3& s)
{
    return se3(d * s._w[0], d * s._w[1], d * s._w[2],
               d * s._w[3], d * s._w[4], d * s._w[5]);
}

double operator*(const dse3& t, const se3& s)
{
    return (t._m[0] * s._w[0] + t._m[1] * s._w[1] + t._m[2] * s._w[2]
          + t._m[3] * s._w[3] + t._m[4] * s._w[4] + t._m[5] * s._w[5]);
}

double operator*(const dse3& t, const Axis& s)
{
    return (t._m[0] * s._v[0] + t._m[1] * s._v[1] + t._m[2] * s._v[2]);
}

se3 Log(const SE3& T)
{
    //--------------------------------------------------------------------------
    // T = (R, p) = exp([w, v]), t = ||w||
    // v = beta * p + gamma * w + 1 / 2 * cross(p, w)
    //    , beta = t * (1 + cos(t)) / (2 * sin(t)), gamma = <w, p> * (1 - beta) / t^2
    //--------------------------------------------------------------------------
    double theta = std::acos(std::max(std::min(SCALAR_1_2 * (T._T[0] + T._T[4] + T._T[8] - SCALAR_1), SCALAR_1), -SCALAR_1));
    double alpha;
    double beta;
    double gamma;

    if ( theta > M_PI - LIE_EPS )
    {
        const double c1 = 0.10132118364234;		// 1 / pi^2
        const double c2 = 0.01507440267955;		// 1 / 4 / pi - 2 / pi^3
        const double c3 = 0.00546765085347;		// 3 / pi^4 - 1 / 4 / pi^2

        double phi = M_PI - theta;
        double delta = SCALAR_1_2 + SCALAR_1_8 * phi * phi;

        double w[] = {	T._T[5] > T._T[7] ? theta * sqrt(SCALAR_1 + (T._T[0] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T._T[0] - SCALAR_1) * delta),
                        T._T[6] > T._T[2] ? theta * sqrt(SCALAR_1 + (T._T[4] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T._T[4] - SCALAR_1) * delta),
                        T._T[1] > T._T[3] ? theta * sqrt(SCALAR_1 + (T._T[8] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T._T[8] - SCALAR_1) * delta) };

        beta = SCALAR_1_4 * theta * (M_PI - theta);
        gamma = (w[0] * T._T[9] + w[1] * T._T[10] + w[2] * T._T[11]) * (c1 -  c2 * phi + c3 * phi * phi);

        return se3(	w[0], w[1], w[2],
                    beta * T._T[ 9] - SCALAR_1_2 * (w[1] * T._T[11] - w[2] * T._T[10]) + gamma * w[0],
                    beta * T._T[10] - SCALAR_1_2 * (w[2] * T._T[ 9] - w[0] * T._T[11]) + gamma * w[1],
                    beta * T._T[11] - SCALAR_1_2 * (w[0] * T._T[10] - w[1] * T._T[ 9]) + gamma * w[2]);
    } else
    {
        if ( theta > LIE_EPS )
        {
            alpha = SCALAR_1_2 * theta / sin(theta);
            beta = (SCALAR_1 + cos(theta)) * alpha;
            gamma = (SCALAR_1 - beta) / theta / theta;
        } else
        {
            alpha = SCALAR_1_2 + SCALAR_1_12 * theta * theta;
            beta = SCALAR_1 - SCALAR_1_12 * theta * theta;
            gamma = SCALAR_1_12 + SCALAR_1_720 * theta * theta;
        }

        double w[] = { alpha * (T._T[5] - T._T[7]), alpha * (T._T[6] - T._T[2]), alpha * (T._T[1] - T._T[3]) };
        gamma *= w[0] * T._T[9] + w[1] * T._T[10] + w[2] * T._T[11];

        return se3(	w[0], w[1], w[2],
                    beta * T._T[ 9] + SCALAR_1_2 * (w[2] * T._T[10] - w[1] * T._T[11]) + gamma * w[0],
                    beta * T._T[10] + SCALAR_1_2 * (w[0] * T._T[11] - w[2] * T._T[ 9]) + gamma * w[1],
                    beta * T._T[11] + SCALAR_1_2 * (w[1] * T._T[ 9] - w[0] * T._T[10]) + gamma * w[2]);
    }
}

Axis LogR(const SE3& T)
{
    double theta = std::acos(std::max(std::min(SCALAR_1_2 * (T._T[0] + T._T[4] + T._T[8] - SCALAR_1), SCALAR_1), -SCALAR_1)), alpha;

    if ( theta > M_PI - LIE_EPS )
    {
        double delta = SCALAR_1_2 + SCALAR_1_8 * (M_PI - theta) * (M_PI - theta);

        return Axis(T._T[5] > T._T[7] ? theta * sqrt(SCALAR_1 + (T._T[0] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T._T[0] - SCALAR_1) * delta),
                    T._T[6] > T._T[2] ? theta * sqrt(SCALAR_1 + (T._T[4] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T._T[4] - SCALAR_1) * delta),
                    T._T[1] > T._T[3] ? theta * sqrt(SCALAR_1 + (T._T[8] - SCALAR_1) * delta) : -theta * sqrt(SCALAR_1 + (T._T[8] - SCALAR_1) * delta));
    } else
    {
        if ( theta > LIE_EPS )
            alpha = SCALAR_1_2 * theta / sin(theta);
        else
            alpha = SCALAR_1_2 + SCALAR_1_12 * theta * theta;

        return Axis(alpha * (T._T[5] - T._T[7]), alpha * (T._T[6] - T._T[2]), alpha * (T._T[1] - T._T[3]));
    }
}

// re = T * s * Inv(T)
se3 Ad(const SE3& T, const se3& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------
    double Rw[3] = { T._T[0] * s._w[0] + T._T[3] * s._w[1] + T._T[6] * s._w[2],
                     T._T[1] * s._w[0] + T._T[4] * s._w[1] + T._T[7] * s._w[2],
                     T._T[2] * s._w[0] + T._T[5] * s._w[1] + T._T[8] * s._w[2] };
    return se3(	Rw[0], Rw[1], Rw[2],
                T._T[10] * Rw[2] - T._T[11] * Rw[1] + T._T[0] * s._w[3] + T._T[3] * s._w[4] + T._T[6] * s._w[5],
                T._T[11] * Rw[0] - T._T[ 9] * Rw[2] + T._T[1] * s._w[3] + T._T[4] * s._w[4] + T._T[7] * s._w[5],
                T._T[ 9] * Rw[1] - T._T[10] * Rw[0] + T._T[2] * s._w[3] + T._T[5] * s._w[4] + T._T[8] * s._w[5]);
}

se3 Ad(const SE3& T, const Axis& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w
    //--------------------------------------------------------------------------
    double Rw[3] = { T._T[0] * s._v[0] + T._T[3] * s._v[1] + T._T[6] * s._v[2],
                     T._T[1] * s._v[0] + T._T[4] * s._v[1] + T._T[7] * s._v[2],
                     T._T[2] * s._v[0] + T._T[5] * s._v[1] + T._T[8] * s._v[2] };
    return se3(	Rw[0], Rw[1], Rw[2],
                T._T[10] * Rw[2] - T._T[11] * Rw[1],
                T._T[11] * Rw[0] - T._T[9] * Rw[2],
                T._T[9] * Rw[1] - T._T[10] * Rw[0]);
}

se3 Ad(const SE3& T, const Vec3& v)
{
    //--------------------------------------------------------------------------
    // w' = 0
    // v' = R * v
    //--------------------------------------------------------------------------
    return se3(	SCALAR_0, SCALAR_0, SCALAR_0,
                T._T[0] * v._v[0] + T._T[3] * v._v[1] + T._T[6] * v._v[2],
                T._T[1] * v._v[0] + T._T[4] * v._v[1] + T._T[7] * v._v[2],
                T._T[2] * v._v[0] + T._T[5] * v._v[1] + T._T[8] * v._v[2]);
}

Jacobian Ad(const SE3& T, const Jacobian& J)
{
    Jacobian AdTJ(J.mJ.size());

    for (int i = 0; i < J.mJ.size(); ++i)
    {
        AdTJ[i] = Ad(T, J.mJ[i]);
    }

    return AdTJ;
}

se3 AdR(const SE3& T, const se3& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = R * v
    //--------------------------------------------------------------------------
    return se3(	T._T[0] * s._w[0] + T._T[3] * s._w[1] + T._T[6] * s._w[2],
                T._T[1] * s._w[0] + T._T[4] * s._w[1] + T._T[7] * s._w[2],
                T._T[2] * s._w[0] + T._T[5] * s._w[1] + T._T[8] * s._w[2],

                T._T[0] * s._w[3] + T._T[3] * s._w[4] + T._T[6] * s._w[5],
                T._T[1] * s._w[3] + T._T[4] * s._w[4] + T._T[7] * s._w[5],
                T._T[2] * s._w[3] + T._T[5] * s._w[4] + T._T[8] * s._w[5]);
}

se3 Ad(const Vec3& p, const se3& s)
{
    //--------------------------------------------------------------------------
    // w' = w
    // v' = p x w + v
    //--------------------------------------------------------------------------
    return se3(s._w[0],
               s._w[1],
               s._w[2],

               p._v[1] * s._w[2] - p._v[2] * s._w[1] + s._w[3],
               p._v[2] * s._w[0] - p._v[0] * s._w[2] + s._w[4],
               p._v[0] * s._w[1] - p._v[1] * s._w[0] + s._w[5]);
}

Jacobian AdR(const SE3& T, const Jacobian& J)
{
    Jacobian AdTJ(J.mJ.size());

    for (int i = 0; i < J.mJ.size(); ++i)
    {
        AdTJ[i] = AdR(T, J.mJ[i]);
    }

    return AdTJ;
}

// re = Inv(T) * s * T
se3 InvAd(const SE3& T, const se3& s)
{
    double tmp[3] = {	s._w[3] + s._w[1] * T._T[11] - s._w[2] * T._T[10],
                        s._w[4] + s._w[2] * T._T[9]  - s._w[0] * T._T[11],
                        s._w[5] + s._w[0] * T._T[10] - s._w[1] * T._T[9] };
    return se3(	T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2],
                T._T[3] * s._w[0] + T._T[4] * s._w[1] + T._T[5] * s._w[2],
                T._T[6] * s._w[0] + T._T[7] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2],
                T._T[3] * tmp[0] + T._T[4] * tmp[1] + T._T[5] * tmp[2],
                T._T[6] * tmp[0] + T._T[7] * tmp[1] + T._T[8] * tmp[2]);
}

se3 InvAdR(const SE3& T, const se3& s)
{
    return se3(	T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2],
                T._T[3] * s._w[0] + T._T[4] * s._w[1] + T._T[5] * s._w[2],
                T._T[6] * s._w[0] + T._T[7] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * s._w[3] + T._T[1] * s._w[4] + T._T[2] * s._w[5],
                T._T[3] * s._w[3] + T._T[4] * s._w[4] + T._T[5] * s._w[5],
                T._T[6] * s._w[3] + T._T[7] * s._w[4] + T._T[8] * s._w[5]);
}

se3 InvAdR(const SE3& T, const Vec3& v)
{
    return se3(	0.0,
                0.0,
                0.0,
                T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

se3 ad(const se3& s1, const se3& s2)
{
    //--------------------------------------------------------------------------
    // ad(s1, s2) = | [w1]    0 | | w2 |
    //              | [v1] [w1] | | v2 |
    //
    //            = |          [w1]w2 |
    //              | [v1]w2 + [w1]v2 |
    //--------------------------------------------------------------------------

    return se3(	s1._w[1] * s2._w[2] - s1._w[2] * s2._w[1],
                s1._w[2] * s2._w[0] - s1._w[0] * s2._w[2],
                s1._w[0] * s2._w[1] - s1._w[1] * s2._w[0],
                s1._w[1] * s2._w[5] - s1._w[2] * s2._w[4] - s2._w[1] * s1._w[5] + s2._w[2] * s1._w[4],
                s1._w[2] * s2._w[3] - s1._w[0] * s2._w[5] - s2._w[2] * s1._w[3] + s2._w[0] * s1._w[5],
                s1._w[0] * s2._w[4] - s1._w[1] * s2._w[3] - s2._w[0] * s1._w[4] + s2._w[1] * s1._w[3]);
}

se3 Rotate(const SE3& T, const se3& s)
{
    return se3(	T._T[0] * s._w[0] + T._T[3] * s._w[1] + T._T[6] * s._w[2],
                T._T[1] * s._w[0] + T._T[4] * s._w[1] + T._T[7] * s._w[2],
                T._T[2] * s._w[0] + T._T[5] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * s._w[3] + T._T[3] * s._w[4] + T._T[6] * s._w[5],
                T._T[1] * s._w[3] + T._T[4] * s._w[4] + T._T[7] * s._w[5],
                T._T[2] * s._w[3] + T._T[5] * s._w[4] + T._T[8] * s._w[5]);
}

se3 InvRotate(const SE3& T, const se3& s)
{
    return se3(	T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2],
                T._T[3] * s._w[0] + T._T[4] * s._w[1] + T._T[5] * s._w[2],
                T._T[6] * s._w[0] + T._T[7] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * s._w[3] + T._T[1] * s._w[4] + T._T[2] * s._w[5],
                T._T[3] * s._w[3] + T._T[4] * s._w[4] + T._T[5] * s._w[5],
                T._T[6] * s._w[3] + T._T[7] * s._w[4] + T._T[8] * s._w[5]);
}

dse3 operator*(double d, const dse3& t)
{
    return dse3(d * t._m[0], d * t._m[1], d * t._m[2], d * t._m[3], d * t._m[4], d * t._m[5]);
}

dse3 dAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	t._m[0] - T._T[10] * t._m[5] + T._T[11] * t._m[4],
                        t._m[1] - T._T[11] * t._m[3] + T._T[ 9] * t._m[5],
                        t._m[2] - T._T[ 9] * t._m[4] + T._T[10] * t._m[3] };
    return dse3(T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2],
                T._T[3] * tmp[0] + T._T[4] * tmp[1] + T._T[5] * tmp[2],
                T._T[6] * tmp[0] + T._T[7] * tmp[1] + T._T[8] * tmp[2],
                T._T[0] * t._m[3] + T._T[1] * t._m[4] + T._T[2] * t._m[5],
                T._T[3] * t._m[3] + T._T[4] * t._m[4] + T._T[5] * t._m[5],
                T._T[6] * t._m[3] + T._T[7] * t._m[4] + T._T[8] * t._m[5]);
}

dse3 dAd(const SE3& T, const Vec3& v)
{
    double tmp[3] = {	- T._T[10] * v._v[2] + T._T[11] * v._v[1],
                        - T._T[11] * v._v[0] + T._T[ 9] * v._v[2],
                        - T._T[ 9] * v._v[1] + T._T[10] * v._v[0] };
    return dse3(T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2],
                T._T[3] * tmp[0] + T._T[4] * tmp[1] + T._T[5] * tmp[2],
                T._T[6] * tmp[0] + T._T[7] * tmp[1] + T._T[8] * tmp[2],
                T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

dse3 InvdAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	T._T[0] * t._m[3] + T._T[3] * t._m[4] + T._T[6] * t._m[5],
                        T._T[1] * t._m[3] + T._T[4] * t._m[4] + T._T[7] * t._m[5],
                        T._T[2] * t._m[3] + T._T[5] * t._m[4] + T._T[8] * t._m[5] };
    return dse3(T._T[10] * tmp[2] - T._T[11] * tmp[1] + T._T[0] * t._m[0] + T._T[3] * t._m[1] + T._T[6] * t._m[2],
                T._T[11] * tmp[0] - T._T[ 9] * tmp[2] + T._T[1] * t._m[0] + T._T[4] * t._m[1] + T._T[7] * t._m[2],
                T._T[ 9] * tmp[1] - T._T[10] * tmp[0] + T._T[2] * t._m[0] + T._T[5] * t._m[1] + T._T[8] * t._m[2],
                tmp[0], tmp[1], tmp[2]);
}

dse3 InvdAd(const Vec3& p, const Vec3& f)
{
    return dse3(p._v[1] * f._v[2] - p._v[2] * f._v[1],
                p._v[2] * f._v[0] - p._v[0] * f._v[2],
                p._v[0] * f._v[1] - p._v[1] * f._v[0],
                f._v[0],
                f._v[1],
                f._v[2]);
}

SE3 EulerXYZ(const Vec3& angle)
{
    double c0 = cos(angle._v[0]);
    double s0 = sin(angle._v[0]);
    double c1 = cos(angle._v[1]);
    double s1 = sin(angle._v[1]);
    double c2 = cos(angle._v[2]);
    double s2 = sin(angle._v[2]);

    return SE3( c1*c2, c0*s2 + c2*s0*s1, s0*s2 - c0*c2*s1,
               -c1*s2, c0*c2 - s0*s1*s2, c2*s0 + c0*s1*s2,
                   s1,           -c1*s0,            c0*c1);
}

SE3 EulerZYX(const Vec3& angle)
{
    double ca = cos(angle._v[0]);
    double sa = sin(angle._v[0]);
    double cb = cos(angle._v[1]);
    double sb = sin(angle._v[1]);
    double cg = cos(angle._v[2]);
    double sg = sin(angle._v[2]);

    return SE3(               ca * cb,                sa * cb,     -sb,
               ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg,
               ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg);
}

SE3 EulerZYZ(const Vec3& angle)
{
    double ca = cos(angle._v[0]), sa = sin(angle._v[0]), cb = cos(angle._v[1]), sb = sin(angle._v[1]), cg = cos(angle._v[2]), sg = sin(angle._v[2]);
    return SE3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb);
}

SE3 EulerXYZ(const Vec3& angle, const Vec3& pos)
{
    SE3 T = RotX(angle._v[0]) * RotY(angle._v[1]) * RotZ(angle._v[2]);
    T.setPosition(pos);
    return T;
}

SE3 EulerZYX(const Vec3& angle, const Vec3& pos)
{
    double ca = cos(angle._v[0]);
    double sa = sin(angle._v[0]);
    double cb = cos(angle._v[1]);
    double sb = sin(angle._v[1]);
    double cg = cos(angle._v[2]);
    double sg = sin(angle._v[2]);

    return SE3(ca * cb, sa * cb, -sb,
               ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg,
               ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg,
               pos._v[0], pos._v[1], pos._v[2]);
}

SE3 EulerZYZ(const Vec3& angle, const Vec3& pos)
{
    double ca = cos(angle._v[0]), sa = sin(angle._v[0]), cb = cos(angle._v[1]), sb = sin(angle._v[1]), cg = cos(angle._v[2]), sg = sin(angle._v[2]);
    return SE3( ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,
               -ca * cb * sg - sa * cg, ca * cg - sa * cb * sg,  sb * sg,
                ca * sb,                sa * sb,                      cb,
                pos._v[0],                 pos._v[1],                   pos._v[2]);
}

// R = Exp(w)
// p = sin(t) / t * v + (t - sin(t)) / t^3 * <w, v> * w + (1 - cos(t)) / t^2 * (w X v)
// , when S = (w, v), t = |w|
SE3 Exp(const se3& s)
{
    double s2[] = { s._w[0] * s._w[0], s._w[1] * s._w[1], s._w[2] * s._w[2] };
    double s3[] = { s._w[0] * s._w[1], s._w[1] * s._w[2], s._w[2] * s._w[0] };
    double theta = sqrt(s2[0] + s2[1] + s2[2]), cos_t = cos(theta), alpha, beta, gamma;

    if ( theta > LIE_EPS )
    {
        double sin_t = sin(theta);
        alpha = sin_t / theta;
        beta = (SCALAR_1 - cos_t) / theta / theta;
        gamma = (s._w[0] * s._w[3] + s._w[1] * s._w[4] + s._w[2] * s._w[5]) * (theta - sin_t) / theta / theta / theta;
    }
    else
    {
        alpha = SCALAR_1 - SCALAR_1_6 * theta * theta;
        beta = SCALAR_1_2 - SCALAR_1_24 * theta * theta;
        gamma = (s._w[0] * s._w[3] + s._w[1] * s._w[4] + s._w[2] * s._w[5]) * SCALAR_1_6 - SCALAR_1_120 * theta * theta;
    }

    return SE3(beta * s2[0] + cos_t,
               beta * s3[0] + alpha * s._w[2],
               beta * s3[2] - alpha * s._w[1],

               beta * s3[0] - alpha * s._w[2],
               beta * s2[1] + cos_t,
               beta * s3[1] + alpha * s._w[0],

               beta * s3[2] + alpha * s._w[1],
               beta * s3[1] - alpha * s._w[0],
               beta * s2[2] + cos_t,

               alpha * s._w[3] + beta * (s._w[1] * s._w[5] - s._w[2] * s._w[4]) + gamma * s._w[0],
               alpha * s._w[4] + beta * (s._w[2] * s._w[3] - s._w[0] * s._w[5]) + gamma * s._w[1],
               alpha * s._w[5] + beta * (s._w[0] * s._w[4] - s._w[1] * s._w[3]) + gamma * s._w[2]);
}

// I + sin(t) / t * [S] + (1 - cos(t)) / t^2 * [S]^2, where t = |S|
SE3 Exp(const Axis& S)
{
    double s2[] = { S._v[0] * S._v[0], S._v[1] * S._v[1], S._v[2] * S._v[2] };
    double s3[] = { S._v[0] * S._v[1], S._v[1] * S._v[2], S._v[2] * S._v[0] };
    double theta = sqrt(s2[0] + s2[1] + s2[2]);
    double cos_t = cos(theta);
    double alpha = 0.0;
    double beta = 0.0;

    if (theta > LIE_EPS)
    {
        alpha = sin(theta) / theta;
        beta = (SCALAR_1 - cos_t) / theta / theta;
    }
    else
    {
        alpha = SCALAR_1 - SCALAR_1_6 * theta * theta;
        beta = SCALAR_1_2 - SCALAR_1_24 * theta * theta;
    }

    return SE3( beta * s2[0] + cos_t,           beta * s3[0] + alpha * S._v[2], beta * s3[2] - alpha * S._v[1],
                beta * s3[0] - alpha * S._v[2], beta * s2[1] + cos_t,           beta * s3[1] + alpha * S._v[0],
                beta * s3[2] + alpha * S._v[1], beta * s3[1] - alpha * S._v[0], beta * s2[2] + cos_t);
}

SE3 Exp(const Vec3& S)
{
    return SE3(S);
}

// I + sin(t) * [S] + (1 - cos(t)) * [S]^2,, where |S| = 1
SE3 Exp(const Axis& S, double theta)
{
    double s2[] = { S._v[0] * S._v[0], S._v[1] * S._v[1], S._v[2] * S._v[2] };

    if ( abs(s2[0] + s2[1] + s2[2] - SCALAR_1) > LIE_EPS ) return Exp(theta * S);

    double s3[] = { S._v[0] * S._v[1], S._v[1] * S._v[2], S._v[2] * S._v[0] };
    double alpha = sin(theta), cos_t = cos(theta), beta = SCALAR_1 - cos_t;

    return SE3( beta * s2[0] + cos_t,           beta * s3[0] + alpha * S._v[2], beta * s3[2] - alpha * S._v[1],
                beta * s3[0] - alpha * S._v[2], beta * s2[1] + cos_t,           beta * s3[1] + alpha * S._v[0],
                beta * s3[2] + alpha * S._v[1], beta * s3[1] - alpha * S._v[0], beta * s2[2] + cos_t);
}

SE3 Inv(const SE3& T)
{
    return SE3(	T._T[0], T._T[3], T._T[6],
                T._T[1], T._T[4], T._T[7],
                T._T[2], T._T[5], T._T[8],

               -T._T[0] * T._T[9] - T._T[1] * T._T[10] - T._T[2] * T._T[11],
               -T._T[3] * T._T[9] - T._T[4] * T._T[10] - T._T[5] * T._T[11],
               -T._T[6] * T._T[9] - T._T[7] * T._T[10] - T._T[8] * T._T[11]);
}

SE3 RotX(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(SCALAR_1, SCALAR_0, SCALAR_0, SCALAR_0, c, s, SCALAR_0, -s, c);
}

SE3 RotY(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(c, SCALAR_0, -s, SCALAR_0, SCALAR_1, SCALAR_0, s, SCALAR_0, c);
}

SE3 RotZ(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(c, s, SCALAR_0, -s, c, SCALAR_0, SCALAR_0, SCALAR_0, SCALAR_1);
}

// invskew(T - I)
se3 Linearize(const SE3& T)
{
    return se3(SCALAR_1_2 * (T._T[5] - T._T[7]),
               SCALAR_1_2 * (T._T[6] - T._T[2]),
               SCALAR_1_2 * (T._T[1] - T._T[3]),
               T._T[9], T._T[10], T._T[11]);
}

SE3 Normalize(const SE3& T)
{
    double idet = SCALAR_1 / (T._T[0] * (T._T[4] * T._T[8] - T._T[5] * T._T[7]) + T._T[3] * (T._T[2] * T._T[7] - T._T[1] * T._T[8]) + T._T[6] * (T._T[1] * T._T[5] - T._T[2] * T._T[4]));

    return SE3(	SCALAR_1_2 * (T._T[0] + idet * (T._T[4] * T._T[8] - T._T[5] * T._T[7])),
                SCALAR_1_2 * (T._T[1] + idet * (T._T[5] * T._T[6] - T._T[3] * T._T[8])),
                SCALAR_1_2 * (T._T[2] + idet * (T._T[3] * T._T[7] - T._T[4] * T._T[6])),
                SCALAR_1_2 * (T._T[3] + idet * (T._T[2] * T._T[7] - T._T[1] * T._T[8])),
                SCALAR_1_2 * (T._T[4] + idet * (T._T[0] * T._T[8] - T._T[2] * T._T[6])),
                SCALAR_1_2 * (T._T[5] + idet * (T._T[1] * T._T[6] - T._T[0] * T._T[7])),
                SCALAR_1_2 * (T._T[6] + idet * (T._T[1] * T._T[5] - T._T[2] * T._T[4])),
                SCALAR_1_2 * (T._T[7] + idet * (T._T[2] * T._T[3] - T._T[0] * T._T[5])),
                SCALAR_1_2 * (T._T[8] + idet * (T._T[0] * T._T[4] - T._T[1] * T._T[3])),
                T._T[9], T._T[10], T._T[11]);
}

SE3 Quaternion2SE3(const double q[4])
{
    double q11 = q[1] * q[1];
    double q22 = q[2] * q[2];
    double q33 = q[3] * q[3];
    double q12 = q[1] * q[2];
    double q23 = q[2] * q[3];
    double q31 = q[1] * q[3];
    double q01 = q[0] * q[1];
    double q02 = q[0] * q[2];
    double q03 = q[0] * q[3];

    assert(abs(q[0] * q[0] + q11 + q22 + q33 - SCALAR_1) < LIE_EPS
            && "Quaternion2SE3() --> not unit quaternion");

    return SE3( SCALAR_1 - SCALAR_2 * (q22 + q33),              SCALAR_2 * (q12 + q03),              SCALAR_2 * (q31 - q02),
                           SCALAR_2 * (q12 - q03),   SCALAR_1 - SCALAR_2 * (q11 + q33),              SCALAR_2 * (q01 + q23),
                           SCALAR_2 * (q02 + q31),              SCALAR_2 * (q23 - q01),   SCALAR_1 - SCALAR_2 * (q11 + q22));
}

SE3 Quaternion2SE3(double w, double x, double y, double z)
{
    double q11 = x * x;
    double q22 = y * y;
    double q33 = z * z;

    double q12 = x * y;
    double q23 = y * z;
    double q31 = z * x;

    double q01 = w * x;
    double q02 = w * y;
    double q03 = w * z;

    assert(abs(w * w + q11 + q22 + q33 - SCALAR_1) < LIE_EPS
            && "Quaternion2SE3() --> not unit quaternion");

    return SE3( SCALAR_1 - SCALAR_2 * (q22 + q33),              SCALAR_2 * (q12 + q03),              SCALAR_2 * (q31 - q02),
                           SCALAR_2 * (q12 - q03),   SCALAR_1 - SCALAR_2 * (q11 + q33),              SCALAR_2 * (q23 + q01),
                           SCALAR_2 * (q31 + q02),              SCALAR_2 * (q23 - q01),   SCALAR_1 - SCALAR_2 * (q11 + q22));
}

Inertia BoxInertia(double density, const Vec3& size)
{
    double mass = (double)8.0 * density * size._v[0] * size._v[1] * size._v[2];
    double ix = mass * (size._v[1] * size._v[1] + size._v[2] * size._v[2]) / SCALAR_3;
    double iy = mass * (size._v[0] * size._v[0] + size._v[2] * size._v[2]) / SCALAR_3;
    double iz = mass * (size._v[0] * size._v[0] + size._v[1] * size._v[1]) / SCALAR_3;
    return Inertia(mass, ix, iy, iz);
}

Inertia SphereInertia(double density, double rad)
{
    rad *= rad;
    double mass = density * M_PI * rad;
    double i = (double)0.4 * mass * rad;
    return Inertia(mass, i, i, i);
}

Inertia CylinderInertia(double density, double rad, double height)
{
    rad *= rad;
    double mass = density * M_PI * rad * height;
    double ix = mass * height * height  / (double)12.0 + SCALAR_1_4 * mass * rad;
    double iy = ix;
    double iz = SCALAR_1_2 * mass * rad;
    return Inertia(mass, ix, iy, iz);
}

Inertia TorusInertia(double density, double ring_rad, double tube_rad)
{
    double mass = density * SCALAR_2 * M_PI_SQR * ring_rad * tube_rad * tube_rad;
    double ix = mass * ((double)0.625 * tube_rad * tube_rad + SCALAR_1_2 * ring_rad + ring_rad);
    double iy = ix;
    double iz = mass * ((double)0.75 * tube_rad * tube_rad + ring_rad + ring_rad);
    return Inertia(mass, ix, iy, iz);
}

AInertia Inv(const Inertia& I)
{
    // TODO: Need to check
    double im = SCALAR_1 / I[9];
    double ims = sqrt(im);
    double r0 = ims * I[6];
    double r1 = ims * I[7];
    double r2 = ims * I[8];
    double a00 = I[0] - (r1 * r1 + r2 * r2);
    double a11 = I[1] - (r0 * r0 + r2 * r2);
    double a22 = I[2] - (r0 * r0 + r1 * r1);
    double a01 = I[3] + r0 * r1;
    double a02 = I[4] + r0 * r2;
    double a12 = I[5] + r1 * r2;
    double j0 = a11 * a22 - a12 * a12;
    double j6 = a02 * a12 - a01 * a22;
    double j12 = a01 * a12 - a02 * a11;
    double idet = SCALAR_1 / (a00 * j0 + a01 * j6 + a02 * j12);
    j0 *= idet;
    j6 *= idet;
    j12 *= idet;
    double j7 = idet * (a00 * a22 - a02 * a02);
    double j13 = idet * (a02 * a01 - a00 * a12);
    double j14 = idet * (a00 * a11 - a01 * a01);
    r0 *= ims;
    r1 *= ims;
    r2 *= ims;
    double j18 = j12 * r1 - j6 * r2;
    double j19 = j13 * r1 - j7 * r2;
    double j20 = j14 * r1 - j13 * r2;
    double j24 = j0 * r2 - j12 * r0;
    double j25 = j6 * r2 - j13 * r0;
    double j26 = j12 * r2 - j14 * r0;
    double j30 = j6 * r0 - j0 * r1;
    double j31 = j7 * r0 - j6 * r1;
    double j32 = j13 * r0 - j12 * r1;

    return AInertia(j0, j6, j12, j18, j24, j30, j7, j13, j19, j25, j31, j14, j20, j26, j32, r1 * j20 - r2 * j19 + im, r1 * j26 - r2 * j25, r1 * j32 - r2 * j31, r2 * j24 - r0 * j26 + im, r2 * j30 - r0 * j32, r0 * j31 - r1 * j30 + im);
}

Axis Reparameterize(const Axis& s)
{
    double theta = sqrt(s._v[0] * s._v[0] + s._v[1] * s._v[1] + s._v[2] * s._v[2]);
    double eta = theta < LIE_EPS ? 1.0 : 1.0 - (double)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
    return eta * s;
}

Axis Rotate(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v._v[0] + T._T[3] * v._v[1] + T._T[6] * v._v[2],
                T._T[1] * v._v[0] + T._T[4] * v._v[1] + T._T[7] * v._v[2],
                T._T[2] * v._v[0] + T._T[5] * v._v[1] + T._T[8] * v._v[2]);
}

Axis InvRotate(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

Axis operator*(double d, const Axis& v)
{
    return Axis(d * v._v[0], d * v._v[1], d * v._v[2]);
}

double Norm(const Axis& v)
{
    return sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
}

Axis Normalize(const Axis& v)
{
    double mag = sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
        return Axis(SCALAR_0, SCALAR_0, SCALAR_1);

    mag = SCALAR_1 / mag;
    return Axis(mag * v._v[0], mag * v._v[1], mag * v._v[2]);
}

Axis Square(const Axis& p)
{
    return Axis(p._v[0] * p._v[0], p._v[1] * p._v[1], p._v[2] * p._v[2]);
}

Axis InvAd(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

Axis ad(const Axis& s1, const se3& s2)
{
    return Axis(s2._w[2] * s1._v[1] - s2._w[1] * s1._v[2],
                s2._w[0] * s1._v[2] - s2._w[2] * s1._v[0],
                s2._w[1] * s1._v[0] - s2._w[0] * s1._v[1]);
}

Axis ad(const Axis& s1, const Axis& s2)
{
    return Axis(s2._v[2] * s1._v[1] - s2._v[1] * s1._v[2],
                s2._v[0] * s1._v[2] - s2._v[2] * s1._v[0],
                s2._v[1] * s1._v[0] - s2._v[0] * s1._v[1]);
}

Axis Axis::operator+(const Axis& v) const
{
    return Axis(_v[0] + v._v[0], _v[1] + v._v[1], _v[2] + v._v[2]);
}

se3 Axis::operator+(const Vec3& v) const
{
    return se3(_v[0], _v[1], _v[2], v._v[0], v._v[1], v._v[2]);
}

Axis Axis::operator-(const Axis& v) const
{
    return Axis(_v[0] - v._v[0], _v[1] - v._v[1], _v[2] - v._v[2]);
}

const Axis& Axis::operator += (const Axis& v)
{
    _v[0] += v._v[0];
    _v[1] += v._v[1];
    _v[2] += v._v[2];
    return *this;
}

const Axis& Axis::operator-= (const Axis& v)
{
    _v[0] -= v._v[0];
    _v[1] -= v._v[1];
    _v[2] -= v._v[2];
    return *this;
}

double Distance(const SE3& T1, const SE3& T2)
{
    return Norm(Log(Inv(T1)*T2));
}

double Norm(const se3& s)
{
    return sqrt(s._w[0] * s._w[0]
            + s._w[1] * s._w[1]
            + s._w[2] * s._w[2]
            + s._w[3] * s._w[3]
            + s._w[4] * s._w[4]
            + s._w[5] * s._w[5]);
}

double Norm(const dse3& f)
{
    return sqrt(f._m[0] * f._m[0]
            + f._m[1] * f._m[1]
            + f._m[2] * f._m[2]
            + f._m[3] * f._m[3]
            + f._m[4] * f._m[4]
            + f._m[5] * f._m[5]);
}






















































std::ostream& operator<<(std::ostream& os, const Vec3& v)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 3; i++ )
    {
        if ( v._v[i] >= SCALAR_0 ) os << " " << std::setw(6) << v._v[i] << " ";
        else os << std::setw(7) << v._v[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const Axis& v)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 3; i++ )
    {
        if ( v._v[i] >= SCALAR_0 ) os << " " << std::setw(6) << v._v[i] << " ";
        else os << std::setw(7) << v._v[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const se3& s)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 6; i++ )
    {
        if ( s._w[i] >= SCALAR_0 ) os << " " << std::setw(6) << s._w[i] << " ";
        else os << std::setw(7) << s._w[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const dse3& t)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[ ";
    for ( int i = 0; i < 6; i++ )
    {
        if ( t._m[i] >= SCALAR_0 ) os << " " << std::setw(6) << t._m[i] << " ";
        else os << std::setw(7) << t._m[i] << " ";
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}

std::ostream& operator<<(std::ostream& os, const SE3& T)
{
    std::ios_base::fmtflags flags = os.setf(std::ios::left | std::ios::fixed);
    std::streamsize sz = os.precision(3);
    os << "[" << std::endl;
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
        {
            if ( T(i,j) >= SCALAR_0 ) os << " " << std::setw(6) << T(i,j) << " ";
            else os << std::setw(7) << T(i,j) << " ";
        }
        os << ";" << std::endl;
    }
    os << "];" << std::endl;
    os.setf(flags);
    os.precision(sz);
    return os;
}






} // namespace math
} // namespace dart
