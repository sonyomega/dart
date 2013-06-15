#include <iomanip>

#include "math/LieGroup.h"

namespace dart {
namespace math {

//SE3 operator/(const SE3& T1, const SE3& T2)
//{
//    SE3 ret = SE3::Identity();

//    double tmp[] = {T1(0,0)*T2(0,0) + T1(0,1)*T2(0,1) + T1(0,2)*T2(0,2),
//                    T1(1,0)*T2(0,0) + T1(1,1)*T2(0,1) + T1(1,2)*T2(0,2),
//                    T1(2,0)*T2(0,0) + T1(2,1)*T2(0,1) + T1(2,2)*T2(0,2),
//                    T1(0,0)*T2(1,0) + T1(0,1)*T2(1,1) + T1(0,2)*T2(1,2),
//                    T1(1,0)*T2(1,0) + T1(1,1)*T2(1,1) + T1(1,2)*T2(1,2),
//                    T1(2,0)*T2(1,0) + T1(2,1)*T2(1,1) + T1(2,2)*T2(1,2),
//                    T1(0,0)*T2(2,0) + T1(0,1)*T2(2,1) + T1(0,2)*T2(2,2),
//                    T1(1,0)*T2(2,0) + T1(1,1)*T2(2,1) + T1(1,2)*T2(2,2),
//                    T1(2,0)*T2(2,0) + T1(2,1)*T2(2,1) + T1(2,2)*T2(2,2) };

//    ret(0,0) = tmp[0];  ret(0,1) = tmp[3];  ret(0,2) = tmp[6];  ret(0,3) = T1(0,3) - tmp[0]*T2(0,3) - tmp[3]*T2(1,3) - tmp[6]*T2(2,3);
//    ret(1,0) = tmp[1];  ret(1,1) = tmp[4];  ret(1,2) = tmp[7];  ret(1,3) = T1(1,3) - tmp[1]*T2(0,3) - tmp[4]*T2(1,3) - tmp[7]*T2(2,3);
//    ret(2,0) = tmp[2];  ret(2,1) = tmp[5];  ret(2,2) = tmp[8];  ret(2,3) = T1(2,3) - tmp[2]*T2(0,3) - tmp[5]*T2(1,3) - tmp[8]*T2(2,3);

//    return ret;
//}

//Vec3 Rotate(const SE3& T, const Vec3& v)
//{
//    return Vec3(T(0,0)*v[0] + T(0,1)*v[1] + T(0,2)*v[2],
//                T(1,0)*v[0] + T(1,1)*v[1] + T(1,2)*v[2],
//                T(2,0)*v[0] + T(2,1)*v[1] + T(2,2)*v[2]);
//}

//Vec3 RotateInv(const SE3& T, const Vec3& v)
//{
//    return Vec3(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
//}

//Vec3 AdInvTLinear(const SE3& T, const Vec3& v)
//{
//    return Vec3(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
//}

//Vec3 iEulerXYZ(const SE3& T)
//{
//    return Vec3(atan2(-T(1,2), T(2,2)),
//                atan2( T(0,2), sqrt(T(1,2)*T(1,2) + T(2,2)*T(2,2))),
//                atan2(-T(0,1), T(0,0)));
//}

//Vec3 iEulerZYX(const SE3& T)
//{
//    return Vec3(atan2( T(1,0), T(0,0)),
//                atan2(-T(2,0), sqrt(T(0,0)*T(0,0) + T(1,0)*T(1,0))),
//                atan2( T(2,1), T(2,2)));
//}

//Vec3 iEulerZYZ(const SE3& T)
//{
//    return Vec3(atan2(T(1,2), T(0,2)),
//                atan2(sqrt(T(2,0)*T(2,0) + T(2,1)*T(2,1)), T(2,2)),
//                atan2(T(2,1), -T(2,0)));
//}

//Vec3 iEulerZXY(const SE3& T)
//{
//    return Vec3(atan2(-T(0,1), T(1,1)),
//                atan2( T(2,1), sqrt(T(0,1)*T(0,1)+T(1,1)*T(1,1))),
//                atan2(-T(2,0), T(2,2)));
//}

//Vec3 ad_Vec3_se3(const Vec3& s1, const se3& s2)
//{
//    Vec3 ret;

//    ret << s2[2]*s1[1] - s2[1]*s1[2],
//           s2[0]*s1[2] - s2[2]*s1[0],
//           s2[1]*s1[0] - s2[0]*s1[1];

//    return ret;
//}

//se3 Log(const SE3& T)
//{
//    //--------------------------------------------------------------------------
//    // T = (R, p) = exp([w, v]), t = ||w||
//    // v = beta*p + gamma*w + 1 / 2*cross(p, w)
//    //    , beta = t*(1 + cos(t)) / (2*sin(t)), gamma = <w, p>*(1 - beta) / t^2
//    //--------------------------------------------------------------------------
//    double theta = std::acos(std::max(std::min(SCALAR_1_2*(T(0,0) + T(1,1) + T(2,2) - SCALAR_1), SCALAR_1), -SCALAR_1));
//    double alpha;
//    double beta;
//    double gamma;
//    se3 ret;

//    if (theta > M_PI - LIE_EPS)
//    {
//        const double c1 = 0.10132118364234;		// 1 / pi^2
//        const double c2 = 0.01507440267955;		// 1 / 4 / pi - 2 / pi^3
//        const double c3 = 0.00546765085347;		// 3 / pi^4 - 1 / 4 / pi^2

//        double phi = M_PI - theta;
//        double delta = SCALAR_1_2 + SCALAR_1_8*phi*phi;

//        double w[] = {	T(2,1) > T(1,2) ? theta*sqrt(SCALAR_1 + (T(0,0) - SCALAR_1)*delta) : -theta*sqrt(SCALAR_1 + (T(0,0) - SCALAR_1)*delta),
//                        T(0,2) > T(2,0) ? theta*sqrt(SCALAR_1 + (T(1,1) - SCALAR_1)*delta) : -theta*sqrt(SCALAR_1 + (T(1,1) - SCALAR_1)*delta),
//                        T(1,0) > T(0,1) ? theta*sqrt(SCALAR_1 + (T(2,2) - SCALAR_1)*delta) : -theta*sqrt(SCALAR_1 + (T(2,2) - SCALAR_1)*delta) };

//        beta = SCALAR_1_4*theta*(M_PI - theta);
//        gamma = (w[0]*T(0,3) + w[1]*T(1,3) + w[2]*T(2,3))*(c1 -  c2*phi + c3*phi*phi);

//        ret << w[0],
//               w[1],
//               w[2],
//               beta*T(0,3) - SCALAR_1_2*(w[1]*T(2,3) - w[2]*T(1,3)) + gamma*w[0],
//               beta*T(1,3) - SCALAR_1_2*(w[2]*T(0,3) - w[0]*T(2,3)) + gamma*w[1],
//               beta*T(2,3) - SCALAR_1_2*(w[0]*T(1,3) - w[1]*T(0,3)) + gamma*w[2];
//    }
//    else
//    {
//        if (theta > LIE_EPS)
//        {
//            alpha = SCALAR_1_2*theta / sin(theta);
//            beta = (SCALAR_1 + cos(theta))*alpha;
//            gamma = (SCALAR_1 - beta) / theta / theta;
//        }
//        else
//        {
//            alpha = SCALAR_1_2 + SCALAR_1_12*theta*theta;
//            beta = SCALAR_1 - SCALAR_1_12*theta*theta;
//            gamma = SCALAR_1_12 + SCALAR_1_720*theta*theta;
//        }

//        double w[] = { alpha*(T(2,1) - T(1,2)), alpha*(T(0,2) - T(2,0)), alpha*(T(1,0) - T(0,1)) };
//        gamma *= w[0]*T(0,3) + w[1]*T(1,3) + w[2]*T(2,3);

//        ret << w[0],
//               w[1],
//               w[2],
//               beta*T(0,3) + SCALAR_1_2*(w[2]*T(1,3) - w[1]*T(2,3)) + gamma*w[0],
//               beta*T(1,3) + SCALAR_1_2*(w[0]*T(2,3) - w[2]*T(0,3)) + gamma*w[1],
//               beta*T(2,3) + SCALAR_1_2*(w[1]*T(0,3) - w[0]*T(1,3)) + gamma*w[2];
//    }

//    return ret;
//}

//Axis LogR(const SE3& T)
//{
//    double theta = std::acos(std::max(std::min(SCALAR_1_2*(T(0,0) + T(1,1) + T(2,2) - SCALAR_1), SCALAR_1), -SCALAR_1)), alpha;

//    if ( theta > M_PI - LIE_EPS )
//    {
//        double delta = SCALAR_1_2 + SCALAR_1_8*(M_PI - theta)*(M_PI - theta);

//        return Axis(T(2,1) > T(1,2) ? theta*sqrt(SCALAR_1 + (T(0,0) - SCALAR_1)*delta) : -theta*sqrt(SCALAR_1 + (T(0,0) - SCALAR_1)*delta),
//                    T(0,2) > T(2,0) ? theta*sqrt(SCALAR_1 + (T(1,1) - SCALAR_1)*delta) : -theta*sqrt(SCALAR_1 + (T(1,1) - SCALAR_1)*delta),
//                    T(1,0) > T(0,1) ? theta*sqrt(SCALAR_1 + (T(2,2) - SCALAR_1)*delta) : -theta*sqrt(SCALAR_1 + (T(2,2) - SCALAR_1)*delta));
//    } else
//    {
//        if ( theta > LIE_EPS )
//            alpha = SCALAR_1_2*theta / sin(theta);
//        else
//            alpha = SCALAR_1_2 + SCALAR_1_12*theta*theta;

//        return Axis(alpha*(T(2,1) - T(1,2)), alpha*(T(0,2) - T(2,0)), alpha*(T(1,0) - T(0,1)));
//    }
//}

//// re = T*s*Inv(T)
//se3 AdT(const SE3& T, const se3& s)
//{
//    //--------------------------------------------------------------------------
//    // w' = R*w
//    // v' = p x R*w + R*v
//    //--------------------------------------------------------------------------
//    se3 ret;
//    double Rw[3] = { T(0,0)*s[0] + T(0,1)*s[1] + T(0,2)*s[2],
//                     T(1,0)*s[0] + T(1,1)*s[1] + T(1,2)*s[2],
//                     T(2,0)*s[0] + T(2,1)*s[1] + T(2,2)*s[2] };
//    ret << Rw[0],
//           Rw[1],
//           Rw[2],
//           T(1,3)*Rw[2] - T(2,3)*Rw[1] + T(0,0)*s[3] + T(0,1)*s[4] + T(0,2)*s[5],
//           T(2,3)*Rw[0] - T(0,3)*Rw[2] + T(1,0)*s[3] + T(1,1)*s[4] + T(1,2)*s[5],
//           T(0,3)*Rw[1] - T(1,3)*Rw[0] + T(2,0)*s[3] + T(2,1)*s[4] + T(2,2)*s[5];

//    return ret;
//}

//se3 AdR(const SE3& T, const se3& s)
//{
//    //--------------------------------------------------------------------------
//    // w' = R*w
//    // v' = R*v
//    //--------------------------------------------------------------------------
//    se3 ret;

//    ret << T(0,0)*s[0] + T(0,1)*s[1] + T(0,2)*s[2],
//           T(1,0)*s[0] + T(1,1)*s[1] + T(1,2)*s[2],
//           T(2,0)*s[0] + T(2,1)*s[1] + T(2,2)*s[2],
//           T(0,0)*s[3] + T(0,1)*s[4] + T(0,2)*s[5],
//           T(1,0)*s[3] + T(1,1)*s[4] + T(1,2)*s[5],
//           T(2,0)*s[3] + T(2,1)*s[4] + T(2,2)*s[5];

//    return ret;
//}

//se3 AdTAngular(const SE3& T, const Axis& s)
//{
//    //--------------------------------------------------------------------------
//    // w' = R*w
//    // v' = r x R*w
//    //--------------------------------------------------------------------------
//    se3 ret;
//    double Rw[3] = { T(0,0)*s[0] + T(0,1)*s[1] + T(0,2)*s[2],
//                     T(1,0)*s[0] + T(1,1)*s[1] + T(1,2)*s[2],
//                     T(2,0)*s[0] + T(2,1)*s[1] + T(2,2)*s[2] };
//    ret << Rw[0],
//           Rw[1],
//           Rw[2],
//           T(1,3)*Rw[2] - T(2,3)*Rw[1],
//           T(2,3)*Rw[0] - T(0,3)*Rw[2],
//           T(0,3)*Rw[1] - T(1,3)*Rw[0];

//    return ret;
//}

//se3 AdTLinear(const SE3& T, const Vec3& v)
//{
//    //--------------------------------------------------------------------------
//    // w' = 0
//    // v' = R*v
//    //--------------------------------------------------------------------------
//    se3 ret;

//    ret << SCALAR_0, SCALAR_0, SCALAR_0,
//                T(0,0)*v[0] + T(0,1)*v[1] + T(0,2)*v[2],
//                T(1,0)*v[0] + T(1,1)*v[1] + T(1,2)*v[2],
//                T(2,0)*v[0] + T(2,1)*v[1] + T(2,2)*v[2];

//    return ret;
//}

//Jacobian AdTJac(const SE3& T, const Jacobian& J)
//{
//    Jacobian AdTJ = Jacobian::Zero(6,J.cols());

//    for (int i = 0; i < J.cols(); ++i)
//    {
//        AdTJ.col(i).noalias() = AdT(T, J.col(i));
//    }

//    return AdTJ;
//}

//se3 AdP(const Vec3& p, const se3& s)
//{
//    //--------------------------------------------------------------------------
//    // w' = w
//    // v' = p x w + v
//    //--------------------------------------------------------------------------
//    se3 ret;
//    ret << s[0],
//           s[1],
//           s[2],
//           p[1]*s[2] - p[2]*s[1] + s[3],
//           p[2]*s[0] - p[0]*s[2] + s[4],
//           p[0]*s[1] - p[1]*s[0] + s[5];

//    return ret;
//}

//Jacobian AdRJac(const SE3& T, const Jacobian& J)
//{
//    Jacobian AdTJ(6,J.cols());

//    for (int i = 0; i < J.cols(); ++i)
//    {
//        AdTJ.col(i) = math::AdR(T, J.col(i));
//    }

//    return AdTJ;
//}

//// re = Inv(T)*s*T
//se3 AdInvT(const SE3& T, const se3& s)
//{
//    se3 ret;
//    double tmp[3] = {	s[3] + s[1]*T(2,3) - s[2]*T(1,3),
//                        s[4] + s[2]*T(0,3)  - s[0]*T(2,3),
//                        s[5] + s[0]*T(1,3) - s[1]*T(0,3) };

//    ret << T(0,0)*s[0] + T(1,0)*s[1] + T(2,0)*s[2],
//                T(0,1)*s[0] + T(1,1)*s[1] + T(2,1)*s[2],
//                T(0,2)*s[0] + T(1,2)*s[1] + T(2,2)*s[2],
//                T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
//                T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
//                T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2];

//    return ret;
//}

//se3 AdInvR(const SE3& T, const se3& s)
//{
//    se3 ret;

//    ret << T(0,0)*s[0] + T(1,0)*s[1] + T(2,0)*s[2],
//                T(0,1)*s[0] + T(1,1)*s[1] + T(2,1)*s[2],
//                T(0,2)*s[0] + T(1,2)*s[1] + T(2,2)*s[2],
//                T(0,0)*s[3] + T(1,0)*s[4] + T(2,0)*s[5],
//                T(0,1)*s[3] + T(1,1)*s[4] + T(2,1)*s[5],
//                T(0,2)*s[3] + T(1,2)*s[4] + T(2,2)*s[5];

//    return ret;
//}

//se3 AdInvRLinear(const SE3& T, const Vec3& v)
//{
//    se3 ret;

//    ret << 0.0,
//                0.0,
//                0.0,
//                T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2];

//    return ret;
//}

//se3 ad(const se3& s1, const se3& s2)
//{
//    //--------------------------------------------------------------------------
//    // ad(s1, s2) = | [w1]    0 | | w2 |
//    //              | [v1] [w1] | | v2 |
//    //
//    //            = |          [w1]w2 |
//    //              | [v1]w2 + [w1]v2 |
//    //--------------------------------------------------------------------------
//    se3 ret;

//    ret << s1[1]*s2[2] - s1[2]*s2[1],
//                s1[2]*s2[0] - s1[0]*s2[2],
//                s1[0]*s2[1] - s1[1]*s2[0],
//                s1[1]*s2[5] - s1[2]*s2[4] - s2[1]*s1[5] + s2[2]*s1[4],
//                s1[2]*s2[3] - s1[0]*s2[5] - s2[2]*s1[3] + s2[0]*s1[5],
//                s1[0]*s2[4] - s1[1]*s2[3] - s2[0]*s1[4] + s2[1]*s1[3];

//    return ret;
//}

//dse3 dAdT(const SE3& T, const dse3& t)
//{
//    dse3 ret;

//    double tmp[3] = {	t[0] - T(1,3)*t[5] + T(2,3)*t[4],
//                        t[1] - T(2,3)*t[3] + T(0,3)*t[5],
//                        t[2] - T(0,3)*t[4] + T(1,3)*t[3] };
//    ret << T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
//                T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
//                T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2],
//                T(0,0)*t[3] + T(1,0)*t[4] + T(2,0)*t[5],
//                T(0,1)*t[3] + T(1,1)*t[4] + T(2,1)*t[5],
//                T(0,2)*t[3] + T(1,2)*t[4] + T(2,2)*t[5];

//    return ret;
//}

//dse3 dAdTLinear(const SE3& T, const Vec3& v)
//{
//    dse3 ret;
//    double tmp[3] = {	- T(1,3)*v[2] + T(2,3)*v[1],
//                        - T(2,3)*v[0] + T(0,3)*v[2],
//                        - T(0,3)*v[1] + T(1,3)*v[0] };
//    ret << T(0,0)*tmp[0] + T(1,0)*tmp[1] + T(2,0)*tmp[2],
//                T(0,1)*tmp[0] + T(1,1)*tmp[1] + T(2,1)*tmp[2],
//                T(0,2)*tmp[0] + T(1,2)*tmp[1] + T(2,2)*tmp[2],
//                T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2];

//    return ret;
//}

//dse3 dAdInvT(const SE3& T, const dse3& t)
//{
//    dse3 ret;

//    double tmp[3] = {	T(0,0)*t[3] + T(0,1)*t[4] + T(0,2)*t[5],
//                        T(1,0)*t[3] + T(1,1)*t[4] + T(1,2)*t[5],
//                        T(2,0)*t[3] + T(2,1)*t[4] + T(2,2)*t[5] };

//    ret << T(1,3)*tmp[2] - T(2,3)*tmp[1] + T(0,0)*t[0] + T(0,1)*t[1] + T(0,2)*t[2],
//                T(2,3)*tmp[0] - T(0,3)*tmp[2] + T(1,0)*t[0] + T(1,1)*t[1] + T(1,2)*t[2],
//                T(0,3)*tmp[1] - T(1,3)*tmp[0] + T(2,0)*t[0] + T(2,1)*t[1] + T(2,2)*t[2],
//                tmp[0], tmp[1], tmp[2];

//    return ret;
//}

//dse3 dAdInvPLinear(const Vec3& p, const Vec3& f)
//{
//    dse3 ret;

//    ret << p[1]*f[2] - p[2]*f[1],
//                p[2]*f[0] - p[0]*f[2],
//                p[0]*f[1] - p[1]*f[0],
//                f[0],
//                f[1],
//                f[2];

//    return ret;
//}

//SE3 EulerXYZ(const Vec3& angle)
//{
//    SE3 ret = SE3::Identity();

//    double c0 = cos(angle[0]);
//    double s0 = sin(angle[0]);
//    double c1 = cos(angle[1]);
//    double s1 = sin(angle[1]);
//    double c2 = cos(angle[2]);
//    double s2 = sin(angle[2]);

//    ret(0,0) = c1*c2;             ret(0,1) = -c1*s2;            ret(0,2) = s1;
//    ret(1,0) = c0*s2 + c2*s0*s1;  ret(1,1) = c0*c2 - s0*s1*s2;  ret(1,2) = -c1*s0;
//    ret(2,0) = s0*s2 - c0*c2*s1;  ret(2,1) = c2*s0 + c0*s1*s2;  ret(2,2) = c0*c1;

//    return ret;
//}

//SE3 EulerZYX(const Vec3& angle)
//{
//    SE3 ret = SE3::Identity();

//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb;  ret(0,1) = ca*sb*sg - sa*cg;  ret(0,2) = ca*sb*cg + sa*sg;
//    ret(1,0) = sa*cb;  ret(1,1) = sa*sb*sg + ca*cg;  ret(1,2) = sa*sb*cg - ca*sg;
//    ret(2,0) = -sb;    ret(2,1) = cb*sg;             ret(2,2) = cb*cg;

//    return ret;
//}

//SE3 EulerZYZ(const Vec3& angle)
//{
//    SE3 ret = SE3::Identity();

//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb*cg - sa*sg;  ret(0,1) = -ca*cb*sg - sa*cg;  ret(0,2) = ca*sb;
//    ret(1,0) = sa*cb*cg + ca*sg;  ret(1,1) = ca*cg - sa*cb*sg;   ret(1,2) = sa*sb;
//    ret(2,0) = -sb*cg;            ret(2,1) = sb*sg;              ret(2,2) = cb;

//    return ret;
//}

//SE3 EulerXYZ(const Vec3& angle, const Vec3& pos)
//{
//    SE3 T = RotX(angle[0])*RotY(angle[1])*RotZ(angle[2]);

//    T(0,3) = pos[0];
//    T(1,3) = pos[1];
//    T(2,3) = pos[2];

//    return T;
//}

//SE3 EulerZYX(const Vec3& angle, const Vec3& pos)
//{
//    SE3 ret;
//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb;  ret(0,1) = ca*sb*sg - sa*cg;  ret(0,2) = ca*sb*cg + sa*sg;  ret(0,3) = pos[0];
//    ret(1,0) = sa*cb;  ret(1,1) = sa*sb*sg + ca*cg;  ret(1,2) = sa*sb*cg - ca*sg;  ret(1,3) = pos[1];
//    ret(2,0) = -sb;    ret(2,1) = cb*sg;             ret(2,2) = cb*cg;             ret(2,3) = pos[2];

//    return ret;
//}

//SE3 EulerZYZ(const Vec3& angle, const Vec3& pos)
//{
//    SE3 ret;
//    double ca = cos(angle[0]);
//    double sa = sin(angle[0]);
//    double cb = cos(angle[1]);
//    double sb = sin(angle[1]);
//    double cg = cos(angle[2]);
//    double sg = sin(angle[2]);

//    ret(0,0) = ca*cb*cg - sa*sg;  ret(0,1) = -ca*cb*sg - sa*cg;  ret(0,2) = ca*sb;  ret(0,3) = pos[0];
//    ret(1,0) = sa*cb*cg + ca*sg;  ret(1,1) = ca*cg - sa*cb*sg;   ret(1,2) = sa*sb;  ret(1,3) = pos[1];
//    ret(2,0) = -sb*cg;            ret(2,1) = sb*sg;              ret(2,2) = cb;     ret(2,3) = pos[2];

//    return ret;
//}

//// R = Exp(w)
//// p = sin(t) / t*v + (t - sin(t)) / t^3*<w, v>*w + (1 - cos(t)) / t^2*(w X v)
//// , when S = (w, v), t = |w|
//SE3 Exp(const se3& s)
//{
//    SE3 ret;
//    double s2[] = { s[0]*s[0], s[1]*s[1], s[2]*s[2] };
//    double s3[] = { s[0]*s[1], s[1]*s[2], s[2]*s[0] };
//    double theta = sqrt(s2[0] + s2[1] + s2[2]), cos_t = cos(theta), alpha, beta, gamma;

//    if ( theta > LIE_EPS )
//    {
//        double sin_t = sin(theta);
//        alpha = sin_t / theta;
//        beta = (SCALAR_1 - cos_t) / theta / theta;
//        gamma = (s[0]*s[3] + s[1]*s[4] + s[2]*s[5])*(theta - sin_t) / theta / theta / theta;
//    }
//    else
//    {
//        alpha = SCALAR_1 - SCALAR_1_6*theta*theta;
//        beta = SCALAR_1_2 - SCALAR_1_24*theta*theta;
//        gamma = (s[0]*s[3] + s[1]*s[4] + s[2]*s[5])*SCALAR_1_6 - SCALAR_1_120*theta*theta;
//    }

//    ret(0,0) = beta*s2[0] + cos_t;       ret(0,1) = beta*s3[0] - alpha*s[2];  ret(0,2) = beta*s3[2] + alpha*s[1];  ret(0,3) = alpha*s[3] + beta*(s[1]*s[5] - s[2]*s[4]) + gamma*s[0];
//    ret(1,0) = beta*s3[0] + alpha*s[2];  ret(1,1) = beta*s2[1] + cos_t;       ret(1,2) = beta*s3[1] - alpha*s[0];  ret(1,3) = alpha*s[4] + beta*(s[2]*s[3] - s[0]*s[5]) + gamma*s[1];
//    ret(2,0) = beta*s3[2] - alpha*s[1];  ret(2,1) = beta*s3[1] + alpha*s[0];  ret(2,2) = beta*s2[2] + cos_t;       ret(2,3) = alpha*s[5] + beta*(s[0]*s[4] - s[1]*s[3]) + gamma*s[2];

//    return ret;
//}

//// I + sin(t) / t*[S] + (1 - cos(t)) / t^2*[S]^2, where t = |S|
//SE3 ExpAngular(const Axis& S)
//{
//    SE3 ret = SE3::Identity();
//    double s2[] = { S[0]*S[0], S[1]*S[1], S[2]*S[2] };
//    double s3[] = { S[0]*S[1], S[1]*S[2], S[2]*S[0] };
//    double theta = sqrt(s2[0] + s2[1] + s2[2]);
//    double cos_t = cos(theta);
//    double alpha = 0.0;
//    double beta = 0.0;

//    if (theta > LIE_EPS)
//    {
//        alpha = sin(theta) / theta;
//        beta = (SCALAR_1 - cos_t) / theta / theta;
//    }
//    else
//    {
//        alpha = SCALAR_1 - SCALAR_1_6*theta*theta;
//        beta = SCALAR_1_2 - SCALAR_1_24*theta*theta;
//    }

//    ret(0,0) = beta*s2[0] + cos_t;       ret(0,1) = beta*s3[0] - alpha*S[2];  ret(0,2) = beta*s3[2] + alpha*S[1];
//    ret(1,0) = beta*s3[0] + alpha*S[2];  ret(1,1) = beta*s2[1] + cos_t;       ret(1,2) = beta*s3[1] - alpha*S[0];
//    ret(2,0) = beta*s3[2] - alpha*S[1];  ret(2,1) = beta*s3[1] + alpha*S[0];  ret(2,2) = beta*s2[2] + cos_t;

//    return ret;
//}

//// I + sin(t)*[S] + (1 - cos(t))*[S]^2,, where |S| = 1
//SE3 ExpAngular(const Axis& S, double theta)
//{
//    SE3 ret = SE3::Identity();
//    double s2[] = { S[0]*S[0], S[1]*S[1], S[2]*S[2] };

//    if ( abs(s2[0] + s2[1] + s2[2] - SCALAR_1) > LIE_EPS ) return ExpAngular(theta*S);

//    double s3[] = { S[0]*S[1], S[1]*S[2], S[2]*S[0] };
//    double alpha = sin(theta), cos_t = cos(theta), beta = SCALAR_1 - cos_t;

//    ret(0,0) = beta*s2[0] + cos_t;       ret(0,1) = beta*s3[0] - alpha*S[2];  ret(0,2) = beta*s3[2] + alpha*S[1];
//    ret(1,0) = beta*s3[0] + alpha*S[2];  ret(1,1) = beta*s2[1] + cos_t;       ret(1,2) = beta*s3[1] - alpha*S[0];
//    ret(2,0) = beta*s3[2] - alpha*S[1];  ret(2,1) = beta*s3[1] + alpha*S[0];  ret(2,2) = beta*s2[2] + cos_t;

//    return ret;
//}

//SE3 ExpLinear(const Vec3& s)
//{
//    SE3 ret = SE3::Identity();

//    ret(0,3) = s[0];
//    ret(1,3) = s[1];
//    ret(2,3) = s[2];

//    return ret;
//}

//SE3 Inv(const SE3& T)
//{
//    SE3 ret = SE3::Identity();

//    ret(0,0) = T(0,0);  ret(0,1) = T(1,0);  ret(0,2) = T(2,0);  ret(0,3) = -T(0,0)*T(0,3) - T(1,0)*T(1,3) - T(2,0)*T(2,3);
//    ret(1,0) = T(0,1);  ret(1,1) = T(1,1);  ret(1,2) = T(2,1);  ret(1,3) = -T(0,1)*T(0,3) - T(1,1)*T(1,3) - T(2,1)*T(2,3);
//    ret(2,0) = T(0,2);  ret(2,1) = T(1,2);  ret(2,2) = T(2,2);  ret(2,3) = -T(0,2)*T(0,3) - T(1,2)*T(1,3) - T(2,2)*T(2,3);

//    return ret;
//}

//SE3 RotX(double t)
//{
//    SE3 ret = SE3::Identity();
//    double c = cos(t);
//    double s = sin(t);

//    ret(1,1) = c;  ret(1,2) = -s;
//    ret(2,1) = s;  ret(2,2) = c;

//    return ret;
//}

//SE3 RotY(double t)
//{
//    SE3 ret = SE3::Identity();
//    double c = cos(t);
//    double s = sin(t);

//    ret(0,0) = c;   ret(0,2) = s;
//    ret(2,0) = -s;  ret(2,2) = c;

//    return ret;
//}

//SE3 RotZ(double t)
//{
//    SE3 ret = SE3::Identity();
//    double c = cos(t);
//    double s = sin(t);

//    ret(0,0) = c;  ret(0,1) = -s;
//    ret(1,0) = s;  ret(1,1) = c;

//    return ret;
//}

//SE3 Normalize(const SE3& T)
//{
//    SE3 ret = SE3::Identity();
//    double idet = SCALAR_1 / (T(0,0)*(T(1,1)*T(2,2) - T(2,1)*T(1,2)) +
//                              T(0,1)*(T(2,0)*T(1,2) - T(1,0)*T(2,2)) +
//                              T(0,2)*(T(1,0)*T(2,1) - T(2,0)*T(1,1)));

//    ret(0,0) = SCALAR_1_2*(T(0,0) + idet*(T(1,1)*T(2,2) - T(2,1)*T(1,2)));
//    ret(0,1) = SCALAR_1_2*(T(0,1) + idet*(T(2,0)*T(1,2) - T(1,0)*T(2,2)));
//    ret(0,2) = SCALAR_1_2*(T(0,2) + idet*(T(1,0)*T(2,1) - T(2,0)*T(1,1)));
//    ret(0,3) = T(0,3);

//    ret(1,0) = SCALAR_1_2*(T(1,0) + idet*(T(2,1)*T(0,2) - T(0,1)*T(2,2)));
//    ret(1,1) = SCALAR_1_2*(T(1,1) + idet*(T(0,0)*T(2,2) - T(2,0)*T(0,2)));
//    ret(1,2) = SCALAR_1_2*(T(1,2) + idet*(T(2,0)*T(0,1) - T(0,0)*T(2,1)));
//    ret(1,3) = T(1,3);

//    ret(2,0) = SCALAR_1_2*(T(2,0) + idet*(T(0,1)*T(1,2) - T(1,1)*T(0,2)));
//    ret(2,1) = SCALAR_1_2*(T(2,1) + idet*(T(1,0)*T(0,2) - T(0,0)*T(1,2)));
//    ret(2,2) = SCALAR_1_2*(T(2,2) + idet*(T(0,0)*T(1,1) - T(1,0)*T(0,1)));
//    ret(2,3) = T(2,3);

//    return ret;
//}

//SE3 Quaternion2SE3(const double q[4])
//{
//    SE3 ret = SE3::Identity();
//    double q11 = q[1]*q[1];
//    double q22 = q[2]*q[2];
//    double q33 = q[3]*q[3];
//    double q12 = q[1]*q[2];
//    double q23 = q[2]*q[3];
//    double q31 = q[1]*q[3];
//    double q01 = q[0]*q[1];
//    double q02 = q[0]*q[2];
//    double q03 = q[0]*q[3];

//    assert(abs(q[0]*q[0] + q11 + q22 + q33 - SCALAR_1) < LIE_EPS
//            && "Quaternion2SE3() --> not unit quaternion");

//    ret(0,0) = SCALAR_1 - SCALAR_2*(q22 + q33);
//    ret(0,1) =            SCALAR_2*(q12 - q03);
//    ret(0,2) =            SCALAR_2*(q02 + q31);

//    ret(1,0) =            SCALAR_2*(q12 + q03);
//    ret(1,1) = SCALAR_1 - SCALAR_2*(q11 + q33);
//    ret(1,2) =            SCALAR_2*(q23 - q01);

//    ret(2,0) =            SCALAR_2*(q31 - q02);
//    ret(2,1) =            SCALAR_2*(q01 + q23);
//    ret(2,2) = SCALAR_1 - SCALAR_2*(q11 + q22);

//    return ret;
//}

//SE3 Quaternion2SE3(double w, double x, double y, double z)
//{
//    SE3 ret = SE3::Identity();
//    double q11 = x*x;
//    double q22 = y*y;
//    double q33 = z*z;

//    double q12 = x*y;
//    double q23 = y*z;
//    double q31 = z*x;

//    double q01 = w*x;
//    double q02 = w*y;
//    double q03 = w*z;

//    assert(abs(w*w + q11 + q22 + q33 - SCALAR_1) < LIE_EPS
//            && "Quaternion2SE3() --> not unit quaternion");

//    ret(0,0) = SCALAR_1 - SCALAR_2*(q22 + q33);
//    ret(0,1) =            SCALAR_2*(q12 - q03);
//    ret(0,2) =            SCALAR_2*(q02 + q31);

//    ret(1,0) =            SCALAR_2*(q12 + q03);
//    ret(1,1) = SCALAR_1 - SCALAR_2*(q11 + q33);
//    ret(1,2) =            SCALAR_2*(q23 - q01);

//    ret(2,0) =            SCALAR_2*(q31 - q02);
//    ret(2,1) =            SCALAR_2*(q01 + q23);
//    ret(2,2) = SCALAR_1 - SCALAR_2*(q11 + q22);

//    return ret;
//}

//Axis Reparameterize(const Axis& s)
//{
//    double theta = sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
//    double eta = theta < LIE_EPS ? 1.0 : 1.0 - (double)((int)(theta / M_PI + 1.0) / 2)*M_2PI / theta;

//    return eta*s;
//}

//Axis AdInvTAngular(const SE3& T, const Axis& v)
//{
//    return Axis(T(0,0)*v[0] + T(1,0)*v[1] + T(2,0)*v[2],
//                T(0,1)*v[0] + T(1,1)*v[1] + T(2,1)*v[2],
//                T(0,2)*v[0] + T(1,2)*v[1] + T(2,2)*v[2]);
//}

////Axis ad(const Axis& s1, const se3& s2)
////{
////    return Axis(s2[2]*s1[1] - s2[1]*s1[2],
////                s2[0]*s1[2] - s2[2]*s1[0],
////                s2[1]*s1[0] - s2[0]*s1[1]);
////}

////Axis ad_Axis_Axis(const Axis& s1, const Axis& s2)
////{
////    return Axis(s2[2]*s1[1] - s2[1]*s1[2],
////                s2[0]*s1[2] - s2[2]*s1[0],
////                s2[1]*s1[0] - s2[0]*s1[1]);
////}

//dse3 dad(const se3& s, const dse3& t)
//{
//    dse3 ret;

//    ret << t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4],
//           t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5],
//           t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3],
//           t[4] * s[2] - t[5] * s[1],
//           t[5] * s[0] - t[3] * s[2],
//           t[3] * s[1] - t[4] * s[0];

//    return ret;
//}

//Inertia Transform(const SE3& T, const Inertia& AI)
//{
//    // operation count: multiplication = 186, addition = 117, subtract = 21

//    Inertia ret;

//    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
//    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
//    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
//    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
//    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
//    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
//    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
//    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
//    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
//    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
//    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
//    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
//    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
//    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
//    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
//    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
//    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
//    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
//    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
//    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
//    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
//    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
//    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
//    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
//    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
//    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
//    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
//    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
//    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
//    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
//    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
//    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
//    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
//    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
//    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
//    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

//    ret(0,0) = f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0);
//    ret(0,1) = f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1);
//    ret(0,2) = f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2);
//    ret(0,3) = f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0);
//    ret(0,4) = f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1);
//    ret(0,5) = f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2);
//    ret(1,1) = f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1);
//    ret(1,2) = f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2);
//    ret(1,3) = g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0);
//    ret(1,4) = g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1);
//    ret(1,5) = g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2);
//    ret(2,2) = (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2);
//    ret(2,3) = g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0);
//    ret(2,4) = g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1);
//    ret(2,5) = g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2);
//    ret(3,3) = h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0);
//    ret(3,4) = h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1);
//    ret(3,5) = h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2);
//    ret(4,4) = h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1);
//    ret(4,5) = h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2);
//    ret(5,5) = (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

//    ret.triangularView<Eigen::StrictlyLower>() = ret.transpose();

//    return ret;
//}



//double Distance(const SE3& T1, const SE3& T2)
//{
//    return Norm(Log(Inv(T1)*T2));
//}

} // namespace math
} // namespace dart
