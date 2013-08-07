#ifndef DART_MATH_LIE_GROUP_H
#define DART_MATH_LIE_GROUP_H

#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>
#include <cfloat>

#include <Eigen/Dense>

#include "common/UtilsCode.h"

#define LIE_EPS		 1E-6
#define SCALAR_0	 0.0
#define SCALAR_1	 1.0
#define SCALAR_2	 2.0
#define SCALAR_3	 3.0
#define SCALAR_1_2	 0.5
#define SCALAR_1_3	 0.333333333333333333333
#define SCALAR_1_4	 0.25
#define SCALAR_1_6	 0.166666666666666666667
#define SCALAR_1_8	 0.125
#define SCALAR_1_12	 0.0833333333333333333333
#define SCALAR_1_24	 0.0416666666666666666667
#define SCALAR_1_30	 0.0333333333333333333333
#define SCALAR_1_60	 0.0166666666666666666667
#define SCALAR_1_120 0.00833333333333333333333
#define SCALAR_1_180 0.00555555555555555555556
#define SCALAR_1_720 0.00138888888888888888889
#define SCALAR_1_1260 0.000793650793650793650794

#define M_PI		 3.14159265358979323846
#define M_2PI		 6.28318530717958647693		// = 2 * pi
#define M_PI_SQR	 9.86960440108935861883		// = pi^2
#define M_RADIAN	 0.0174532925199432957692	// = pi / 180
#define M_DEGREE	 57.2957795130823208768		// = pi / 180
#define SCALAR_MAX	 DBL_MAX

// TODO:
//   3. naming check
//   5. remove wired functions
//   7. License!
//   9. check if Inertia * se3
//  11. random setter
//  13. / operator for SE3 * invSE3.

namespace Eigen {
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
}

namespace dart {
namespace math {

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector3d Axis;
typedef Eigen::Vector3d so3;
typedef Eigen::Vector3d dso3;
typedef Eigen::Matrix3d SO3;
typedef Eigen::Isometry3d SE3;
typedef Eigen::Vector6d se3;
typedef Eigen::Vector6d dse3;
typedef Eigen::Matrix6d Inertia;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> Jacobian;

/// @brief Compute geometric distance on SE(3) manifold.
/// Norm(Log(Inv(T1) * T2)).
//double Distance(const SE3& T1, const SE3& T2);

//SE3 operator/(const SE3& T1, const SE3& T2);

//------------------------------------------------------------------------------

/// @brief Get a transformation matrix given by the Euler XYZ angle,
/// where the positional part is set to be zero.
SE3 EulerXYZ(const Vec3& angle);

/// @brief get a transformation matrix given by the Euler XYZ angle and
/// position.
SE3 EulerXYZ(const Vec3& angle, const Vec3& position);

///// @brief get a transformation matrix given by the Euler ZYX angle,
///// where the positional part is set to be zero.
///// singularity : x[1] = -+ 0.5*PI
///// @sa SE3::iEulerZYX
//SE3 EulerZYX(const Vec3& angle);

///// @brief get a transformation matrix given by the Euler ZYX angle and
///// position.
///// singularity : x[1] = -+ 0.5*PI
//SE3 EulerZYX(const Vec3& angle, const Vec3& position);

///// @brief Get a transformation matrix given by the Euler ZYZ angle,
///// where the positional part is set to be zero.
///// singularity : x[1] = 0, PI
///// @sa SE3::iEulerZYZ
//SE3 EulerZYZ(const Vec3& angle);

///// @brief get a transformation matrix given by the Euler ZYZ angle and
///// position.
///// singularity : x[1] = 0, PI
//SE3 EulerZYZ(const Vec3& angle, const Vec3& position);

/// @brief get the Euler ZYX angle from T
////// @sa Eigen_Vec3::EulerXYZ
Vec3 iEulerXYZ(const SE3& T);

///// @brief get the Euler ZYX angle from T
///// @sa Eigen_Vec3::EulerZYX
//Vec3 iEulerZYX(const SE3& T);

///// @brief get the Euler ZYZ angle from T
///// @sa Eigen_Vec3::EulerZYZ
//Vec3 iEulerZYZ(const SE3& T);

///// @brief get the Euler ZYZ angle from T
///// @sa Eigen_Vec3::EulerZXY
//Vec3 iEulerZXY(const SE3 &T);

//------------------------------------------------------------------------------
/// @brief rotate q by T.
/// @return @f$R q@f$, where @f$T=(R,p)@f$.
Vec3 Rotate(const SE3& T, const Vec3& q);

///// @brief rotate q by Inv(T).
//Vec3 RotateInv(const SE3& T, const Vec3& q);

//------------------------------------------------------------------------------

///// @brief reparameterize such as ||s'|| < M_PI and Exp(s) == Epx(s')
//Axis Reparameterize(const Axis& s);

/// @brief Exponential mapping
SE3 Exp(const se3& s);

/// @brief fast version of Exp(se3(s, 0))
SE3 ExpAngular(const Axis& s);

///// @brief fast version of Exp(t * s), when |s| = 1
//SE3 ExpAngular(const Axis& s, double t);

/// @brief fast version of Exp(se3(s, 0))
SE3 ExpLinear(const Vec3& s);

///// @brief Log mapping
//se3 Log(const SE3& );

///// @brief Log mapping of rotation part only
///// @note When @f$|LogR(T)| = @pi@f$, Exp(LogR(T) = Exp(-LogR(T)).
///// The implementation returns only the positive one.
//Axis LogR(const SE3& T);

//------------------------------------------------------------------------------

/// @brief get inversion of T
/// @note @f$T^{-1} = (R^T, -R^T p), where T=(R,p)@in SE(3)@f$.
SE3 Inv(const SE3& T);

//------------------------------------------------------------------------------

/// @brief get rotation matrix rotated along x-Eigen_Axis by theta angle.
/// @note theta is represented in radian.
SE3 RotX(double angle);

/// @brief get rotation matrix rotated along y-Eigen_Axis by theta angle.
SE3 RotY(double angle);

/// @brief get rotation matrix rotated along z-Eigen_Axis by theta angle.
SE3 RotZ(double angle);

//------------------------------------------------------------------------------
///// @brief Rectify the rotation part so as that it satifies the orthogonality
///// condition.
/////
///// It is one step of @f$R_{i_1}=1/2(R_i + R_i^{-T})@f$.
///// Hence by calling this function iterativley, you can make the rotation part
///// closer to SO(3).
//SE3 Normalize(const SE3& T);

//------------------------------------------------------------------------------

/// @brief adjoint mapping
/// @note @f$Ad_TV = ( Rw@,, ~p @times Rw + Rv)@f$,
/// where @f$T=(R,p)@in SE(3), @quad V=(w,v)@in se(3) @f$.
se3 AdT(const SE3& T, const se3& V);

/// @brief Fast version of Ad([R 0; 0 1], V)
se3 AdR(const SE3& T, const se3& V);

/// @brief fast version of Ad(T, se3(w, 0))
se3 AdTAngular(const SE3& T, const Axis& w);

/// @brief fast version of Ad(T, se3(0, v))
se3 AdTLinear(const SE3& T, const Vec3& v);

///// @brief fast version of Ad([I p; 0 1], V)
//se3 AdP(const Vec3& p, const se3& s);

/// @brief
Jacobian AdTJac(const SE3& T, const Jacobian& J);

///// @brief fast version of Ad([R 0; 0 1], J)
//Jacobian AdRJac(const SE3& T, const Jacobian& J);

/// @brief fast version of Ad(Inv(T), V)
se3 AdInvT(const SE3& T, const se3& V);

///// @brief fast version of Ad(Inv(T), se3(Eigen_Vec3(0), v))
//Vec3 AdInvTLinear(const SE3& T, const Vec3& v);

///// @brief fast version of Ad(Inv(T), se3(w, Eigen_Vec3(0)))
//Axis AdInvTAngular(const SE3& T, const Axis& w);

///// @brief Fast version of Ad(Inv([R 0; 0 1]), V)
//se3 AdInvR(const SE3& T, const se3& V);

/// @brief Fast version of Ad(Inv([R 0; 0 1]), se3(0, v))
se3 AdInvRLinear(const SE3& T, const Vec3& V);

/// @brief dual adjoint mapping
/// @note @f$Ad^{@,*}_TF = ( R^T (m - p@times f)@,,~ R^T f)@f$, where @f$T=(R,p)@in SE(3), F=(m,f)@in se(3)^*@f$.
dse3 dAdT(const SE3& T, const dse3& F);

///// @brief fast version of Ad(Inv(T), dse3(Eigen_Vec3(0), F))
//dse3 dAdTLinear(const SE3& T, const Vec3& F);

/// @brief fast version of dAd(Inv(T), F)
dse3 dAdInvT(const SE3& T, const dse3& F);

///// @brief fast version of dAd(Inv(SE3(p)), dse3(Eigen_Vec3(0), F))
//dse3 dAdInvPLinear(const Vec3& p, const Vec3& F);

/// @brief adjoint mapping
/// @note @f$ad_X Y = ( w_X @times w_Y@,,~w_X @times v_Y - w_Y @times v_X),@f$,
/// where @f$X=(w_X,v_X)@in se(3), @quad Y=(w_Y,v_Y)@in se(3) @f$.
se3 ad(const se3& X, const se3& Y);

/// @brief fast version of ad(se3(Eigen_Vec3(0), v), S)
//Vec3 ad_Vec3_se3(const Vec3& v, const se3& S);

/// @brief fast version of ad(se3(w, 0), se3(v, 0))	-> check
//Axis ad_Axis_Axis(const Axis& w, const Axis& v);

/// @brief dual adjoint mapping
/// @note @f$ad^{@,*}_V F = (m @times w + f @times v@,,~ f @times w),@f$
/// , where @f$F=(m,f)@in se^{@,*}(3), @quad V=(w,v)@in se(3) @f$.
dse3 dad(const se3& V, const dse3& F);

Inertia Transform(const SE3& T, const Inertia& AI);

bool VerifySE3(const SE3& _T);

#include "math/LieGroupinl.h"

} // namespace math
} // namespace dart

#endif

