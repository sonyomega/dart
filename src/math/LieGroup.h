#ifndef DART_MATH_LIE_GROUP_H
#define DART_MATH_LIE_GROUP_H

#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>
#include <cfloat>

#include <Eigen/Dense>

#include "utils/UtilsCode.h"

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

namespace dart {
namespace math {

class Vec3;
class Axis;
class se3;
class dse3;
class SE3;
class SO3;
class Inertia;
class AInertia;
class Jacobian;

// TODO:
//   1. copy constructors
//   2. explicit check
//   3. naming check
//   4. add Eigen helpers
//   5. remove wired functions
//   6. add missing parameter names
//   7. License!
//   8. default constructer (member variable initialization)
//   9. check if Inertia * se3
//  10. not use [] operator for SE3, Inertia
//  11. random setter
//  12. split LieGroups into multiple files

//------------------------------------------------------------------------------

/// @brief double multiplication
inline Axis operator*(double c, const Axis& p);

/// @brief double multiplicaiton operator
inline se3 operator*(double c, const se3& V);

/// @brief double multiplication operator
inline dse3 operator*(double c, const dse3& F);

//------------------------------------------------------------------------------

/// @brief get a magnitude of p.
inline double Norm(const Vec3& p);

/// @brief get a magnitude of v.
inline double Norm(const Axis& v);

/// @brief get a magnitude of S.
inline double Norm(const se3& S);

/// @brief get a magnitude of F.
inline double Norm(const dse3& F);

/// @brief get a normalized vector from p.
inline Vec3 Normalize(const Vec3& p);

/// @brief get a normalized vector from p.
inline Axis Normalize(const Axis& p);

/// @brief Compute geometric distance on SE(3) manifold.
/// Norm(Log(Inv(T1) * T2)).
inline double Distance(const SE3& T1, const SE3& T2);

//------------------------------------------------------------------------------

/// @brief get a squared sum of all the elements in p.
inline double SquareSum(const Vec3& );

/// @brief get a squared sum of all the elements in p.
inline double SquareSum(const Axis& );

/// @brief get squared sum of all the elements
inline double SquareSum(const se3& S);

/// @brief get a squared sum of all the elements in p.
inline double SquareSum(const dse3& );

//------------------------------------------------------------------------------
/// @brief Get a transformation matrix given by the Euler XYZ angle,
/// where the positional part is set to be zero.
inline SE3 EulerXYZ(const Vec3& angle);

/// @brief get a transformation matrix given by the Euler XYZ angle and
/// position.
inline SE3 EulerXYZ(const Vec3& angle, const Vec3& position);

/// @brief get a transformation matrix given by the Euler ZYX angle,
/// where the positional part is set to be zero.
/// singularity : x[1] = -+ 0.5*PI
/// @sa SE3::iEulerZYX
inline SE3 EulerZYX(const Vec3& angle);

/// @brief get a transformation matrix given by the Euler ZYX angle and
/// position.
/// singularity : x[1] = -+ 0.5*PI
inline SE3 EulerZYX(const Vec3& angle, const Vec3& position);

/// @brief Get a transformation matrix given by the Euler ZYZ angle,
/// where the positional part is set to be zero.
/// singularity : x[1] = 0, PI
/// @sa SE3::iEulerZYZ
inline SE3 EulerZYZ(const Vec3& angle);

/// @brief get a transformation matrix given by the Euler ZYZ angle and
/// position.
/// singularity : x[1] = 0, PI
inline SE3 EulerZYZ(const Vec3& angle, const Vec3& position);

/// @brief get the Euler ZYX angle from T
////// @sa Vec3::EulerXYZ
inline Vec3 iEulerXYZ(const SE3& T);

/// @brief get the Euler ZYX angle from T
/// @sa Vec3::EulerZYX
inline Vec3 iEulerZYX(const SE3& T);

/// @brief get the Euler ZYZ angle from T
/// @sa Vec3::EulerZYZ
inline Vec3 iEulerZYZ(const SE3& T);

/// @brief get the Euler ZYZ angle from T
/// @sa Vec3::EulerZXY
inline Vec3 iEulerZXY(const SE3 &T);

//------------------------------------------------------------------------------

/// @brief fast version of se3(Rotate(T, Vec3(S[0], S[1], S[2])), Rotate(T, Vec3(S[3], S[4], S[5])))
inline se3 Rotate(const SE3& T, const se3& S);

/// @brief rotate q by T.
/// @return @f$R q@f$, where @f$T=(R,p)@f$.
inline Vec3 Rotate(const SE3& T, const Vec3& q);

/// @brief
inline Axis Rotate(const SE3& T, const Axis& v);

/// @brief rotate q by Inv(T).
inline Vec3 InvRotate(const SE3& T, const Vec3& q);


/// @brief fast version of se3(Rotate(Inv(T), Vec3(S[0], S[1], S[2])), Rotate(Inv(T), Vec3(S[3], S[4], S[5])))
inline se3 InvRotate(const SE3& T, const se3& S);

//------------------------------------------------------------------------------

/// @brief reparameterize such as ||s'|| < M_PI and Exp(s) == Epx(s')
inline Axis Reparameterize(const Axis& s);

//------------------------------------------------------------------------------

/// @brief get a cross product of p and q.
inline Vec3 Cross(const Vec3& p, const Vec3& a);

/// @brief get a cross product of p and q.
inline Axis Cross(const Axis& p, const Axis& a);

//------------------------------------------------------------------------------

/// @brief get an inner product of p and q.
inline double Inner(const Vec3& p, const Vec3& a);

/// @brief get an inner product of p and q.
inline double Inner(const Axis& p, const Axis& a);

/// @brief get an inner product of p and q.
inline double Inner(const Vec3& p, const Axis& a);

/// @brief get an inner product of p and q.
inline double Inner(const Axis& p, const Vec3& a);

/// @brief inner product
/// @note @f$ @langle F, V@rangle = @langle V, F@rangle = @langle m,
/// w@rangle + @langle f, v@rangle @f$ ,where @f$F=(m,f)@in se(3)^*,@quad
/// V=(w,v)@in se(3)@f$.
inline double Inner(const se3& V, const dse3& F);

/// @brief inner product
inline double Inner(const dse3& F, const se3& V);

/// @brief fast version of Inner(F, se3(w, 0))
inline double Inner(const dse3& F, const Axis& w);

/// @brief fast version of Inner(F, se3(0, v))
inline double Inner(const dse3& F, const Vec3& v);

//------------------------------------------------------------------------------

/// @brief Exponential mapping
inline SE3 Exp(const se3& );

/// @brief fast version of Exp(se3(s, 0))
inline SE3 Exp(const Axis& s);

/// @brief fast version of Exp(se3(0, v))
inline SE3 Exp(const Vec3& v);

/// @brief fast version of Exp(t * s), when |s| = 1
inline SE3 Exp(const Axis& s, double t);

/// @brief Log mapping
inline se3 Log(const SE3& );

/// @brief Log mapping of rotation part only
/// @note When @f$|LogR(T)| = @pi@f$, Exp(LogR(T) = Exp(-LogR(T)).
/// The implementation returns only the positive one.
inline Axis LogR(const SE3& T);

//------------------------------------------------------------------------------

/// @brief get inversion of T
/// @note @f$T^{-1} = (R^T, -R^T p), where T=(R,p)@in SE(3)@f$.
inline SE3 Inv(const SE3& T);

//------------------------------------------------------------------------------

/// @brief get rotation matrix rotated along x-axis by theta angle.
/// @note theta is represented in radian.
inline SE3 RotX(double angle);

/// @brief get rotation matrix rotated along y-axis by theta angle.
inline SE3 RotY(double angle);

/// @brief get rotation matrix rotated along z-axis by theta angle.
inline SE3 RotZ(double angle);

//------------------------------------------------------------------------------

/// @brief get the first order approximation of T.
/// @note If T is near to an identity, Linearize(T) ~= Log(T).
/// Since it is cheaper than Log, it is recommended to use Linearize
/// rather than Log near identity.
inline se3 Linearize(const SE3& T);

//------------------------------------------------------------------------------
/// @brief Rectify the rotation part so as that it satifies the orthogonality
/// condition.
///
/// It is one step of @f$R_{i_1}=1/2(R_i + R_i^{-T})@f$.
/// Hence by calling this function iterativley, you can make the rotation part
/// closer to SO(3).
inline SE3 Normalize(const SE3& T);

//------------------------------------------------------------------------------

/// @brief convert unit quaternion to SE3
/// @note The first element of q[] is a real part and the last three are
/// imaginary parts. Make sure that q[] is unit quaternion, that is,
/// @f$@sum_i q_i^2 = 1@f$.
inline SE3 Quaternion2SE3(const double q[4]);

/// @brief convert unit quaternion to SE3
/// @note The w is a real part and the last three (x, y, z) are
/// imaginary parts. Make sure that (w, x, y, z) is unit quaternion, that is,
/// @f$@w^2 + y^2 + y^2 + z^2 = 1@f$.
inline SE3 Quaternion2SE3(double w, double x, double y, double z);

//------------------------------------------------------------------------------

/// @brief get inverse of J.
inline AInertia Inv(const Inertia& J);

//------------------------------------------------------------------------------

/// @brief get an inertia of box shaped geometry.
/// @param d desity of the geometry
/// @param sz size of the box
inline Inertia BoxInertia(double d, const Vec3& sz);

/// @brief get an inertia of sphere shaped geometry.
/// @param d desity of the geometry
/// @param r radius of the sphere
inline Inertia SphereInertia(double d, double r);

/// @brief get an inertia of cylindrical geometry.
/// @param d desity of the geometry
/// @param r radius of the cylinder
/// @param h height of the cylinder
inline Inertia CylinderInertia(double d, double r, double h);

/// @brief get an inertia of torus geometry.
/// @param d desity of the geometry
/// @param r1 ring radius of the torus
/// @param r2 tube radius of the torus
inline Inertia TorusInertia(double d, double r1, double r2);

/// @brief The Kronecker product
inline AInertia KroneckerProduct(const dse3& , const dse3& );

//------------------------------------------------------------------------------

/// @brief adjoint mapping
/// @note @f$Ad_TV = ( Rw@,, ~p @times Rw + Rv)@f$,
/// where @f$T=(R,p)@in SE(3), @quad V=(w,v)@in se(3) @f$.
inline se3 Ad(const SE3& T, const se3& V);

/// @brief fast version of Ad(T, se3(w, Vec3(0))
inline se3 Ad(const SE3& T, const Axis& w);

/// @brief fast version of Ad(T, se3(Axis(0), v)
inline se3 Ad(const SE3& T, const Vec3& v);

/// @brief fast version of Ad([R 0; 0 1], V)
inline se3 AdR(const SE3& T, const se3& V);
//inline se3 Ad(const SO3& R, const se3& V);

/// @brief fast version of Ad([I p; 0 1], V)
inline se3 Ad(const Vec3& p, const se3& s);

/// @brief
inline Jacobian Ad(const SE3& T, const Jacobian& J);

/// @brief fast version of Ad([R 0; 0 1], J)
inline Jacobian AdR(const SE3& T, const Jacobian& J);

/// @brief fast version of Ad(Inv(T), V)
inline se3 InvAd(const SE3& T, const se3& V);

/// @brief fast version of Ad(Inv(T), se3(Vec3(0), v))
inline Vec3 InvAd(const SE3& T, const Vec3& v);

/// @brief fast version of Ad(Inv(T), se3(w, Vec3(0)))
inline Axis InvAd(const SE3& T, const Axis& w);

/// @brief Fast version of Ad(Inv([R 0; 0 1]), V)
inline se3 InvAdR(const SE3& T, const se3& V);

/// @brief Fast version of Ad(Inv([R 0; 0 1]), se3(Vec3(0), v))
inline se3 InvAdR(const SE3& T, const Vec3& V);

/// @brief get a linear part of Ad(SE3(-p), V).
inline Vec3 MinusLinearAd(const Vec3& p, const se3& V);

/// @brief dual adjoint mapping
/// @note @f$Ad^{@,*}_TF = ( R^T (m - p@times f)@,,~ R^T f)@f$, where @f$T=(R,p)@in SE(3), F=(m,f)@in se(3)^*@f$.
inline dse3 dAd(const SE3& T, const dse3& F);

/// @brief fast version of Ad(Inv(T), dse3(Vec3(0), F))
inline dse3 dAd(const SE3& T, const Vec3& F);

/// @brief fast version of dAd(Inv(T), F)
inline dse3 InvdAd(const SE3& T, const dse3& F);

/// @brief fast version of dAd(Inv(SE3(p)), dse3(Vec3(0), F))
inline dse3 InvdAd(const Vec3& p, const Vec3& F);

/// @brief adjoint mapping
/// @note @f$ad_X Y = ( w_X @times w_Y@,,~w_X @times v_Y - w_Y @times v_X),@f$,
/// where @f$X=(w_X,v_X)@in se(3), @quad Y=(w_Y,v_Y)@in se(3) @f$.
inline se3 ad(const se3& X, const se3& Y);

/// @brief fast version of ad(se3(Vec3(0), v), S)
inline Vec3 ad(const Vec3& v, const se3& S);

/// @brief fast version of ad(se3(w, 0), se3(v, 0))	-> check
inline Axis ad(const Axis& w, const Axis& v);

/// @brief dual adjoint mapping
/// @note @f$ad^{@,*}_V F = (m @times w + f @times v@,,~ f @times w),@f$
/// , where @f$F=(m,f)@in se^{@,*}(3), @quad V=(w,v)@in se(3) @f$.
inline dse3 dad(const se3& V, const dse3& F);

//------------------------------------------------------------------------------

/// @brief standard output operator
std::ostream& operator<<(std::ostream& os, const Vec3& v);

/// @brief standard output operator
std::ostream& operator<<(std::ostream& os, const Axis& w);

/// @brief standard output operator
std::ostream& operator<<(std::ostream& os, const se3& V);

/// @brief standard output operator
std::ostream& operator<<(std::ostream& os, const dse3& F);

/// @brief standard output operator
std::ostream& operator<<(std::ostream& os, const SE3& T);

//==============================================================================
// Vec3
//==============================================================================
/// @class Vec3
/// @brief 3 dimensional vector
///
/// Vec3 is a class for representing 3 dimensional vector.
class Vec3
{
public:
    //--------------------------------------------------------------------------
    // Constructors and desctructor
    //--------------------------------------------------------------------------
    /// @brief constructor : (0.0, 0.0, 0.0)
    Vec3();

    /// @brief constructor : (c0, c1, c2)
    explicit Vec3(double c0, double c1, double c2);

    /// @brief constructor : (c, c, c)
    explicit Vec3(double c);

    /// @brief constructor : (v[0], v[1], v[2])
    explicit Vec3(const double v[]);

    /// @brief constructor : (v[0], v[1], v[2])
    explicit Vec3(const Eigen::Vector3d& v);

    /// @brief
    ~Vec3();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief unary plus operator
    const Vec3& operator+() const;

    /// @brief unary minus operator
    Vec3 operator-() const;

    /// @brief access to the idx th element.
    double& operator[](int idx);
    const double& operator[](int) const;

    /// @brief substitution operator
    const Vec3& operator=(const Vec3& );

    /// @brief substitute operator
    /// set all the elements to be c.
    const Vec3& operator=(double c);

    /// @brief addition and substitution operator
    const Vec3& operator+=(const Vec3& );

    /// @brief -= operator
    const Vec3& operator-=(const Vec3& );

    /// @brief *= operator
    const Vec3& operator*=(double);

    /// @brief
    bool operator==(const Vec3& T) const;

    /// @brief
    bool operator!=(const Vec3& T) const;

    /// @brief multiplication operator
    Vec3 operator*(double) const;

    /// @brief addition operator
    Vec3 operator+(const Vec3& ) const;

    /// @brief subtraction operator
    Vec3 operator-(const Vec3& ) const;

    //--------------------------------------------------------------------------
    // Getters and setters
    //--------------------------------------------------------------------------
    /// @brief
    void setValues(double x, double y, double z);

    /// @brief normalize the vector.
    /// @return length of the vector.
    double Normalize();

    /// @brief
    Eigen::Vector3d getEigenVector() const;

    //--------------------------------------------------------------------------
    // Friend functions
    //--------------------------------------------------------------------------
    friend class Axis;
    friend class se3;
    friend class dse3;
    friend class SO3;
    friend class SE3;
    friend class Inertia;
    friend class AInertia;

    friend Vec3 operator*(double d, const Vec3& v);
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v);
    friend double Norm(const Vec3& p);
    friend Vec3	Normalize(const Vec3& p);
    friend Vec3	Cross(const Vec3& p, const Vec3& a);
    friend double Inner(const Vec3& p, const Vec3& a);
    friend double Inner(const Vec3& p, const Axis& q);
    friend double Inner(const Axis& p, const Vec3& q);
    friend double Inner(const dse3& f, const Vec3& v);
    friend Axis Square(const Axis& p);
    friend double SquareSum(const Vec3& );
    friend se3  Ad(const SE3& T, const Vec3& v);
    friend se3  Ad(const Vec3& p, const se3& s);
    friend dse3 dAd(const SE3& T, const Vec3& F);
    friend dse3 InvdAd(const Vec3& p, const Vec3& F);
    friend Vec3 InvAd(const SE3& T, const Vec3& v);
    friend se3  InvAdR(const SE3& T, const Vec3& V);
    friend Axis ad(const Axis& s1, const se3& s2);
    friend Axis ad(const Axis& s1, const Axis& s2);
    friend SE3 EulerXYZ(const Vec3& angle);
    friend SE3 EulerZYX(const Vec3& angle);
    friend SE3 EulerZYX(const Vec3& angle, const Vec3& position);
    friend SE3 EulerXYZ(const Vec3& angle, const Vec3& position);
    friend SE3 EulerZYZ(const Vec3& angle);
    friend SE3 EulerZYZ(const Vec3& angle, const Vec3& position);
    friend Vec3 Rotate(const SE3& T, const Vec3& q);
    friend Vec3 InvRotate(const SE3& T, const Vec3& q);
    friend Vec3 ad(const Vec3& v, const se3& S);
    friend Vec3 MinusLinearAd(const Vec3& p, const se3& s);
    friend Inertia BoxInertia(double density, const Vec3& size);

private:
    double _v[3];
};

// 3 dimensional vector but used for angular part of se(3) or dse(3)
class Axis
{
public:
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief constructor : (0.0, 0.0, 0.0)
    Axis();

    /// @brief constructor : (c0, c1, c2)
    explicit Axis(double c0, double c1, double c2);

    /// @brief constructor : (c, c, c)
    explicit Axis(double c);

    /// @brief constructor : (v[0], v[1], v[2])
    explicit Axis(const double v[]);

    /// @brief constructor : (v[0], v[1], v[2])
    explicit Axis(const Vec3& v);

    /// @brief destructor
    ~Axis();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief unary plus operator
    const Axis& operator+() const;

    /// @brief unary minus operator
    Axis operator-() const;

    /// @brief access to the idx th element.
    //    double& operator[](int idx);
    //    const double& operator[](int) const;
    double& operator()(int idx);
    const double& operator()(int) const;

    /// @brief substitution operator
    const Axis& operator=(const Axis& );

    /// @brief fast version of = Axis(s[0], s[1], s[2])
    const Axis& operator=(const se3& );

    /// @brief substitute operator
    /// set all the elements to be c.
    const Axis& operator=(double c);

    /// @brief *= operator
    const Axis& operator *= (double);

    /// @brief multiplication operator
    Axis operator*(double) const;

    /// @brief addition operator
    Axis operator+(const Axis& ) const;

    /// @brief addition operator
    se3 operator+(const Vec3& ) const;

    /// @brief subtraction operator
    Axis operator-(const Axis& ) const;

    /// @brief addition and substitution operator
    const Axis& operator+=(const Axis& );

    /// @brief -= operator
    const Axis& operator-=(const Axis& );

    /// @brief == operator
    bool operator==(const Axis& v) const;

    /// @brief != operator
    bool operator!=(const Axis& v) const;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief normalize the vector.
    /// @return length of the vector.
    double Normalize();

    /// @brief reparameterize such that ||s'|| <= M_PI and Exp(s) == Epx(s')
    void Reparameterize();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    friend class se3;
    friend class dse3;
    friend class Inertia;
    friend class AInertia;

    friend double operator*(const dse3& t, const Axis& s);
    friend Axis operator*(double d, const Axis& v);
    friend std::ostream& operator<<(std::ostream& os, const Axis& v);
    friend double Norm(const Axis& v);
    friend Axis Normalize(const Axis& p);
    friend Axis Reparameterize(const Axis& s);
    friend Axis Cross(const Axis& p, const Axis& a);
    friend double Inner(const Axis& p, const Axis& a);
    friend double Inner(const Vec3& p, const Axis& a);
    friend double Inner(const Axis& p, const Vec3& a);
    friend double Inner(const dse3& F, const Axis& w);
    friend Axis Square(const Axis& p);
    friend double SquareSum(const Axis& );
    friend SE3 Exp(const Axis& S);
    friend SE3 Exp(const Axis& S, double theta);
    friend Axis Rotate(const SE3& T, const Axis& q);
    friend Axis InvRotate(const SE3& T, const Axis& q);
    friend se3 Ad(const SE3& T, const Axis& w);
    friend Axis InvAd(const SE3& T, const Axis& w);
    friend Axis ad(const Axis& w, const Axis& v);
    friend Axis ad(const Axis& s1, const se3& s2);

private:
    double _v[3];
};

typedef Axis so3;
typedef Axis dso3;

/// @class se3
/// @brief Lie algebra of SE(3)
/// se3 is a class for representing @f$se(3)@f$, the Lie algebra of @f$SE(3)@f$.
/// Geometrically it deals with generalized velocity.
/// The first three elements correspond to angular velocity
/// and the last three elements correspond to linear velocity.
class se3
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief constructor : (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    se3();

    /// @brief constructor : (c, c, c, c, c, c)
    explicit se3(double c);

    /// @brief constructor : (c0, c1, c2, c3, c4, c5)
    explicit se3(double c0, double c1, double c2, double c3, double c4, double c5);

    /// @brief constructor : (w[0], w[1], w[2], v[0], v[1], v[2])
    explicit se3(const Axis& w, const Vec3& v);

    /// @brief constructor : (w[0], w[1], w[2], 0.0, 0.0, 0.0)
    explicit se3(const Axis& w);

    /// @brief constructor : (0.0, 0.0, 0.0, v[0], v[1], v[2])
    explicit se3(const Vec3& v);

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief unary plus operator
    const se3 &operator+() const;

    /// @brief unary minus operator
    se3 operator-() const;

    /// @brief substitution operator
    const se3& operator=(const se3& s);

    /// @brief fast version of = se3(w, 0)
    const se3& operator=(const Axis& w);

    /// @brief fast version of = se3(0, v)
    const se3& operator=(const Vec3& v);

    /// @brief substitution operator, fast version of = se3(c)
    const se3& operator=(double c);

    /// @brief += operator
    const se3& operator+=(const se3& s);

    /// @brief fast version of += se3(w, 0)
    const se3& operator+=(const Axis& w);

    /// @brief fast version of += se3(0, v)
    const se3& operator+=(const Vec3& v);

    /// @brief -= operator
    const se3& operator-=(const se3& );

    /// @brief *= operator with double
    const se3& operator*=(double c);

    /// @brief == operator
    bool operator==(const se3& v) const;

    /// @brief != operator
    bool operator!=(const se3& v) const;

    /// @brief addition operator
    se3 operator+(const se3& s) const;

    /// @brief subtraction operator
    se3 operator-(const se3& s) const;

    /// @brief double multiplication operator
    se3 operator*(double) const;

    /// @brief access to the idx th element.
    //    double& operator[](int idx);
    //    const double& operator[](int) const;
    double& operator()(int idx);
    const double& operator()(int) const;

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief set itself to zeros.
    void setZero();

    /// @brief
    void setFromMatrixForm(const Eigen::Matrix4d& mat);

    /// @brief set itself to be Ad(T, V).
    void setAd(const SE3& T, const se3& V);

    /// @brief set itself to be Ad(Inv(T), V).
    void setInvAd(const SE3& T, const se3& V);

    /// @brief set itself to be ad(V, W).
    void setad(const se3& V, const se3& W);

    /// @brief fast version ad(V, se3(W, 0))
    void setad(const se3& V, const Axis& W);

    /// @brief
    void setEigenVector(const Eigen::Matrix<double,6,1>& vec6);

    /// @brief
    Vec3 getAngular() const;

    /// @brief
    Vec3 getLinear() const;

    /// @brief
    Eigen::Matrix<double,6,1> getEigenVector() const;

    /// @brief
    void setEigenMatrix(const Eigen::Matrix4d& mat);

    /// @brief
    Eigen::Matrix4d getEigenMatrix() const;

    //--------------------------------------------------------------------------
    // Friend functions
    //--------------------------------------------------------------------------
    friend class Vec3;
    friend class Axis;
    friend class dse3;
    friend class SE3;
    friend class Inertia;
    friend class AInertia;

    friend double operator*(const dse3& t, const se3& s);
    friend se3 operator*(double d, const se3& s);
    friend std::ostream& operator<<(std::ostream& os, const se3& s);
    friend SE3 Exp(const se3& );
    friend SE3 Exp(const Axis& s);
    friend SE3 Exp(const Axis& s, double t);
    friend se3 Log(const SE3& S);
    friend se3 setAd(const SE3& T, const se3& V);
    friend se3 setInvAd(const SE3& T, const se3& V);
    friend Vec3 MinusLinearAd(const Vec3& p, const se3& V);
    friend se3 setad(const se3& X, const se3& Y);
    friend se3 Ad(const SE3& T, const se3& s);
    friend se3 Ad(const Vec3& p, const se3& s);
    friend se3 AdR(const SE3& T, const se3& s);
    friend se3 InvAd(const SE3& T, const se3& s);
    friend se3 InvAdR(const SE3& T, const se3& s);
    friend se3 ad(const se3& s1, const se3& s2);
    friend Vec3 ad(const Vec3& s1, const se3& s2);
    friend Axis ad(const Axis& s1, const se3& s2);
    friend dse3 dad(const se3& V, const dse3& F);
    friend double Norm(const se3& s);
    friend double Inner(const se3& V, const dse3& F);
    friend double Inner(const dse3& F, const se3& V);
    friend double SquareSum(const se3& S);
    friend se3 Rotate(const SE3& T, const se3& S);
    friend se3 InvRotate(const SE3& T, const se3& S);

private:
    double _w[6];
};

/// @class dse3
/// @brief Dual space of se(3)
///
/// dse3 is a class for representing @f$se(3)^*@f$, a dual of the Lie algebra @f$se(3)@f$.
/// Geometrically it deals with generalized force.
/// The first three elements correspond to moment(or torque)
/// and the last three elements correspond to force.
class dse3
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief constructor : (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    dse3();

    /// @brief constructor : (c, c, c, c, c, c)
    explicit dse3(double c);

    /// @brief constructor : (c0, c1, c2, c3, c4, c5)
    explicit dse3(double c0, double c1, double c2, double c3, double c4, double c5);

    /// @brief constructor : (m[0], m[1], m[2], f[0], f[1], f[2])
    explicit dse3(const Axis& m, const Vec3& f);

    /// @brief constructor : Inertia(mass) * dV
    explicit dse3(double mass, const se3& dV);

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief unary plus operator
    const dse3& operator+() const;

    /// @brief unary minus operator
    dse3 operator-() const;

    /// @brief substitution operator
    const dse3& operator=(const dse3& );

    /// @brief fast version of = dse3(0, f)
    const dse3& operator=(const Vec3& f);

    /// @brief fast version of = dse3(m, 0)
    const dse3& operator=(const Axis& m);

    /// @brief substitution operator, fast version of = dse3(c)
    const dse3& operator=(double c);

    /// @brief += operator
    const dse3& operator += (const dse3& );

    /// @brief	+= operator, fast version of = dse3(m, 0)
    const dse3& operator += (const Axis& m);

    /// @brief -= operator
    const dse3& operator -= (const dse3& );

    /// @brief *= operator
    const dse3& operator *= (double);

    /// @brief addition operator
    dse3 operator+(const dse3& ) const;

    /// @brief subtraction operator
    dse3 operator-(const dse3& ) const;

    /// @brief double multiplication operator
    dse3 operator*(double) const;

    /// @brief access to the idx th element.
    //double& operator[](int idx);
    //const double& operator[](int) const;
    double& operator()(int idx);
    const double& operator()(int) const;

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief Set as zeros.
    void setZero();

    /// @brief
    Eigen::Matrix<double,6,1> getEigenVector() const;

    /// @brief set itself to be dad(V, F).
    void dad(const se3& V, const dse3& F);

    /// @brief set itself to be dAd(T, F).
    void dAd(const SE3& T, const dse3& F);

    //--------------------------------------------------------------------------
    // Friend functions
    //--------------------------------------------------------------------------
    friend class AInertia;

    friend dse3 operator*(double, const dse3& );
    friend double operator*(const dse3& t, const se3& s);
    friend double operator*(const dse3& t, const Axis& s);
    friend double operator*(const dse3& F, const Vec3& V);
    friend std::ostream& operator<<(std::ostream& os, const dse3& t);
    friend dse3 dAd(const SE3& T, const dse3& F);
    friend dse3 InvdAd(const SE3& T, const dse3& F);
    friend dse3 dad(const se3& s, const dse3& t);
    friend double SquareSum(const dse3& );
    friend double Inner(const se3& V, const dse3& F);
    friend double Inner(const dse3& F, const se3& V);
    friend double Inner(const dse3& F, const Axis& w);
    friend double Inner(const dse3& F, const Vec3& v);
    friend double Norm(const dse3& f);
    friend AInertia KroneckerProduct(const dse3& x, const dse3& y);

private:
    double _m[6];
};


/// @class SO3
/// @brief Special Orientation group
/// | R[0]  R[3]  R[6] |
///	| R[1]  R[4]  R[7] |
///	| R[2]  R[5]  R[8] |
class SO3
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief default constructor.
    SO3();

    /// @brief copy constructor.
    SO3(const SO3& v);

    /// @brief
    explicit SO3(double R0, double R1, double R2,	//Rx
                 double R3, double R4, double R5,	//Ry
                 double R6, double R7, double R8);	//Rz

    /// @brief
    explicit SO3(const Vec3 & Rx, const Vec3 & Ry, const Vec3 & Rz);

    /// @brief default destructor.
    ~SO3();

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief Casting operator.
    SO3* operator&() { return this; }

    /// @brief Const Casting operator.
    const SO3* operator&() const { return this; }

    /// @brief access to the i-th element, where it is assumed as an array.
    /// in a matrix form, it looks like
    /// | R[0]	R[3]	R[6] |
    /// | R[1]	R[4]	R[7] |
    /// | R[2]	R[5]	R[8] |
    /// ,where the left 3X3 matrix is the rotation matrix.
    const double	&operator [] (int i) const;
    double			&operator [] (int);

    /// @brief substitution operator.
    const SO3& operator = (const SO3 &);

    /// @brief multiplication operator\n
    /// T_this *= T is a fast version of T_this = T_this * T
    const SO3 & operator *= (const SO3 & R);

    /// @brief multiplication operator\n
    /// T_this /= T is a fast version of T_this =  T_this * Inv(T)
    const SO3 & operator /= (const SO3 & T);

    /// @brief multiplication operator\n
    /// T_this \%= T is a fast version of T_this= Inv(T_this) * T
    const SO3 & operator %= (const SO3 & T);

    /// @brief multiplication operator */
    SO3 operator * (const SO3 & T) const;

    /// @brief multiplication operator, T_this / T = T_this * Inv(T) */
    SO3 operator / (const SO3 & T) const;

    /// @brief multiplication operator, T_this \% T = Inv(T_this) * T */
    SO3 operator % (const SO3 & T) const;

    /// @brief
    Vec3 operator * (const Vec3& q) const;

    /// @brief
    Vec3 operator % (const Vec3& q) const;

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief Set values.
    void setValues(double, double, double,	// Rx
                   double, double, double,	// Ry
                   double, double, double);	// Rz

    /// @brief set itself to be identity.
    void setIdentity(void);

    /// @brief get x-axis. */
    Vec3 getRx(void) const;

    /// @brief get y-axis. */
    Vec3 getRy(void) const;

    /// @brief get z-axis. */
    Vec3 getRz(void) const;

    /// @brief Exponential mapping */
    const SO3& setExp(const Vec3 &);

    /// @brief Exponential mapping for unit length axis. */
    const SO3& setExp(const Vec3 &, double);

private:
    double _R[9];
};

/// @class SE3
/// @brief Special Euclidean group
///
/// SE3 is a class for representing the special Euclidean group.
/// Geometrically, it deals with  rigid transformations on @f$ @mathbb{R}^3 @f$.
/// SE(3) is defined as the set of
/// mappings @f$g: @mathbb{R}^3 @rightarrow @mathbb{R}^3@f$ of the form @f$g(x) = Rx + p@f$,
/// where @f$R@in@f$ the special orthogonal group and @f$p@in @mathbb{R}^3@f$.
/// An element of SE(3), written as (R, p), can also be represented in
/// the matrix form	@f$@begin{bmatrix} R&  p @@ 0&  1@end{bmatrix}.@f$
class SE3
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief
    SE3();

    /// @brief copy constructor
    SE3(const SE3& T);

    /// @brief constructor
    /// rotation part
    explicit SE3(double, double, double,
                 double, double, double,
                 double, double, double);

    /// @brief constructor
    /// rotation and position part
    explicit SE3(double, double, double,
                 double, double, double,
                 double, double, double,
                 double, double, double);

    /// @brief constructor
    /// rotation part = an identity, position part = p
    explicit SE3(const Vec3& p);

    /// @brief constructor
    /// Rx, Ry, Rz = x, y, z axes of rotation matrix, p = position
    explicit SE3(const Vec3& Rx, const Vec3& Ry, const Vec3& Rz, const Vec3& p);

    /// @brief constructor
    /// fast version of SE3(Vec3(c))
    explicit SE3(double c);
    explicit SE3(int c);

    /// @brief constructor
    /// array T is assumed to be column based 4X4 matrix
    explicit SE3(const double T[]);

    /// @brief Destructor.
    ~SE3();

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief get the i-th row and the j-th column element.
    double operator()(int i, int j) const;

    /// @brief access to the i-th element, where it is assumed as an array.
    /// in a matrix form, it looks like
    /// | T[0]	T[3]	T[6]	T[ 9] |
    /// | T[1]	T[4]	T[7]	T[10] |
    /// | T[2]	T[5]	T[8]	T[11] |
    /// ,where the left 3X3 matrix is the rotation matrix and the right 3 vector
    /// is the position.
    const double& operator[](int i) const;
    double& operator[](int i);

    /// @brief substitution operator
    const SE3& operator=(const SE3& T);

    /// @brief substitution operator, fast version of = SE3(p)
    const SE3& operator=(const Vec3& p);

    /// @brief multiplication operator
    SE3 operator*(const SE3& T) const;

    /// @brief multiplication operator, Ta / Tb = Ta * Inv(Tb)
    SE3 operator/(const SE3& T) const;

    /// @brief multiplication operator, Ta @% Tb = Inv(Ta) * Tb
    SE3 operator%(const SE3& T) const;

    /// @brief multiplication operator
    /// @note @f$ T q = R q + p@f$, where @f$T=(R,p)@in SE(3), @quad q@in@mathbb{R}^3@f$.
    Vec3 operator*(const Vec3& p) const;

    /// @brief multiplication operator, T @% p = Inv(T) * p
    Vec3 operator%(const Vec3& p) const;

    /// @brief multiplication operator@n
    /// Ta *= Tb is a fast version of Ta = Ta * Tb
    const SE3& operator*=(const SE3& );

    /// @brief multiplication operator@n
    /// Ta /= Tb is a fast version of Ta = Ta * Inv(Tb)
    const SE3& operator/=(const SE3& );

    /// @brief  multiplication operator@n
    /// Ta @%= Tb is a fast version of Ta = Inv(Ta) * Tb
    const SE3& operator%=(const SE3& );

    /// @brief
    bool operator==(const SE3& T) const;

    /// @brief
    bool operator!=(const SE3& T) const;

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief set itself to be identity.
    void setIdentity();

    /// @brief set rotation part from T and position part from p.
    void setOrientationPosition(const SE3& T, const Vec3& p);

    /// @brief set rotation part only from T
    void setOrientation(const SE3& T);

    /// @brief Set rotation with quaternion values.
    void setOrientationQuaternion(double w, double x, double y, double z);

    /// @brief Get x-axis from orientation part.
    Vec3 getRx(void) const;

    /// @brief Get y-axis from orientation part.
    Vec3 getRy(void) const;

    /// @brief Get z-axis from orientation part.
    Vec3 getRz(void) const;

    /// @brief set position part from p.
    void setPosition(const Vec3& p);

    /// @brief get position part.
    Vec3 getPosition() const;

    /// @brief Fill in the array M
    /// M[0] = T[0]		M[4] = T[3]		M[ 8] = T[6]		M[12] = T[ 9]
    /// M[1] = T[1]		M[5] = T[4]		M[ 9] = T[7]		M[13] = T[10]
    /// M[2] = T[2]		M[6] = T[5]		M[10] = T[8]		M[14] = T[11]
    /// M[3] = 0		M[7] = 0		M[11] = 0			M[15] = 1
    void toDoubleArray(double M[]) const;

    /// @brief
    Eigen::Matrix4d getEigenMatrix() const;
    Eigen::Affine3d getEigenAffine() const;

    /// @brief convert SE3 to unit quaternion
    /// @note The first element of q[] is a real part and the last three are imaginary parts
    ///        In general, unit quaternion is a dual cover of SE3. And it chooses a positive q[0].
    void toQuaternion(double q[4]) const;

    /// @brief rectify rotational part
    /// @note If rotational part does not satisfy the orthogonality condition, correct elements to the closest rotation group element
    void rectify();

    //--------------------------------------------------------------------------
    // Friend functions
    //--------------------------------------------------------------------------
    friend class se3;
    friend class dse3;
    friend class Inertia;
    friend class AInertia;

    friend std::ostream& operator<<(std::ostream& os, const SE3& T);
    friend SE3  Inv(const SE3& T);
    friend SE3  Exp(const se3& S);
    friend se3  Log(const SE3& T);
    friend Axis LogR(const SE3& T);
    friend se3  Linearize(const SE3& T);
    friend Vec3 iEulerXYZ(const SE3& T);
    friend Vec3 iEulerZXY(const SE3& T);
    friend Vec3 iEulerZYX(const SE3& T);
    friend Vec3 iEulerZYZ(const SE3& T);
    friend SE3  Normalize(const SE3& T);
    friend Vec3 Rotate(const SE3& T, const Vec3& v);
    friend se3  Rotate(const SE3& T, const se3& v);
    friend Axis Rotate(const SE3& T, const Axis& v);
    friend Vec3 InvRotate(const SE3& T, const Vec3& v);
    friend se3  InvRotate(const SE3& T, const se3& v);
    friend Axis InvRotate(const SE3& T, const Axis& v);
    friend se3  Ad(const SE3& T, const se3& s);
    friend se3  Ad(const SE3& T, const Axis& s);
    friend se3  Ad(const SE3& T, const Vec3& v);
    friend se3  AdR(const SE3& T, const se3& s);
    friend se3  InvAd(const SE3& T, const se3& s);
    friend Vec3 InvAd(const SE3& T, const Vec3& v);
    friend Axis InvAd(const SE3& T, const Axis& v);
    friend se3  InvAdR(const SE3& T, const se3& s);
    friend se3 InvAdR(const SE3& T, const Vec3& V);
    friend dse3 dAd(const SE3& T, const dse3& t);
    friend dse3 dAd(const SE3& T, const Vec3& f);
    friend dse3 InvdAd(const SE3& T, const dse3& t);

private:
    double _T[12];
};

/// @class Inertia
/// @brief Generalized inertia tensor
///
/// Inertia is a class for representing generalized inertia tensor.
/// Generalized inertia tensor can be expressed as the triple (I, m, r),
/// where @f$m@in@mathbb{R},@,r@in@mathbb{R}^3@f$ and @f$I@in@mathbb{R}^{3@times 3}@f$ is positive
/// definite.
class Inertia
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief Constructor.
    /// Default mass: 1.0.
    /// Default Ixx, Iyy, Izz = 1.0.
    /// Default Ixy, Ixz, Iyz = 0.0.
    Inertia();

    /// @brief constructor
    explicit Inertia(double mass, double Ixx, double Iyy, double Izz);

    /// @brief constructor
    explicit Inertia(double mass,
                     double Ixx, double Iyy, double Izz,
                     double Ixy, double Iyz, double Izx,
                     double r0, double r1, double r2);

    ~Inertia();

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief multiplication operator
    /// @note @f$J V = ( Iw + r@times v,~ mv-r@times w)@in se(3)^*@f$, where @f$J=(I,m,r)@in@f$ Inertia, @f$V=(w,v)@in se(3)@f$.
    dse3 operator*(const se3& V) const;

    /// @brief fast version of * se3(Vec3(0), v))
    dse3 operator*(const Vec3& v) const;

    /// @brief fast version of * se3(v, Vec3(0)))
    dse3 operator*(const Axis& v) const;

    /// @brief addition operator
    Inertia operator+(const Inertia& I) const;

    /// @brief substitution operator
    const Inertia& operator=(const Inertia& I);

    /// @brief access to ith element
    /// Inertia[i] = inertia tensor, i @in [0, 5]
    ///            = offset        , i @in [6, 8]
    ///            = mass          , i = 9
    double& operator[](int);
    const double& operator[](int) const;

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief get coordinate transformed inertia
    Inertia	Transform(const SE3& ) const;

    /// @brief Fill in the array M (double array[36])
    // M = | I - m * [r] * [r]   m * [r] |
    //     |          -m * [r]     m * 1 |
    void toDoubleArray(double M[]) const;

    /// @brief Fill in the Eigen matrix (6 x 6 matrix)
    Eigen::Matrix<double,6,6> toTensor() const;

    /// @brief Set a mass.
    void setMass(double mass);

    /// @brief Get a mass.
    double getMass() const;

    /// @brief
    void setAngularMomentDiag(double Ixx, double Iyy, double Izz);

    /// @brief
    Vec3 getAngularMomentDiag() const;

    /// @brief
    void setAngularMomentOffDiag(double Ixy, double Ixz, double Iyz);

    /// @brief
    void setBoxShapeWithDensity(double density, const Vec3& size);

    /// @brief
    void setSphereShapeWithDensity(double density, double radius);

    /// @brief
    void setCylinderShapeWithDensity(double density, double radius, double height);

    /// @brief
    void setTorusShapeWithDensity(double density, double ring_rad, double tube_rad);

    /// @brief
    void setBoxShapeWithMass(double mass, const Vec3& size);

    /// @brief
    void setSphereShapeWithMass(double mass, double radius);

    /// @brief
    void setCylinderShapeWithMass(double mass, double radius, double height);

    /// @brief
    void setTorusShapeWithMass(double mass, double ring_rad, double tube_rad);

    /// @brief
    Vec3 getAngularMomentOffDiag() const;

    /// @brief Set a Ixx.
    void setIxx(double Ixx);

    /// @brief Set a Ixx.
    void setIyy(double Iyy);

    /// @brief Set a Ixx.
    void setIzz(double Izz);

    /// @brief Set a Ixx.
    void setIxy(double Ixy);

    /// @brief Set a Ixx.
    void setIxz(double Ixz);

    /// @brief Set a Ixx.
    void setIyz(double Iyz);

    /// @brief Get a mass.
    double getIxx() const;

    /// @brief Get a mass.
    double getIyy() const;

    /// @brief Get a mass.
    double getIzz() const;

    /// @brief Get a mass.
    double getIxy() const;

    /// @brief Get a mass.
    double getIxz() const;

    /// @brief Get a mass.
    double getIyz() const;

    /// @brief
    void setOffset(const Vec3& offset);

    /// @brief
    Vec3 getOffset() const;

    //--------------------------------------------------------------------------
    // Friend functions
    //--------------------------------------------------------------------------
    friend AInertia Inv(const Inertia& J);

private:
    double _I[10];
};

/// @class AInertia
///	@brief AInertia is a class for representing articulated inertia tensor.
class AInertia
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief
    AInertia();

    /// @brief
    explicit AInertia(const Inertia& I);

    /// @brief
    explicit AInertia(double);

    /// @brief
    explicit AInertia(double, double, double,
                      double, double, double,
                      double, double, double,
                      double, double, double,
                      double, double, double,
                      double, double, double,
                      double, double, double);

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief
    double& operator[](int);

    /// @brief
    const double& operator[](int) const;

    /// @brief
    const AInertia& operator+() const;

    /// @brief
    AInertia operator-() const;

    /// @brief
    dse3 operator*(const se3& V) const;

    /// @brief
    dse3 operator*(const Vec3& v) const;

    /// @brief
    dse3 operator*(const Axis& w) const;

    /// @brief
    /// assumed to be an inverse of AInertia.
    se3 operator*(const dse3& F) const;

    /// @brief
    AInertia operator+(const AInertia& J) const;

    /// @brief
    AInertia operator+(const Inertia& I) const;

    /// @brief
    AInertia operator-(const AInertia& J) const;

    /// @brief
    AInertia operator-(const Inertia& I) const;

    /// @brief
    const AInertia& operator+=(const AInertia& J);

    /// @brief
    const AInertia& operator+=(const Inertia& I);

    /// @brief
    const AInertia& operator-=(const AInertia& J);

    /// @brief
    const AInertia& operator-=(const Inertia& I);

    /// @brief
    se3 operator%(const dse3& ) const;

    /// @brief
    const AInertia& operator=(const AInertia& J);

    /// @brief
    const AInertia& operator=(const Inertia& I);

    // TODO:
    /// @brief
    const AInertia& operator=(const Eigen::Matrix<double,6,6>& _matrix);

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief
    void SubtractKroneckerProduct(const dse3& , const dse3& );

    /// @brief
    void AddTransform(const AInertia& , const SE3& );

    /// @brief
    AInertia Transform(const SE3& T) const;

    /// @brief
    template <class TYPE>
    void ToArray(TYPE []) const;

private:
    double _J[21];
};

//==============================================================================
/// @class Jacobian
class Jacobian
{
public:
    //--------------------------------------------------------------------------
    // Constructors and destructor
    //--------------------------------------------------------------------------
    /// @brief
    Jacobian();

    /// @brief
    explicit Jacobian(unsigned int _size);

    /// @brief
    //explicit Jacobian(const Eigen::MatrixXd& _J);

    /// @brief
    virtual ~Jacobian();

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
    /// @brief substitution operator
    const Jacobian& operator=(const Jacobian& T);

    /// @brief Access to the idx th element.
    se3& operator[](int _i);

    /// @brief Access to the idx th element.
    const se3& operator[](int _i) const;

    /// @brief
    se3 operator*(const Eigen::VectorXd& _qdot);

    /// @brief
    /// @return Generalized forces.
    Eigen::VectorXd getInnerProduct(const dse3& _F) const;

    /// @brief
    bool operator==(const Jacobian& _rhs) const;

    /// @brief
    bool operator!=(const Jacobian& _rhs) const;

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief
    void setSize(int _size) { mJ.resize(_size); }

    /// @brief
    unsigned int getSize() const { return mJ.size(); }

    /// @brief
    void setFromEigenMatrix(const Eigen::MatrixXd& _J);

    /// @brief
    Eigen::MatrixXd getEigenMatrix() const;

    /// @brief
    void setZero();

    /// @brief
    void setColumn(int _idx, const se3& _J) { mJ[_idx] = _J; }

    /// @brief
    se3 getColumn(int _idx);

    /// @brief
    Jacobian getColumns(int _idx, int _size);

    /// @brief
    //void setLinear(int _idx, Eigen::Vector3d& _Jv) { mJ[_idx].setLinear(_Jv); }

    /// @brief
    //Eigen::Vector3d getLinear(int _idx) const { return mJ[_idx].getLinear(); }

    /// @brief
    //void setAngular(int _idx, so3& _Jw) { mJ[_idx].setAngular(_Jw); }

    /// @brief
    //so3 getAngular(int _idx) const { return mJ[_idx].getAngular(); }


    /// @brief
    // TODO: NOT IMPLEMENTED
    Jacobian getAdjointed(const SE3& _T) const;

    /// @brief
    // TODO: NOT IMPLEMENTED
    Jacobian getAdjointedInv(const SE3& _Tinv) const;

protected:
    /// @brief
    std::vector<se3> mJ;

private:
    friend Jacobian Ad(const SE3& T, const Jacobian& J);
    friend Jacobian AdR(const SE3& T, const Jacobian& J);

};

#include "math/LieGroupinl.h"

} // namespace math
} // namespace dart

#endif

