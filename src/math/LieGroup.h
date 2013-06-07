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

//using namespace std;

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

/// @brief rotate q by T.
/// @return @f$R q@f$, where @f$T=(R,p)@f$.
inline Vec3 Rotate(const SE3& T, const Vec3& q);

/// @brief rotate q by Inv(T).
inline Vec3 InvRotate(const SE3& T, const Vec3& q);

/// @brief fast version of se3(Rotate(T, Vec3(S[0], S[1], S[2])), Rotate(T, Vec3(S[3], S[4], S[5])))
inline se3 Rotate(const SE3& T, const se3& S);

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
    double& operator()(int idx);
    const double& operator()(int) const;

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
//    const double& operator[](int i) const;
//    double& operator[](int i);

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
    explicit Inertia(double Ixx, double Iyy, double Izz,
                     double Ixy, double Iyz, double Izx,
                     double r0, double r1, double r2,
                     double mass);

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

    //--------------------------------------------------------------------------
    // Setters and getters
    //--------------------------------------------------------------------------
    /// @brief
    void SubtractKroneckerProduct(const dse3& , const dse3& );

    /// @brief
    void AddTransform(const AInertia& , const SE3& );

    /// @brief
    AInertia Transform(const SE3& ) const;

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

//#include "math/LieGroup.inl"




inline Vec3::Vec3()
{
    _v[0] = _v[1] = _v[2] = 0.0;
}

inline Vec3::Vec3(double d)
{
    _v[0] = _v[1] = _v[2] = d;
}

inline Vec3::Vec3(const double v[])
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

inline Vec3::Vec3(const Eigen::Vector3d& v)
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

inline Vec3::~Vec3()
{
}

inline Vec3::Vec3(double v0, double v1, double v2)
{
    _v[0] = v0;
    _v[1] = v1;
    _v[2] = v2;
}

inline const Vec3& Vec3::operator+(void) const
{
    return *this;
}

inline Vec3 Vec3::operator-(void) const
{
    return Vec3(-_v[0], -_v[1], -_v[2]);
}

inline double& Vec3::operator()(int i)
{
    return _v[i];
}

inline const double& Vec3::operator()(int i) const
{
    return _v[i];
}

inline const Vec3& Vec3::operator = (const Vec3& v)
{
    _v[0] = v._v[0];
    _v[1] = v._v[1];
    _v[2] = v._v[2];
    return *this;
}

inline const Vec3& Vec3::operator = (double d)
{
    _v[0] = _v[1] = _v[2] = d;
    return *this;
}

inline const Vec3& Vec3::operator += (const Vec3& v)
{
    _v[0] += v._v[0];
    _v[1] += v._v[1];
    _v[2] += v._v[2];
    return *this;
}

inline const Vec3& Vec3::operator-= (const Vec3& v)
{
    _v[0] -= v._v[0];
    _v[1] -= v._v[1];
    _v[2] -= v._v[2];
    return *this;
}

inline const Vec3& Vec3::operator *= (double d)
{
    _v[0] *= d;
    _v[1] *= d;
    _v[2] *= d;
    return *this;
}

inline bool Vec3::operator==(const Vec3& v) const
{
    if ((_v[0] != v._v[0])
            || (_v[1] != v._v[1])
            || (_v[2] != v._v[2]))
        return false;

    return true;
}

inline bool Vec3::operator!=(const Vec3& v) const
{
    return !(*this == v);
}

inline Vec3 Vec3::operator*(double d) const
{
    return Vec3(d * _v[0], d * _v[1], d * _v[2]);
}

inline Vec3 Vec3::operator+(const Vec3& v) const
{
    return Vec3(_v[0] + v._v[0], _v[1] + v._v[1], _v[2] + v._v[2]);
}

inline Vec3 Vec3::operator-(const Vec3& v) const
{
    return Vec3(_v[0] - v._v[0], _v[1] - v._v[1], _v[2] - v._v[2]);
}

inline double Vec3::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
    {
        _v[0] = _v[1] = SCALAR_0;
        _v[2] = SCALAR_1;
    } else
    {
        _v[0] /= mag;
        _v[1] /= mag;
        _v[2] /= mag;
    }
    return mag;
}

inline Eigen::Vector3d Vec3::getEigenVector() const
{
    Eigen::Vector3d vector3d;
    vector3d << _v[0], _v[1], _v[2];

    return vector3d;
}

//==============================================================================
//
//==============================================================================
inline Vec3 Rotate(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v._v[0] + T._T[3] * v._v[1] + T._T[6] * v._v[2],
                T._T[1] * v._v[0] + T._T[4] * v._v[1] + T._T[7] * v._v[2],
                T._T[2] * v._v[0] + T._T[5] * v._v[1] + T._T[8] * v._v[2]);
}

inline Vec3 InvRotate(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

inline Vec3 operator*(double d, const Vec3& v)
{
    return Vec3(d * v._v[0], d * v._v[1], d * v._v[2]);
}

inline double Norm(const Vec3& v)
{
    return sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
}

inline Vec3 Normalize(const Vec3& v)
{
    double mag = sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
        return Vec3(SCALAR_0, SCALAR_0, SCALAR_1);

    mag = SCALAR_1 / mag;
    return Vec3(mag * v._v[0], mag * v._v[1], mag * v._v[2]);
}

inline Vec3 Cross(const Vec3& p, const Vec3& q)
{
    return Vec3(p._v[1] * q._v[2] - p._v[2] * q._v[1],
                p._v[2] * q._v[0] - p._v[0] * q._v[2],
                p._v[0] * q._v[1] - p._v[1] * q._v[0]);
}

inline double Inner(const Vec3& p, const Vec3& q)
{
    return (p._v[0] * q._v[0] + p._v[1] * q._v[1] + p._v[2] * q._v[2]);
}

inline double SquareSum(const Vec3& p)
{
    return (p._v[0] * p._v[0] + p._v[1] * p._v[1] + p._v[2] * p._v[2]);
}

inline Vec3 MinusLinearAd(const Vec3& p, const se3& s)
{
    return Vec3(p._v[2] * s._w[1] - p._v[1] * s._w[2] + s._w[3],
                p._v[0] * s._w[2] - p._v[2] * s._w[0] + s._w[4],
                p._v[1] * s._w[0] - p._v[0] * s._w[1] + s._w[5]);
}

inline Vec3 InvAd(const SE3& T, const Vec3& v)
{
    return Vec3(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

inline Vec3 iEulerXYZ(const SE3& T)
{
    return Vec3(atan2(-T._T[7], T._T[8]),
                atan2( T._T[6], sqrt(T._T[7] * T._T[7] + T._T[8] * T._T[8])),
                atan2(-T._T[3], T._T[0]));
}

inline Vec3 iEulerZYX(const SE3& T)
{
    return Vec3(atan2( T._T[1], T._T[0]),
                atan2(-T._T[2], sqrt(T._T[0] * T._T[0] + T._T[1] * T._T[1])),
                atan2( T._T[5], T._T[8]));
}

inline Vec3 iEulerZYZ(const SE3& T)
{
    return Vec3(atan2(T._T[7], T._T[6]),
                atan2(sqrt(T._T[2] * T._T[2] + T._T[5] * T._T[5]), T._T[8]),
                atan2(T._T[5], -T._T[2]));
}

inline Vec3 iEulerZXY(const SE3& T)
{
    return Vec3(atan2(-T._T[3], T._T[4]),
                atan2( T._T[5], sqrt(T._T[3]*T._T[3]+T._T[4]*T._T[4])),
                atan2(-T._T[2], T._T[8]));
}

inline Vec3 ad(const Vec3& s1, const se3& s2)
{
    return Vec3(s2._w[2] * s1._v[1] - s2._w[1] * s1._v[2],
                s2._w[0] * s1._v[2] - s2._w[2] * s1._v[0],
                s2._w[1] * s1._v[0] - s2._w[0] * s1._v[1]);
}

//==============================================================================
//
//==============================================================================
inline se3::se3()
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = 0.0;
}

inline se3::se3(double k)
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = k;
}

inline se3::se3(double w0, double w1, double w2,
                double w3, double w4, double w5)
{
    _w[0] = w0;
    _w[1] = w1;
    _w[2] = w2;
    _w[3] = w3;
    _w[4] = w4;
    _w[5] = w5;
}

inline se3::se3(const Axis& w, const Vec3& v)
{
    _w[0] = w._v[0];
    _w[1] = w._v[1];
    _w[2] = w._v[2];
    _w[3] = v._v[0];
    _w[4] = v._v[1];
    _w[5] = v._v[2];
}

inline se3::se3(const Axis& w)
{
    _w[0] = w._v[0];
    _w[1] = w._v[1];
    _w[2] = w._v[2];

    _w[3] = _w[4] = _w[5] = 0.0;
}


inline se3::se3(const Vec3& v)
{
    _w[0] = _w[1] = _w[2] = 0.0;

    _w[3] = v._v[0];
    _w[4] = v._v[1];
    _w[5] = v._v[2];
}

inline const se3& se3::operator+(void) const
{
    return *this;
}

inline se3 se3::operator-(void) const
{
    return se3(-_w[0], -_w[1], -_w[2], -_w[3], -_w[4], -_w[5]);
}

inline const se3& se3::operator = (const se3& s)
{
    _w[0] = s._w[0];
    _w[1] = s._w[1];
    _w[2] = s._w[2];
    _w[3] = s._w[3];
    _w[4] = s._w[4];
    _w[5] = s._w[5];
    return *this;
}

inline const se3& se3::operator = (const Vec3& s)
{
    _w[0] = _w[1] = _w[2] = SCALAR_0;
    _w[3] = s._v[0];
    _w[4] = s._v[1];
    _w[5] = s._v[2];
    return *this;
}

inline const se3& se3::operator = (const Axis& s)
{
    _w[0] = s._v[0];
    _w[1] = s._v[1];
    _w[2] = s._v[2];
    _w[3] = _w[4] = _w[5] = SCALAR_0;
    return *this;
}

inline const se3& se3::operator = (double d)
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = d;
    return *this;
}

inline const se3& se3::operator += (const se3& s)
{
    _w[0] += s._w[0];
    _w[1] += s._w[1];
    _w[2] += s._w[2];
    _w[3] += s._w[3];
    _w[4] += s._w[4];
    _w[5] += s._w[5];
    return *this;
}

inline const se3& se3::operator += (const Axis& s)
{
    _w[0] += s._v[0];
    _w[1] += s._v[1];
    _w[2] += s._v[2];
    return *this;
}

inline const se3& se3::operator += (const Vec3& s)
{
    _w[3] += s._v[0];
    _w[4] += s._v[1];
    _w[5] += s._v[2];
    return *this;
}

inline const se3& se3::operator-= (const se3& s)
{
    _w[0] -= s._w[0];
    _w[1] -= s._w[1];
    _w[2] -= s._w[2];
    _w[3] -= s._w[3];
    _w[4] -= s._w[4];
    _w[5] -= s._w[5];
    return *this;
}

inline const se3& se3::operator *= (double d)
{
    _w[0] *= d;
    _w[1] *= d;
    _w[2] *= d;
    _w[3] *= d;
    _w[4] *= d;
    _w[5] *= d;
    return *this;
}

inline bool se3::operator==(const se3& S) const
{
    for (int i = 0; i < 6; ++i)
        if (_w[i] != S._w[i])
            return false;

    return true;
}

inline bool se3::operator!=(const se3& S) const
{
    return !(*this == S);
}

inline se3 se3::operator+(const se3& s) const
{
    return se3(_w[0] + s._w[0], _w[1] + s._w[1], _w[2] + s._w[2],
               _w[3] + s._w[3], _w[4] + s._w[4], _w[5] + s._w[5]);
}

inline se3 se3::operator-(const se3& s) const
{
    return se3(_w[0] - s._w[0], _w[1] - s._w[1], _w[2] - s._w[2],
               _w[3] - s._w[3], _w[4] - s._w[4], _w[5] - s._w[5]);
}

inline se3 se3::operator*(double d) const
{
    return se3(d * _w[0], d * _w[1], d * _w[2],
               d * _w[3], d * _w[4], d * _w[5]);
}

//inline double& se3::operator[](int i)
//{
//    return _w[i];
//}

//inline const double& se3::operator[](int i) const
//{
//    return _w[i];
//}

inline double& se3::operator()(int i)
{
    return _w[i];
}

inline const double& se3::operator()(int i) const
{
    return _w[i];
}

inline void se3::setZero()
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = 0.0;
}

inline void se3::setFromMatrixForm(const Eigen::Matrix4d& mat)
{
    // Assume that _M is 4x4 matrix as:
    // _M = | [w] v |
    //      |   0 0 |
    //    = |   0 -w3  w2  v1 |
    //      |  w3   0 -w1  v2 |
    //      | -w2  w1   0  v3 |
    //      |   0   0   0   0 |

    _w[0] = mat(2,1);
    _w[1] = mat(0,2);
    _w[2] = mat(1,0);
    _w[3] = mat(0,3);
    _w[4] = mat(1,3);
    _w[5] = mat(2,3);
}

// *this = T * s * Inv(T)
inline void se3::setAd(const SE3& T, const se3& s)
{
    _w[0] = T._T[0] * s._w[0] + T._T[3] * s._w[1] + T._T[6] * s._w[2];
    _w[1] = T._T[1] * s._w[0] + T._T[4] * s._w[1] + T._T[7] * s._w[2];
    _w[2] = T._T[2] * s._w[0] + T._T[5] * s._w[1] + T._T[8] * s._w[2];

    _w[3] = T._T[10] * _w[2] - T._T[11] * _w[1]
            + T._T[0] * s._w[3] + T._T[3] * s._w[4] + T._T[6] * s._w[5];
    _w[4] = T._T[11] * _w[0] - T._T[9] * _w[2]
            + T._T[1] * s._w[3] + T._T[4] * s._w[4] + T._T[7] * s._w[5];
    _w[5] = T._T[9] * _w[1] - T._T[10] * _w[0]
            + T._T[2] * s._w[3] + T._T[5] * s._w[4] + T._T[8] * s._w[5];
}

// re = Inv(T) * s * T
inline void se3::setInvAd(const SE3& T, const se3& s)
{
    double _tmp[3] = {	s._w[3] + s._w[1] * T._T[11] - s._w[2] * T._T[10],
                        s._w[4] + s._w[2] * T._T[9]  - s._w[0] * T._T[11],
                        s._w[5] + s._w[0] * T._T[10] - s._w[1] * T._T[9] };
    _w[0] = T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2];
    _w[1] = T._T[3] * s._w[0] + T._T[4] * s._w[1] + T._T[5] * s._w[2];
    _w[2] = T._T[6] * s._w[0] + T._T[7] * s._w[1] + T._T[8] * s._w[2];
    _w[3] = T._T[0] * _tmp[0] + T._T[1] * _tmp[1] + T._T[2] * _tmp[2];
    _w[4] = T._T[3] * _tmp[0] + T._T[4] * _tmp[1] + T._T[5] * _tmp[2];
    _w[5] = T._T[6] * _tmp[0] + T._T[7] * _tmp[1] + T._T[8] * _tmp[2];
}

inline void se3::setad(const se3& s1, const se3& s2)
{
    _w[0] =	s1._w[1] * s2._w[2] - s1._w[2] * s2._w[1];
    _w[1] =	s1._w[2] * s2._w[0] - s1._w[0] * s2._w[2];
    _w[2] =	s1._w[0] * s2._w[1] - s1._w[1] * s2._w[0];

    _w[3] =	s1._w[1] * s2._w[5] - s1._w[2] * s2._w[4]
          - s2._w[1] * s1._w[5] + s2._w[2] * s1._w[4];
    _w[4] =	s1._w[2] * s2._w[3] - s1._w[0] * s2._w[5]
          - s2._w[2] * s1._w[3] + s2._w[0] * s1._w[5];
    _w[5] =	s1._w[0] * s2._w[4] - s1._w[1] * s2._w[3]
          - s2._w[0] * s1._w[4] + s2._w[1] * s1._w[3];
}

inline void se3::setad(const se3& s1, const Axis& s2)
{
    _w[0] =	s1._w[1] * s2._v[2] - s1._w[2] * s2._v[1];
    _w[1] =	s1._w[2] * s2._v[0] - s1._w[0] * s2._v[2];
    _w[2] =	s1._w[0] * s2._v[1] - s1._w[1] * s2._v[0];

    _w[3] =	s2._v[2] * s1._w[4] - s2._v[1] * s1._w[5];
    _w[4] =	s2._v[0] * s1._w[5] - s2._v[2] * s1._w[3];
    _w[5] = s2._v[1] * s1._w[3] - s2._v[0] * s1._w[4];
}

inline void se3::setEigenVector(const Eigen::Matrix<double,6,1>& vec6)
{
    _w[0] = vec6[0];
    _w[1] = vec6[1];
    _w[2] = vec6[2];
    _w[3] = vec6[3];
    _w[4] = vec6[4];
    _w[5] = vec6[5];
}

inline Vec3 se3::getAngular() const
{
    return Vec3(_w[0], _w[1], _w[2]);
}

inline Vec3 se3::getLinear() const
{
    return Vec3(_w[3], _w[4], _w[5]);
}

inline Eigen::Matrix<double,6,1> se3::getEigenVector() const
{
    Eigen::Matrix<double,6,1> vec6;

    vec6 << _w[0], _w[1], _w[2], _w[3], _w[4], _w[5];

    return vec6;
}

inline void se3::setEigenMatrix(const Eigen::Matrix4d& mat)
{
    // Assume that _M is 4x4 matrix as:
    // _M = | [w] v |
    //      |   0 0 |
    //    = |   0 -w3  w2  v1 |
    //      |  w3   0 -w1  v2 |
    //      | -w2  w1   0  v3 |
    //      |   0   0   0   0 |

    _w[0] = mat(2,1);
    _w[1] = mat(0,2);
    _w[2] = mat(1,0);

    _w[3] = mat(0,3);
    _w[4] = mat(1,3);
    _w[5] = mat(2,3);
}

inline Eigen::Matrix4d se3::getEigenMatrix() const
{
    // Assume that _M is 4x4 matrix as:
    // _M = | [w] v |
    //      |   0 0 |
    //    = |   0 -w3  w2  v1 |
    //      |  w3   0 -w1  v2 |
    //      | -w2  w1   0  v3 |
    //      |   0   0   0   0 |

    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();

    mat(2,1) =  _w[0];
    mat(1,2) = -_w[0];

    mat(0,2) =  _w[1];
    mat(2,0) = -_w[1];

    mat(1,0) =  _w[2];
    mat(0,1) = -_w[2];

    mat(0,3) = _w[3];
    mat(1,3) = _w[4];
    mat(2,3) = _w[5];

    return mat;
}

//==============================================================================
//
//==============================================================================
inline se3 operator*(double d, const se3& s)
{
    return se3(d * s._w[0], d * s._w[1], d * s._w[2],
               d * s._w[3], d * s._w[4], d * s._w[5]);
}

inline double operator*(const dse3& t, const se3& s)
{
    return (t._m[0] * s._w[0] + t._m[1] * s._w[1] + t._m[2] * s._w[2]
          + t._m[3] * s._w[3] + t._m[4] * s._w[4] + t._m[5] * s._w[5]);
}

inline double operator*(const dse3& t, const Axis& s)
{
    return (t._m[0] * s._v[0] + t._m[1] * s._v[1] + t._m[2] * s._v[2]);
}

//double operator*(const dse3& t, const Vec3& s)
//{
//    return (t[3] * s[0] + t[4] * s[1] + t[5] * s[2]);
//}

//double operator*(const se3& s, const dse3& t)
//{
//    return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2] + t[3] * s[3] + t[4] * s[4] + t[5] * s[5]);
//}

/*
    T = (R, p) = exp([w, v]), t = ||w||
    v = beta * p + gamma * w + 1 / 2 * cross(p, w)
    , beta = t * (1 + cos(t)) / (2 * sin(t)), gamma = <w, p> * (1 - beta) / t^2
*/
inline se3 Log(const SE3& T)
{
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

inline Axis LogR(const SE3& T)
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
inline se3 Ad(const SE3& T, const se3& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------
    double tmp[3] = {	T._T[0] * s._w[0] + T._T[3] * s._w[1] + T._T[6] * s._w[2],
                        T._T[1] * s._w[0] + T._T[4] * s._w[1] + T._T[7] * s._w[2],
                        T._T[2] * s._w[0] + T._T[5] * s._w[1] + T._T[8] * s._w[2] };
    return se3(	tmp[0], tmp[1], tmp[2],
                T._T[10] * tmp[2] - T._T[11] * tmp[1] + T._T[0] * s._w[3] + T._T[3] * s._w[4] + T._T[6] * s._w[5],
                T._T[11] * tmp[0] - T._T[ 9] * tmp[2] + T._T[1] * s._w[3] + T._T[4] * s._w[4] + T._T[7] * s._w[5],
                T._T[ 9] * tmp[1] - T._T[10] * tmp[0] + T._T[2] * s._w[3] + T._T[5] * s._w[4] + T._T[8] * s._w[5]);
}

inline se3 Ad(const SE3& T, const Axis& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w
    //--------------------------------------------------------------------------
    double tmp[3] = {	T._T[0] * s._v[0] + T._T[3] * s._v[1] + T._T[6] * s._v[2],
                        T._T[1] * s._v[0] + T._T[4] * s._v[1] + T._T[7] * s._v[2],
                        T._T[2] * s._v[0] + T._T[5] * s._v[1] + T._T[8] * s._v[2] };
    return se3(	tmp[0], tmp[1], tmp[2],
                T._T[10] * tmp[2] - T._T[11] * tmp[1],
                T._T[11] * tmp[0] - T._T[9] * tmp[2],
                T._T[9] * tmp[1] - T._T[10] * tmp[0]);
}

inline se3 Ad(const SE3& T, const Vec3& v)
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

inline Jacobian Ad(const SE3& T, const Jacobian& J)
{
    Jacobian AdTJ(J.mJ.size());

    for (int i = 0; i < J.mJ.size(); ++i)
    {
        AdTJ[i] = Ad(T, J.mJ[i]);
    }

    return AdTJ;
}

inline se3 AdR(const SE3& T, const se3& s)
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

inline se3 Ad(const Vec3& p, const se3& s)
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

inline Jacobian AdR(const SE3& T, const Jacobian& J)
{
    Jacobian AdTJ(J.mJ.size());

    for (int i = 0; i < J.mJ.size(); ++i)
    {
        AdTJ[i] = AdR(T, J.mJ[i]);
    }

    return AdTJ;
}

// re = Inv(T) * s * T
inline se3 InvAd(const SE3& T, const se3& s)
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

inline se3 InvAdR(const SE3& T, const se3& s)
{
    return se3(	T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2],
                T._T[3] * s._w[0] + T._T[4] * s._w[1] + T._T[5] * s._w[2],
                T._T[6] * s._w[0] + T._T[7] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * s._w[3] + T._T[1] * s._w[4] + T._T[2] * s._w[5],
                T._T[3] * s._w[3] + T._T[4] * s._w[4] + T._T[5] * s._w[5],
                T._T[6] * s._w[3] + T._T[7] * s._w[4] + T._T[8] * s._w[5]);
}

inline se3 InvAdR(const SE3& T, const Vec3& v)
{
    return se3(	0.0,
                0.0,
                0.0,
                T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[5]);
}

inline se3 ad(const se3& s1, const se3& s2)
{
    return se3(	s1._w[1] * s2._w[2] - s1._w[2] * s2._w[1],
                s1._w[2] * s2._w[0] - s1._w[0] * s2._w[2],
                s1._w[0] * s2._w[1] - s1._w[1] * s2._w[0],
                s1._w[1] * s2._w[5] - s1._w[2] * s2._w[4] - s2._w[1] * s1._w[5] + s2._w[2] * s1._w[4],
                s1._w[2] * s2._w[3] - s1._w[0] * s2._w[5] - s2._w[2] * s1._w[3] + s2._w[0] * s1._w[5],
                s1._w[0] * s2._w[4] - s1._w[1] * s2._w[3] - s2._w[0] * s1._w[4] + s2._w[1] * s1._w[3]);
}

inline double SquareSum(const se3& s)
{
    return (s._w[0] * s._w[0] + s._w[1] * s._w[1] + s._w[2] * s._w[2]
          + s._w[3] * s._w[3] + s._w[4] * s._w[4] + s._w[5] * s._w[5]);
}

inline se3 Rotate(const SE3& T, const se3& s)
{
    return se3(	T._T[0] * s._w[0] + T._T[3] * s._w[1] + T._T[6] * s._w[2],
                T._T[1] * s._w[0] + T._T[4] * s._w[1] + T._T[7] * s._w[2],
                T._T[2] * s._w[0] + T._T[5] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * s._w[3] + T._T[3] * s._w[4] + T._T[6] * s._w[5],
                T._T[1] * s._w[3] + T._T[4] * s._w[4] + T._T[7] * s._w[5],
                T._T[2] * s._w[3] + T._T[5] * s._w[4] + T._T[8] * s._w[5]);
}

inline se3 InvRotate(const SE3& T, const se3& s)
{
    return se3(	T._T[0] * s._w[0] + T._T[1] * s._w[1] + T._T[2] * s._w[2],
                T._T[3] * s._w[0] + T._T[4] * s._w[1] + T._T[5] * s._w[2],
                T._T[6] * s._w[0] + T._T[7] * s._w[1] + T._T[8] * s._w[2],
                T._T[0] * s._w[3] + T._T[1] * s._w[4] + T._T[2] * s._w[5],
                T._T[3] * s._w[3] + T._T[4] * s._w[4] + T._T[5] * s._w[5],
                T._T[6] * s._w[3] + T._T[7] * s._w[4] + T._T[8] * s._w[5]);
}

//==============================================================================
//
//==============================================================================
inline dse3::dse3()
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = 0.0;
}

inline dse3::dse3(double k)
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = k;
}

inline dse3::dse3(double m0, double m1, double m2, double m3, double m4, double m5)
{
    _m[0] = m0;
    _m[1] = m1;
    _m[2] = m2;
    _m[3] = m3;
    _m[4] = m4;
    _m[5] = m5;
}

inline dse3::dse3(const Axis& m, const Vec3& f)
{
    _m[0] = m._v[0];
    _m[1] = m._v[1];
    _m[2] = m._v[2];
    _m[3] = f._v[0];
    _m[4] = f._v[1];
    _m[5] = f._v[2];
}

inline dse3::dse3(double mass, const se3& dV)
{
    _m[0] = mass * dV._w[0];
    _m[1] = mass * dV._w[1];
    _m[2] = mass * dV._w[2];
    _m[3] = mass * dV._w[3];
    _m[4] = mass * dV._w[4];
    _m[5] = mass * dV._w[5];
}

inline const dse3& dse3::operator+(void) const
{
    return *this;
}

inline dse3 dse3::operator-(void) const
{
    return dse3(-_m[0], -_m[1], -_m[2], -_m[3], -_m[4], -_m[5]);
}

inline const dse3& dse3::operator = (const dse3& t)
{
    _m[0] = t._m[0];
    _m[1] = t._m[1];
    _m[2] = t._m[2];
    _m[3] = t._m[3];
    _m[4] = t._m[4];
    _m[5] = t._m[5];
    return *this;
}

inline const dse3& dse3::operator = (const Axis& t)
{
    _m[0] = t._v[0];
    _m[1] = t._v[1];
    _m[2] = t._v[2];
    _m[3] = _m[4] = _m[5] = SCALAR_0;
    return *this;
}

inline const dse3& dse3::operator = (const Vec3& t)
{
    _m[0] = _m[1] =  _m[2] = SCALAR_0;
    _m[3] = t._v[0];
    _m[4] = t._v[1];
    _m[5] = t._v[2];
    return *this;
}

inline const dse3& dse3::operator = (double d)
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = d;
    return *this;
}

inline const dse3& dse3::operator += (const dse3& t)
{
    _m[0] += t._m[0];
    _m[1] += t._m[1];
    _m[2] += t._m[2];
    _m[3] += t._m[3];
    _m[4] += t._m[4];
    _m[5] += t._m[5];
    return *this;
}

inline const dse3& dse3::operator += (const Axis& t)
{
    _m[0] += t._v[0];
    _m[1] += t._v[1];
    _m[2] += t._v[2];
    return *this;
}

inline const dse3& dse3::operator-= (const dse3& t)
{
    _m[0] -= t._m[0];
    _m[1] -= t._m[1];
    _m[2] -= t._m[2];
    _m[3] -= t._m[3];
    _m[4] -= t._m[4];
    _m[5] -= t._m[5];
    return *this;
}

inline const dse3& dse3::operator *= (double d)
{
    _m[0] *= d;
    _m[1] *= d;
    _m[2] *= d;
    _m[3] *= d;
    _m[4] *= d;
    _m[5] *= d;
    return *this;
}

inline dse3 dse3::operator+(const dse3& t) const
{
    return dse3(_m[0] + t._m[0], _m[1] + t._m[1], _m[2] + t._m[2],
                _m[3] + t._m[3], _m[4] + t._m[4], _m[5] + t._m[5]);
}

inline dse3 dse3::operator-(const dse3& t) const
{
    return dse3(_m[0] - t._m[0], _m[1] - t._m[1], _m[2] - t._m[2],
                _m[3] - t._m[3], _m[4] - t._m[4], _m[5] - t._m[5]);
}

inline dse3 dse3::operator*(double d) const
{
    return dse3(d * _m[0], d * _m[1], d * _m[2], d * _m[3], d * _m[4], d * _m[5]);
}

//inline double& dse3::operator[](int i)
//{
//    return _m[i];
//}

//inline const double& dse3::operator[](int i) const
//{
//    return _m[i];
//}

inline double& dse3::operator()(int i)
{
    return _m[i];
}

inline const double& dse3::operator()(int i) const
{
    return _m[i];
}

inline void dse3::setZero()
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = 0.0;
}

inline dse3 dad(const se3& s, const dse3& t)
{
    return dse3(t._m[1] * s._w[2] - t._m[2] * s._w[1] + t._m[4] * s._w[5] - t._m[5] * s._w[4],
                t._m[2] * s._w[0] - t._m[0] * s._w[2] + t._m[5] * s._w[3] - t._m[3] * s._w[5],
                t._m[0] * s._w[1] - t._m[1] * s._w[0] + t._m[3] * s._w[4] - t._m[4] * s._w[3],
                t._m[4] * s._w[2] - t._m[5] * s._w[1],
                t._m[5] * s._w[0] - t._m[3] * s._w[2],
                t._m[3] * s._w[1] - t._m[4] * s._w[0]);
}

inline void dse3::dad(const se3& s, const dse3& t)
{
    _m[0] =	t._m[1] * s._w[2] - t._m[2] * s._w[1] + t._m[4] * s._w[5] - t._m[5] * s._w[4];
    _m[1] =	t._m[2] * s._w[0] - t._m[0] * s._w[2] + t._m[5] * s._w[3] - t._m[3] * s._w[5];
    _m[2] =	t._m[0] * s._w[1] - t._m[1] * s._w[0] + t._m[3] * s._w[4] - t._m[4] * s._w[3];
    _m[3] =	t._m[4] * s._w[2] - t._m[5] * s._w[1];
    _m[4] =	t._m[5] * s._w[0] - t._m[3] * s._w[2];
    _m[5] =	t._m[3] * s._w[1] - t._m[4] * s._w[0];
}

inline void dse3::dAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	t._m[0] - T._T[10] * t._m[5] + T._T[11] * t._m[4],
                        t._m[1] - T._T[11] * t._m[3] + T._T[ 9] * t._m[5],
                        t._m[2] - T._T[ 9] * t._m[4] + T._T[10] * t._m[3] };
    _m[0] = T._T[0] * tmp[0] + T._T[1] * tmp[1] + T._T[2] * tmp[2];
    _m[1] = T._T[3] * tmp[0] + T._T[4] * tmp[1] + T._T[5] * tmp[2];
    _m[2] = T._T[6] * tmp[0] + T._T[7] * tmp[1] + T._T[8] * tmp[2];
    _m[3] = T._T[0] * t._m[3] + T._T[1] * t._m[4] + T._T[2] * t._m[5];
    _m[4] = T._T[3] * t._m[3] + T._T[4] * t._m[4] + T._T[5] * t._m[5];
    _m[5] = T._T[6] * t._m[3] + T._T[7] * t._m[4] + T._T[8] * t._m[5];
}

//==============================================================================
//
//==============================================================================
inline dse3 operator*(double d, const dse3& t)
{
    return dse3(d * t._m[0], d * t._m[1], d * t._m[2], d * t._m[3], d * t._m[4], d * t._m[5]);
}

inline dse3 dAd(const SE3& T, const dse3& t)
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

inline dse3 dAd(const SE3& T, const Vec3& v)
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

inline dse3 InvdAd(const SE3& T, const dse3& t)
{
    double tmp[3] = {	T._T[0] * t._m[3] + T._T[3] * t._m[4] + T._T[6] * t._m[5],
                        T._T[1] * t._m[3] + T._T[4] * t._m[4] + T._T[7] * t._m[5],
                        T._T[2] * t._m[3] + T._T[5] * t._m[4] + T._T[8] * t._m[5] };
    return dse3(T._T[10] * tmp[2] - T._T[11] * tmp[1] + T._T[0] * t._m[0] + T._T[3] * t._m[1] + T._T[6] * t._m[2],
                T._T[11] * tmp[0] - T._T[ 9] * tmp[2] + T._T[1] * t._m[0] + T._T[4] * t._m[1] + T._T[7] * t._m[2],
                T._T[ 9] * tmp[1] - T._T[10] * tmp[0] + T._T[2] * t._m[0] + T._T[5] * t._m[1] + T._T[8] * t._m[2],
                tmp[0], tmp[1], tmp[2]);
}

inline dse3 InvdAd(const Vec3& p, const Vec3& f)
{
    return dse3(p._v[1] * f._v[2] - p._v[2] * f._v[1],
                p._v[2] * f._v[0] - p._v[0] * f._v[2],
                p._v[0] * f._v[1] - p._v[1] * f._v[0],
                f._v[0],
                f._v[1],
                f._v[2]);
}

inline double SquareSum(const dse3& t)
{
    return (t._m[0] * t._m[0] + t._m[1] * t._m[1] + t._m[2] * t._m[2]
          + t._m[3] * t._m[3] + t._m[4] * t._m[4] + t._m[5] * t._m[5]);
}

//==============================================================================
//
//==============================================================================
inline SO3::SO3()
{
    _R[0] = _R[4] = _R[8] = 1.0;
    _R[1] = _R[2] = _R[3] = _R[5] = _R[6] = _R[7] = 0.0;
}

inline SO3::~SO3()
{
}

inline SO3::SO3(const SO3& R)
{
    _R[0] = R._R[0];
    _R[1] = R._R[1];
    _R[2] = R._R[2];
    _R[3] = R._R[3];
    _R[4] = R._R[4];
    _R[5] = R._R[5];
    _R[6] = R._R[6];
    _R[7] = R._R[7];
    _R[8] = R._R[8];
}

inline const SO3 & SO3::operator = (const SO3 &T)
{
    //if(this != &T)
    {
        _R[ 0] = T._R[ 0];
        _R[ 1] = T._R[ 1];
        _R[ 2] = T._R[ 2];
        _R[ 3] = T._R[ 3];
        _R[ 4] = T._R[ 4];
        _R[ 5] = T._R[ 5];
        _R[ 6] = T._R[ 6];
        _R[ 7] = T._R[ 7];
        _R[ 8] = T._R[ 8];
    }
    return (*this);
}

//////////////////////////////////////////////////////////////////////////

inline SO3::SO3(double R0, double R1, double R2,
         double R3, double R4, double R5,
         double R6, double R7, double R8 )
{
    _R[ 0] = R0;
    _R[ 1] = R1;
    _R[ 2] = R2;
    _R[ 3] = R3;
    _R[ 4] = R4;
    _R[ 5] = R5;
    _R[ 6] = R6;
    _R[ 7] = R7;
    _R[ 8] = R8;
}

inline SO3::SO3(const Vec3 &Rx, const Vec3 &Ry, const Vec3 &Rz)
{
    _R[ 0] = Rx._v[0];
    _R[ 1] = Rx._v[1];
    _R[ 2] = Rx._v[2];
    _R[ 3] = Ry._v[0];
    _R[ 4] = Ry._v[1];
    _R[ 5] = Ry._v[2];
    _R[ 6] = Rz._v[0];
    _R[ 7] = Rz._v[1];
    _R[ 8] = Rz._v[2];
}

inline void SO3::setValues(double Rx1, double Rx2, double Rx3,	// Rx
                    double Ry1, double Ry2, double Ry3,	// Ry
                    double Rz1, double Rz2, double Rz3)	// Rz
{
    _R[ 0] = Rx1;
    _R[ 1] = Rx2;
    _R[ 2] = Rx3;
    _R[ 3] = Ry1;
    _R[ 4] = Ry2;
    _R[ 5] = Ry3;
    _R[ 6] = Rz1;
    _R[ 7] = Rz2;
    _R[ 8] = Rz3;
}

inline void SO3::setIdentity()
{
    _R[0] = _R[4] = _R[8] = 1.0;
    _R[1] = _R[2] = _R[3] = _R[5] = _R[6] = _R[7] = 0.0;
}

inline Vec3 SO3::getRx(void) const
{
    return Vec3(_R[0], _R[1], _R[2]);
}

inline Vec3 SO3::getRy(void) const
{
    return Vec3(_R[3], _R[4], _R[5]);
}

inline Vec3 SO3::getRz(void) const
{
    return Vec3(_R[6], _R[7], _R[8]);
}

inline const SO3& SO3::setExp(const Vec3 &S)
{
    double s2[] = { S._v[0] * S._v[0], S._v[1] * S._v[1], S._v[2] * S._v[2] }, theta = sqrt(s2[0] + s2[1] + s2[2]), st_t, ct_t;

    if ( theta < LIE_EPS )
    {
        st_t = 1.0 - theta * theta / (double)6.0;
        ct_t = 0.5 - theta * theta / (double)24.0;
    } else
    {
        st_t = sin(theta) / theta;
        ct_t = (1.0 - cos(theta)) / theta / theta;
    }

    _R[0] = 1.0 - ct_t * (s2[1] + s2[2]);
    _R[1] = ct_t * S._v[0] * S._v[1] + st_t * S._v[2];
    _R[2] = ct_t * S._v[0] * S._v[2] - st_t * S._v[1];
    _R[3] = ct_t * S._v[0] * S._v[1] - st_t * S._v[2];
    _R[4] = 1.0 - ct_t * (s2[0] + s2[2]);
    _R[5] = ct_t * S._v[1] * S._v[2] + st_t * S._v[0];
    _R[6] = ct_t * S._v[0] * S._v[2] + st_t * S._v[1];
    _R[7] = ct_t * S._v[1] * S._v[2] - st_t * S._v[0];
    _R[8] = 1.0 - ct_t * (s2[0] + s2[1]);

    return (*this);
}

inline const SO3& SO3::setExp(const Vec3 & S, double theta)
{
    double s2[] = { S._v[0] * S._v[0], S._v[1] * S._v[1], S._v[2] * S._v[2] };

    if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > LIE_EPS )
    {
        return setExp(theta * S);
    }

    double st = sin(theta),
           vt = 1.0 - cos(theta),
           sts[] = { st * S._v[0], st * S._v[1], st * S._v[2] };

    _R[0] = 1.0 + vt * (s2[0] - 1.0);
    _R[1] = vt * S._v[0] * S._v[1] + sts[2];
    _R[2] = vt * S._v[0] * S._v[2] - sts[1];
    _R[3] = vt * S._v[0] * S._v[1] - sts[2];
    _R[4] = 1.0 + vt * (s2[1] - 1.0);
    _R[5] = vt * S._v[1] * S._v[2] + sts[0];
    _R[6] = vt * S._v[0] * S._v[2] + sts[1];
    _R[7] = vt * S._v[1] * S._v[2] - sts[0];
    _R[8] = 1.0 + vt * (s2[2] - 1.0);

    return (*this);
}

inline const double & SO3::operator [] (int i) const
{
    return _R[i];
}

inline double & SO3::operator [] (int i)
{
    return _R[i];
}


inline const SO3 & SO3::operator *= (const SO3 & R)
{
    double x0, x1, x2;
    x0 = _R[0] * R[0] + _R[3] * R[1] + _R[6] * R[2];
    x1 = _R[0] * R[3] + _R[3] * R[4] + _R[6] * R[5];
    x2 = _R[0] * R[6] + _R[3] * R[7] + _R[6] * R[8];
    _R[0] = x0;	_R[3] = x1;	_R[6] = x2;
    x0 = _R[1] * R[0] + _R[4] * R[1] + _R[7] * R[2];
    x1 = _R[1] * R[3] + _R[4] * R[4] + _R[7] * R[5];
    x2 = _R[1] * R[6] + _R[4] * R[7] + _R[7] * R[8];
    _R[1] = x0;	_R[4] =x1;	_R[7] = x2;
    x0 = _R[2] * R[0] + _R[5] * R[1] + _R[8] * R[2];
    x1 = _R[2] * R[3] + _R[5] * R[4] + _R[8] * R[5];
    x2 = _R[2] * R[6] + _R[5] * R[7] + _R[8] * R[8];
    _R[2] = x0; _R[5] = x1; _R[8] = x2;
    return  *this;
}

inline const SO3 & SO3::operator /= (const SO3 & R)
{
    double x0, x1, x2;
    x0 = _R[0] * R[0] + _R[3] * R[3] + _R[6] * R[6];
    x1 = _R[0] * R[1] + _R[3] * R[4] + _R[6] * R[7];
    x2 = _R[0] * R[2] + _R[3] * R[5] + _R[6] * R[8];
    _R[0] = x0;	_R[3] = x1;	_R[6] = x2;
    x0 = _R[1] * R[0] + _R[4] * R[3] + _R[7] * R[6];
    x1 = _R[1] * R[1] + _R[4] * R[4] + _R[7] * R[7];
    x2 = _R[1] * R[2] + _R[4] * R[5] + _R[7] * R[8];
    _R[1] = x0;	_R[4] =x1;	_R[7] = x2;
    x0 = _R[2] * R[0] + _R[5] * R[3] + _R[8] * R[6];
    x1 = _R[2] * R[1] + _R[5] * R[4] + _R[8] * R[7];
    x2 = _R[2] * R[2] + _R[5] * R[5] + _R[8] * R[8];
    _R[2] = x0; _R[5] = x1; _R[8] = x2;

    return *this;
}

inline const SO3 & SO3::operator %= (const SO3 & R)
{
    double tmp[9] = { _R[0], _R[1], _R[2],
                      _R[3], _R[4], _R[5],
                      _R[6], _R[7], _R[8] };

    _R[0] = tmp[0] * R[0] + tmp[1] * R[1] + tmp[2] * R[2];
    _R[1] = tmp[3] * R[0] + tmp[4] * R[1] + tmp[5] * R[2];
    _R[2] = tmp[6] * R[0] + tmp[7] * R[1] + tmp[8] * R[2];

    _R[3] = tmp[0] * R[3] + tmp[1] * R[4] + tmp[2] * R[5];
    _R[4] = tmp[3] * R[3] + tmp[4] * R[4] + tmp[5] * R[5];
    _R[5] = tmp[6] * R[3] + tmp[7] * R[4] + tmp[8] * R[5];

    _R[6] = tmp[0] * R[6] + tmp[1] * R[7] + tmp[2] * R[8];
    _R[7] = tmp[3] * R[6] + tmp[4] * R[7] + tmp[5] * R[8];
    _R[8] = tmp[6] * R[6] + tmp[7] * R[7] + tmp[8] * R[8];

    return *this;
}

inline SO3 SO3::operator * (const SO3 & R) const
{
    return SO3(	_R[0] * R[0] + _R[3] * R[1] + _R[6] * R[2],
                _R[1] * R[0] + _R[4] * R[1] + _R[7] * R[2],
                _R[2] * R[0] + _R[5] * R[1] + _R[8] * R[2],
                _R[0] * R[3] + _R[3] * R[4] + _R[6] * R[5],
                _R[1] * R[3] + _R[4] * R[4] + _R[7] * R[5],
                _R[2] * R[3] + _R[5] * R[4] + _R[8] * R[5],
                _R[0] * R[6] + _R[3] * R[7] + _R[6] * R[8],
                _R[1] * R[6] + _R[4] * R[7] + _R[7] * R[8],
                _R[2] * R[6] + _R[5] * R[7] + _R[8] * R[8] );
}

inline SO3 SO3::operator / (const SO3 &R) const
{
    return SO3(	_R[0] * R[0] + _R[3] * R[3] + _R[6] * R[6],
                _R[1] * R[0] + _R[4] * R[3] + _R[7] * R[6],
                _R[2] * R[0] + _R[5] * R[3] + _R[8] * R[6],
                _R[0] * R[1] + _R[3] * R[4] + _R[6] * R[7],
                _R[1] * R[1] + _R[4] * R[4] + _R[7] * R[7],
                _R[2] * R[1] + _R[5] * R[4] + _R[8] * R[7],
                _R[0] * R[2] + _R[3] * R[5] + _R[6] * R[8],
                _R[1] * R[2] + _R[4] * R[5] + _R[7] * R[8],
                _R[2] * R[2] + _R[5] * R[5] + _R[8] * R[8] );
}

inline SO3 SO3::operator % (const SO3 &R) const
{
    return SO3(	_R[0] * R[0] + _R[1] * R[1] + _R[2] * R[2],
                _R[3] * R[0] + _R[4] * R[1] + _R[5] * R[2],
                _R[6] * R[0] + _R[7] * R[1] + _R[8] * R[2],
                _R[0] * R[3] + _R[1] * R[4] + _R[2] * R[5],
                _R[3] * R[3] + _R[4] * R[4] + _R[5] * R[5],
                _R[6] * R[3] + _R[7] * R[4] + _R[8] * R[5],
                _R[0] * R[6] + _R[1] * R[7] + _R[2] * R[8],
                _R[3] * R[6] + _R[4] * R[7] + _R[5] * R[8],
                _R[6] * R[6] + _R[7] * R[7] + _R[8] * R[8] );
}

inline Vec3 SO3::operator * (const Vec3 & q) const
{
    return Vec3(_R[0] * q._v[0] + _R[3] * q._v[1] + _R[6] * q._v[2],
                _R[1] * q._v[0] + _R[4] * q._v[1] + _R[7] * q._v[2],
                _R[2] * q._v[0] + _R[5] * q._v[1] + _R[8] * q._v[2] );
}

inline Vec3 SO3::operator % (const Vec3 & q) const
{
    return Vec3(_R[0] * q._v[0] + _R[1] * q._v[1] + _R[2] * q._v[2],
                _R[3] * q._v[0] + _R[4] * q._v[1] + _R[5] * q._v[2],
                _R[6] * q._v[0] + _R[7] * q._v[1] + _R[8] * q._v[2] );
}


//==============================================================================
//
//==============================================================================
inline SE3::SE3()
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = _T[9] = _T[10] = _T[11] = SCALAR_0;
}

inline SE3::SE3(const SE3& T)
{
    _T[0] = T._T[0];
    _T[1] = T._T[1];
    _T[2] = T._T[2];
    _T[3] = T._T[3];
    _T[4] = T._T[4];
    _T[5] = T._T[5];
    _T[6] = T._T[6];
    _T[7] = T._T[7];
    _T[8] = T._T[8];
    _T[9] = T._T[9];
    _T[10] = T._T[10];
    _T[11] = T._T[11];
}

inline SE3::SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10, double T12, double T13, double T14)
{
    _T[0] = T0;
    _T[1] = T1;
    _T[2] = T2;
    _T[3] = T4;
    _T[4] = T5;
    _T[5] = T6;
    _T[6] = T8;
    _T[7] = T9;
    _T[8] = T10;
    _T[9] = T12;
    _T[10] = T13;
    _T[11] = T14;
}

inline SE3::SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10)
{
    _T[0] = T0;
    _T[1] = T1;
    _T[2] = T2;
    _T[3] = T4;
    _T[4] = T5;
    _T[5] = T6;
    _T[6] = T8;
    _T[7] = T9;
    _T[8] = T10;
    _T[9] = _T[10] = _T[11] = SCALAR_0;
}

inline SE3::SE3(const Vec3& p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = p._v[0];
    _T[10] = p._v[1];
    _T[11] = p._v[2];
}

inline SE3::SE3(const Vec3& Rx, const Vec3& Ry, const Vec3& Rz, const Vec3& p)
{
    _T[0] = Rx._v[0];
    _T[1] = Rx._v[1];
    _T[2] = Rx._v[2];

    _T[3] = Ry._v[0];
    _T[4] = Ry._v[1];
    _T[5] = Ry._v[2];

    _T[6] = Rz._v[0];
    _T[7] = Rz._v[1];
    _T[8] = Rz._v[2];

    _T[ 9] = p._v[0];
    _T[10] = p._v[1];
    _T[11] = p._v[2];
}

inline SE3::SE3(double p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = _T[10] = _T[11] = p;
}

inline SE3::SE3(int p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;
    _T[9] = _T[10] = _T[11] = (double)p;
}

inline SE3::SE3(const double T[])
{
    _T[0] = T[0];
    _T[1] = T[1];
    _T[2] = T[2];
    _T[3] = T[4];
    _T[4] = T[5];
    _T[5] = T[6];
    _T[6] = T[8];
    _T[7] = T[9];
    _T[8] = T[10];
    _T[9] = T[12];
    _T[10] = T[13];
    _T[11] = T[14];
}

inline SE3::~SE3()
{
}

inline double SE3::operator()(int i, int j) const
{
    if (i == 3)
        return j == 3 ? SCALAR_1 : SCALAR_0;

    return _T[i + (3 * j)];
}

//inline const double& SE3::operator[](int i) const
//{
//    return _T[i];
//}

//inline double& SE3::operator[](int i)
//{
//    return _T[i];
//}

inline const SE3& SE3::operator=(const SE3& T)
{
    _T[0] = T._T[0];
    _T[1] = T._T[1];
    _T[2] = T._T[2];
    _T[3] = T._T[3];
    _T[4] = T._T[4];
    _T[5] = T._T[5];
    _T[6] = T._T[6];
    _T[7] = T._T[7];
    _T[8] = T._T[8];
    _T[9] = T._T[9];
    _T[10] = T._T[10];
    _T[11] = T._T[11];
    return *this;
}

inline const SE3& SE3::operator=(const Vec3& p)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = SCALAR_0;

    _T[ 9] = p._v[0];
    _T[10] = p._v[1];
    _T[11] = p._v[2];

    return *this;
}

inline SE3 SE3::operator*(const SE3& T) const
{
    return SE3(	_T[0] * T._T[0] + _T[3] * T._T[1] + _T[6] * T._T[2],
                _T[1] * T._T[0] + _T[4] * T._T[1] + _T[7] * T._T[2],
                _T[2] * T._T[0] + _T[5] * T._T[1] + _T[8] * T._T[2],
                _T[0] * T._T[3] + _T[3] * T._T[4] + _T[6] * T._T[5],
                _T[1] * T._T[3] + _T[4] * T._T[4] + _T[7] * T._T[5],
                _T[2] * T._T[3] + _T[5] * T._T[4] + _T[8] * T._T[5],
                _T[0] * T._T[6] + _T[3] * T._T[7] + _T[6] * T._T[8],
                _T[1] * T._T[6] + _T[4] * T._T[7] + _T[7] * T._T[8],
                _T[2] * T._T[6] + _T[5] * T._T[7] + _T[8] * T._T[8],
                _T[ 9] + _T[0] * T._T[9] + _T[3] * T._T[10] + _T[6] * T._T[11],
                _T[10] + _T[1] * T._T[9] + _T[4] * T._T[10] + _T[7] * T._T[11],
                _T[11] + _T[2] * T._T[9] + _T[5] * T._T[10] + _T[8] * T._T[11] );
}

inline SE3 SE3::operator / (const SE3& T) const
{
    double tmp[] = {_T[0] * T._T[0] + _T[3] * T._T[3] + _T[6] * T._T[6],
                    _T[1] * T._T[0] + _T[4] * T._T[3] + _T[7] * T._T[6],
                    _T[2] * T._T[0] + _T[5] * T._T[3] + _T[8] * T._T[6],
                    _T[0] * T._T[1] + _T[3] * T._T[4] + _T[6] * T._T[7],
                    _T[1] * T._T[1] + _T[4] * T._T[4] + _T[7] * T._T[7],
                    _T[2] * T._T[1] + _T[5] * T._T[4] + _T[8] * T._T[7],
                    _T[0] * T._T[2] + _T[3] * T._T[5] + _T[6] * T._T[8],
                    _T[1] * T._T[2] + _T[4] * T._T[5] + _T[7] * T._T[8],
                    _T[2] * T._T[2] + _T[5] * T._T[5] + _T[8] * T._T[8] };
    return SE3(	tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8],
                _T[ 9] - tmp[0] * T._T[9] - tmp[3] * T._T[10] - tmp[6] * T._T[11],
                _T[10] - tmp[1] * T._T[9] - tmp[4] * T._T[10] - tmp[7] * T._T[11],
                _T[11] - tmp[2] * T._T[9] - tmp[5] * T._T[10] - tmp[8] * T._T[11] );
}

inline SE3 SE3::operator % (const SE3& T) const
{
    return SE3(	_T[0] * T._T[0] + _T[1] * T._T[1] + _T[2] * T._T[2],
                _T[3] * T._T[0] + _T[4] * T._T[1] + _T[5] * T._T[2],
                _T[6] * T._T[0] + _T[7] * T._T[1] + _T[8] * T._T[2],
                _T[0] * T._T[3] + _T[1] * T._T[4] + _T[2] * T._T[5],
                _T[3] * T._T[3] + _T[4] * T._T[4] + _T[5] * T._T[5],
                _T[6] * T._T[3] + _T[7] * T._T[4] + _T[8] * T._T[5],
                _T[0] * T._T[6] + _T[1] * T._T[7] + _T[2] * T._T[8],
                _T[3] * T._T[6] + _T[4] * T._T[7] + _T[5] * T._T[8],
                _T[6] * T._T[6] + _T[7] * T._T[7] + _T[8] * T._T[8],
                _T[0] * (T._T[9] - _T[9]) + _T[1] * (T._T[10] - _T[10]) + _T[2] * (T._T[11] - _T[11]),
                _T[3] * (T._T[9] - _T[9]) + _T[4] * (T._T[10] - _T[10]) + _T[5] * (T._T[11] - _T[11]),
                _T[6] * (T._T[9] - _T[9]) + _T[7] * (T._T[10] - _T[10]) + _T[8] * (T._T[11] - _T[11]) );
}

inline Vec3 SE3::operator % (const Vec3& p) const
{
    return Vec3(_T[0] * (p._v[0] - _T[9]) + _T[1] * (p._v[1] - _T[10]) + _T[2] * (p._v[2] - _T[11]),
                _T[3] * (p._v[0] - _T[9]) + _T[4] * (p._v[1] - _T[10]) + _T[5] * (p._v[2] - _T[11]),
                _T[6] * (p._v[0] - _T[9]) + _T[7] * (p._v[1] - _T[10]) + _T[8] * (p._v[2] - _T[11]) );
}

inline Vec3 SE3::operator*(const Vec3& p) const
{
    return Vec3(_T[9] + _T[0] * p._v[0] + _T[3] * p._v[1] + _T[6] * p._v[2],
                _T[10] + _T[1] * p._v[0] + _T[4] * p._v[1] + _T[7] * p._v[2],
                _T[11] + _T[2] * p._v[0] + _T[5] * p._v[1] + _T[8] * p._v[2] );
}

inline const SE3& SE3::operator*=(const SE3& T)
{
    double x0, x1, x2;

    _T[9] += _T[0] * T._T[9] + _T[3] * T._T[10] + _T[6] * T._T[11];
    _T[10] += _T[1] * T._T[9] + _T[4] * T._T[10] + _T[7] * T._T[11];
    _T[11] += _T[2] * T._T[9] + _T[5] * T._T[10] + _T[8] * T._T[11];

    x0 = _T[0] * T._T[0] + _T[3] * T._T[1] + _T[6] * T._T[2];
    x1 = _T[0] * T._T[3] + _T[3] * T._T[4] + _T[6] * T._T[5];
    x2 = _T[0] * T._T[6] + _T[3] * T._T[7] + _T[6] * T._T[8];

    _T[0] = x0;	_T[3] = x1;	_T[6] = x2;

    x0 = _T[1] * T._T[0] + _T[4] * T._T[1] + _T[7] * T._T[2];
    x1 = _T[1] * T._T[3] + _T[4] * T._T[4] + _T[7] * T._T[5];
    x2 = _T[1] * T._T[6] + _T[4] * T._T[7] + _T[7] * T._T[8];

    _T[1] = x0;	_T[4] =x1;	_T[7] = x2;

    x0 = _T[2] * T._T[0] + _T[5] * T._T[1] + _T[8] * T._T[2];
    x1 = _T[2] * T._T[3] + _T[5] * T._T[4] + _T[8] * T._T[5];
    x2 = _T[2] * T._T[6] + _T[5] * T._T[7] + _T[8] * T._T[8];

    _T[2] = x0; _T[5] = x1; _T[8] = x2;

    return  *this;
}

inline const SE3& SE3::operator /= (const SE3& T)
{
    double tmp[] = {_T[0] * T._T[0] + _T[3] * T._T[3] + _T[6] * T._T[6],
                    _T[1] * T._T[0] + _T[4] * T._T[3] + _T[7] * T._T[6],
                    _T[2] * T._T[0] + _T[5] * T._T[3] + _T[8] * T._T[6],
                    _T[0] * T._T[1] + _T[3] * T._T[4] + _T[6] * T._T[7],
                    _T[1] * T._T[1] + _T[4] * T._T[4] + _T[7] * T._T[7],
                    _T[2] * T._T[1] + _T[5] * T._T[4] + _T[8] * T._T[7],
                    _T[0] * T._T[2] + _T[3] * T._T[5] + _T[6] * T._T[8],
                    _T[1] * T._T[2] + _T[4] * T._T[5] + _T[7] * T._T[8],
                    _T[2] * T._T[2] + _T[5] * T._T[5] + _T[8] * T._T[8] };
    _T[0] = tmp[0]; _T[1] = tmp[1]; _T[2] = tmp[2];
    _T[3] = tmp[3]; _T[4] = tmp[4]; _T[5] = tmp[5];
    _T[6] = tmp[6]; _T[7] = tmp[7]; _T[8] = tmp[8],
    _T[ 9] -= tmp[0] * T._T[9] + tmp[3] * T._T[10] + tmp[6] * T._T[11];
    _T[10] -= tmp[1] * T._T[9] + tmp[4] * T._T[10] + tmp[7] * T._T[11];
    _T[11] -= tmp[2] * T._T[9] + tmp[5] * T._T[10] + tmp[8] * T._T[11];
    return *this;
}

inline const SE3& SE3::operator %= (const SE3& T)
{
    double tmp[12] = { _T[0], _T[1], _T[2],
                       _T[3], _T[4], _T[5],
                       _T[6], _T[7], _T[8],
                       T._T[9] - _T[9], T._T[10] - _T[10], T._T[11] - _T[11] };
    _T[0] = tmp[0] * T._T[0] + tmp[1] * T._T[1] + tmp[2] * T._T[2];
    _T[1] = tmp[3] * T._T[0] + tmp[4] * T._T[1] + tmp[5] * T._T[2];
    _T[2] = tmp[6] * T._T[0] + tmp[7] * T._T[1] + tmp[8] * T._T[2];
    _T[3] = tmp[0] * T._T[3] + tmp[1] * T._T[4] + tmp[2] * T._T[5];
    _T[4] = tmp[3] * T._T[3] + tmp[4] * T._T[4] + tmp[5] * T._T[5];
    _T[5] = tmp[6] * T._T[3] + tmp[7] * T._T[4] + tmp[8] * T._T[5];
    _T[6] = tmp[0] * T._T[6] + tmp[1] * T._T[7] + tmp[2] * T._T[8];
    _T[7] = tmp[3] * T._T[6] + tmp[4] * T._T[7] + tmp[5] * T._T[8];
    _T[8] = tmp[6] * T._T[6] + tmp[7] * T._T[7] + tmp[8] * T._T[8];
    _T[9] = tmp[0] * tmp[9] + tmp[1] * tmp[10] + tmp[2] * tmp[11];
    _T[10] = tmp[3] * tmp[9] + tmp[4] * tmp[10] + tmp[5] * tmp[11];
    _T[11] = tmp[6] * tmp[9] + tmp[7] * tmp[10] + tmp[8] * tmp[11];
    return *this;
}

inline bool SE3::operator==(const SE3& T) const
{
    for (int i = 0; i < 12; ++i)
        if (_T[i] != T._T[i])
            return false;

    return true;
}

inline bool SE3::operator!=(const SE3& T) const
{
    return !(*this == T);
}

inline void SE3::setIdentity(void)
{
    _T[0] = _T[4] = _T[8] = SCALAR_1;
    _T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = _T[9] = _T[10] = _T[11] = SCALAR_0;
}

inline void SE3::setOrientationPosition(const SE3& T, const Vec3& p)
{
    _T[0] = T._T[0];
    _T[1] = T._T[1];
    _T[2] = T._T[2];

    _T[3] = T._T[3];
    _T[4] = T._T[4];
    _T[5] = T._T[5];

    _T[6] = T._T[6];
    _T[7] = T._T[7];
    _T[8] = T._T[8];

    _T[ 9] = p._v[0];
    _T[10] = p._v[1];
    _T[11] = p._v[2];
}

inline void SE3::setOrientation(const SE3& T)
{
    _T[0] = T._T[0];
    _T[1] = T._T[1];
    _T[2] = T._T[2];

    _T[3] = T._T[3];
    _T[4] = T._T[4];
    _T[5] = T._T[5];

    _T[6] = T._T[6];
    _T[7] = T._T[7];
    _T[8] = T._T[8];
}

inline void SE3::setOrientationQuaternion(double w, double x, double y, double z)
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

    _T[0] = SCALAR_1 - SCALAR_2 * (q22 + q33);
    _T[1] = SCALAR_2 * (q12 + q03);
    _T[2] = SCALAR_2 * (q31 - q02);

    _T[3] = SCALAR_2 * (q12 - q03);
    _T[4] = SCALAR_1 - SCALAR_2 * (q11 + q33);
    _T[5] = SCALAR_2 * (q23 + q01);

    _T[6] = SCALAR_2 * (q31 + q02);
    _T[7] = SCALAR_2 * (q23 - q01);
    _T[8] = SCALAR_1 - SCALAR_2 * (q11 + q22);
}

inline void SE3::setPosition(const Vec3& Pos)
{
    _T[ 9] = Pos._v[0];
    _T[10] = Pos._v[1];
    _T[11] = Pos._v[2];
}

inline Vec3 SE3::getPosition(void) const
{
    return Vec3(_T[9], _T[10], _T[11]);
}

inline void SE3::toDoubleArray(double M[]) const
{
    M[0] = _T[0];
    M[1] = _T[1];
    M[2] = _T[2];
    M[3] = SCALAR_0;
    M[4] = _T[3];
    M[5] = _T[4];
    M[6] = _T[5];
    M[7] = SCALAR_0;
    M[8] = _T[6];
    M[9] = _T[7];
    M[10] = _T[8];
    M[11] = SCALAR_0;
    M[12] = _T[9];
    M[13] = _T[10];
    M[14] = _T[11];
    M[15] = SCALAR_1;
}

inline Eigen::Matrix4d SE3::getEigenMatrix() const
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();

    M(0,0) = _T[0];
    M(1,0) = _T[1];
    M(2,0) = _T[2];

    M(0,1) = _T[3];
    M(1,1) = _T[4];
    M(2,1) = _T[5];

    M(0,2) = _T[6];
    M(1,2) = _T[7];
    M(2,2) = _T[8];

    M(0,3) = _T[9];
    M(1,3) = _T[10];
    M(2,3) = _T[11];

    return M;
}

inline Eigen::Affine3d SE3::getEigenAffine() const
{
    Eigen::Affine3d M = Eigen::Affine3d::Identity();

    M(0,0) = _T[0];
    M(1,0) = _T[1];
    M(2,0) = _T[2];

    M(0,1) = _T[3];
    M(1,1) = _T[4];
    M(2,1) = _T[5];

    M(0,2) = _T[6];
    M(1,2) = _T[7];
    M(2,2) = _T[8];

    M(0,3) = _T[9];
    M(1,3) = _T[10];
    M(2,3) = _T[11];

    return M;
}

inline void SE3::rectify(void)
{
    double _S[9];
    while ( true )
    {
        double idet = SCALAR_1 / (_T[0] * _T[4] * _T[8] - _T[0] * _T[7] * _T[5] - _T[1] * _T[3] * _T[8] + _T[1] * _T[6] * _T[5] + _T[2] * _T[3] * _T[7] - _T[2] * _T[6] * _T[4]);

        _S[0] = idet * (_T[4] * _T[8] - _T[7] * _T[5]);
        _S[1] = idet * (_T[6] * _T[5] - _T[3] * _T[8]);
        _S[2] = idet * (_T[3] * _T[7] - _T[6] * _T[4]);
        _S[3] = idet * (_T[7] * _T[2] - _T[1] * _T[8]);
        _S[4] = idet * (_T[0] * _T[8] - _T[6] * _T[2]);
        _S[5] = idet * (_T[6] * _T[1] - _T[0] * _T[7]);
        _S[6] = idet * (_T[1] * _T[5] - _T[4] * _T[2]);
        _S[7] = idet * (_T[3] * _T[2] - _T[0] * _T[5]);
        _S[8] = idet * (_T[0] * _T[4] - _T[3] * _T[1]);

        _T[0] = SCALAR_1_2 * (_T[0] + _S[0]);
        _T[1] = SCALAR_1_2 * (_T[1] + _S[1]);
        _T[2] = SCALAR_1_2 * (_T[2] + _S[2]);
        _T[3] = SCALAR_1_2 * (_T[3] + _S[3]);
        _T[4] = SCALAR_1_2 * (_T[4] + _S[4]);
        _T[5] = SCALAR_1_2 * (_T[5] + _S[5]);
        _T[6] = SCALAR_1_2 * (_T[6] + _S[6]);
        _T[7] = SCALAR_1_2 * (_T[7] + _S[7]);
        _T[8] = SCALAR_1_2 * (_T[8] + _S[8]);

        if ( abs(_S[0] - _T[0]) + abs(_S[1] - _T[1]) + abs(_S[2] - _T[2]) + abs(_S[3] - _T[3]) + abs(_S[4] - _T[4]) + abs(_S[5] - _T[5]) + abs(_S[6] - _T[6]) + abs(_S[7] - _T[7]) + abs(_S[8] - _T[8]) < LIE_EPS ) break;
    }
}

inline void SE3::toQuaternion(double q[4]) const
{
    q[0] = SCALAR_1_2 * sqrt(_T[0] + _T[4] + _T[8] + SCALAR_1);
    double demon = SCALAR_1_4 / q[0];
    q[1] = demon * (_T[5] - _T[7]);
    q[2] = demon * (_T[6] - _T[2]);
    q[3] = demon * (_T[1] - _T[3]);
}

//==============================================================================
//
//==============================================================================
inline SE3 EulerZYX(const Vec3& angle)
{
    double ca = cos(angle._v[0]), sa = sin(angle._v[0]), cb = cos(angle._v[1]), sb = sin(angle._v[1]), cg = cos(angle._v[2]), sg = sin(angle._v[2]);
    return SE3(ca * cb, sa * cb, -sb, ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg, ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg);
}

inline SE3 EulerZYZ(const Vec3& angle)
{
    double ca = cos(angle._v[0]), sa = sin(angle._v[0]), cb = cos(angle._v[1]), sb = sin(angle._v[1]), cg = cos(angle._v[2]), sg = sin(angle._v[2]);
    return SE3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb);
}

inline SE3 EulerXYZ(const Vec3& angle, const Vec3& pos)
{
    SE3 T = RotX(angle._v[0]) * RotY(angle._v[1]) * RotZ(angle._v[2]);
    T.setPosition(pos);
    return T;
}

inline SE3 EulerZYX(const Vec3& angle, const Vec3& pos)
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

inline SE3 EulerZYZ(const Vec3& angle, const Vec3& pos)
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
inline SE3 Exp(const se3& s)
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
inline SE3 Exp(const Axis& S)
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

inline SE3 Exp(const Vec3& S)
{
    return SE3(S);
}

// I + sin(t) * [S] + (1 - cos(t)) * [S]^2,, where |S| = 1
inline SE3 Exp(const Axis& S, double theta)
{
    double s2[] = { S._v[0] * S._v[0], S._v[1] * S._v[1], S._v[2] * S._v[2] };

    if ( abs(s2[0] + s2[1] + s2[2] - SCALAR_1) > LIE_EPS ) return Exp(theta * S);

    double s3[] = { S._v[0] * S._v[1], S._v[1] * S._v[2], S._v[2] * S._v[0] };
    double alpha = sin(theta), cos_t = cos(theta), beta = SCALAR_1 - cos_t;

    return SE3( beta * s2[0] + cos_t,           beta * s3[0] + alpha * S._v[2], beta * s3[2] - alpha * S._v[1],
                beta * s3[0] - alpha * S._v[2], beta * s2[1] + cos_t,           beta * s3[1] + alpha * S._v[0],
                beta * s3[2] + alpha * S._v[1], beta * s3[1] - alpha * S._v[0], beta * s2[2] + cos_t);
}

inline SE3 Inv(const SE3& T)
{
    return SE3(	T._T[0], T._T[3], T._T[6],
                T._T[1], T._T[4], T._T[7],
                T._T[2], T._T[5], T._T[8],

               -T._T[0] * T._T[9] - T._T[1] * T._T[10] - T._T[2] * T._T[11],
               -T._T[3] * T._T[9] - T._T[4] * T._T[10] - T._T[5] * T._T[11],
               -T._T[6] * T._T[9] - T._T[7] * T._T[10] - T._T[8] * T._T[11]);
}

inline SE3 RotX(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(SCALAR_1, SCALAR_0, SCALAR_0, SCALAR_0, c, s, SCALAR_0, -s, c);
}

inline SE3 RotY(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(c, SCALAR_0, -s, SCALAR_0, SCALAR_1, SCALAR_0, s, SCALAR_0, c);
}

inline SE3 RotZ(double t)
{
    double c = cos(t), s = sin(t);
    return SE3(c, s, SCALAR_0, -s, c, SCALAR_0, SCALAR_0, SCALAR_0, SCALAR_1);
}

// invskew(T - I)
inline se3 Linearize(const SE3& T)
{
    return se3(SCALAR_1_2 * (T._T[5] - T._T[7]),
               SCALAR_1_2 * (T._T[6] - T._T[2]),
               SCALAR_1_2 * (T._T[1] - T._T[3]),
               T._T[9], T._T[10], T._T[11]);
}

inline SE3 Normalize(const SE3& T)
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

inline SE3 Quaternion2SE3(const double q[4])
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

inline SE3 Quaternion2SE3(double w, double x, double y, double z)
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

//==============================================================================
//
//==============================================================================
inline Inertia::Inertia()
{
    _I[9] = 1.0;
    _I[0] = _I[1] = _I[2] = 1.0;
    _I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = SCALAR_0;
}

inline Inertia::Inertia(double mass, double Ixx, double Iyy, double Izz)
{
    assert(mass >= 0 && Ixx >= 0 && Iyy >= 0 && Izz >= 0
          && "A mass of this inertia tensor is not positive definate.");

    _I[9] = mass;
    _I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz;
    _I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = SCALAR_0;
}

inline Inertia::Inertia(double Ixx, double Iyy, double Izz,
                        double Ixy, double Iyz, double Izx,
                        double r0, double r1, double r2,
                        double mass)
{
    _I[0] = Ixx;
    _I[1] = Iyy;
    _I[2] = Izz;

    _I[3] = Ixy;
    _I[4] = Izx;
    _I[5] = Iyz;

    _I[6] = r0;
    _I[7] = r1;
    _I[8] = r2;

    _I[9] = mass;
}

inline Inertia::~Inertia()
{
}

inline Inertia Inertia::Transform(const SE3& T) const
{
    double j0 = _I[0] + _I[9] * T._T[11] * T._T[11] + _I[9] * T._T[10] * T._T[10] - SCALAR_2 * _I[8] * T._T[11] - SCALAR_2 * _I[7] * T._T[10];
    double j1 = _I[1] + _I[9] * T._T[11] * T._T[11] + _I[9] * T._T[ 9] * T._T[ 9] - SCALAR_2 * _I[8] * T._T[11] - SCALAR_2 * _I[6] * T._T[ 9];
    double j2 = _I[2] + _I[9] * T._T[10] * T._T[10] + _I[9] * T._T[ 9] * T._T[ 9] - SCALAR_2 * _I[7] * T._T[10] - SCALAR_2 * _I[6] * T._T[ 9];
    double j3 = _I[3] + T._T[10] * _I[6] + T._T[ 9] * _I[7] - _I[9] * T._T[10] * T._T[ 9];
    double j4 = _I[4] + T._T[11] * _I[6] + T._T[ 9] * _I[8] - _I[9] * T._T[11] * T._T[ 9];
    double j5 = _I[5] + T._T[11] * _I[7] + T._T[10] * _I[8] - _I[9] * T._T[11] * T._T[10];
    double t0 = T._T[0] * j0 + T._T[1] * j3 + T._T[2] * j4;
    double t1 = T._T[3] * j0 + T._T[4] * j3 + T._T[5] * j4;
    double t2 = T._T[6] * j0 + T._T[7] * j3 + T._T[8] * j4;
    double t3 = T._T[0] * j3 + T._T[1] * j1 + T._T[2] * j5;
    double t4 = T._T[3] * j3 + T._T[4] * j1 + T._T[5] * j5;
    double t5 = T._T[6] * j3 + T._T[7] * j1 + T._T[8] * j5;
    double t6 = T._T[0] * j4 + T._T[1] * j5 + T._T[2] * j2;
    double t7 = T._T[3] * j4 + T._T[4] * j5 + T._T[5] * j2;
    double t8 = T._T[6] * j4 + T._T[7] * j5 + T._T[8] * j2;

    return Inertia(	t0 * T._T[0] + t3 * T._T[1] + t6 * T._T[2],
                    t1 * T._T[3] + t4 * T._T[4] + t7 * T._T[5],
                    t2 * T._T[6] + t5 * T._T[7] + t8 * T._T[8],
                    t1 * T._T[0] + t4 * T._T[1] + t7 * T._T[2],
                    t2 * T._T[0] + t5 * T._T[1] + t8 * T._T[2],
                    t2 * T._T[3] + t5 * T._T[4] + t8 * T._T[5],
                    T._T[0] * (_I[6] - _I[9] * T._T[9]) + T._T[1] * (_I[7] - _I[9] * T._T[10]) + T._T[2] * (_I[8] - _I[9] * T._T[11]),
                    T._T[3] * (_I[6] - _I[9] * T._T[9]) + T._T[4] * (_I[7] - _I[9] * T._T[10]) + T._T[5] * (_I[8] - _I[9] * T._T[11]),
                    T._T[6] * (_I[6] - _I[9] * T._T[9]) + T._T[7] * (_I[7] - _I[9] * T._T[10]) + T._T[8] * (_I[8] - _I[9] * T._T[11]),
            _I[9]);
}

inline dse3 Inertia::operator*(const se3& s) const
{
    // M = I * w + m * (r X v - r X (r X w))
    // F = m * (v - r X w)

    // rw <-- r X w
    double rw0 =_I[7] * s._w[2] - _I[8] * s._w[1];
    double rw1 =_I[8] * s._w[0] - _I[6] * s._w[2];
    double rw2 =_I[6] * s._w[1] - _I[7] * s._w[0];

    // R <-- r X v - r X (r x w)
    double R0 = (_I[7] * s._w[5] - _I[8] * s._w[4]) - (_I[7] * rw2 - _I[8] * rw1);
    double R1 = (_I[8] * s._w[3] - _I[6] * s._w[5]) - (_I[8] * rw0 - _I[6] * rw2);
    double R2 = (_I[6] * s._w[4] - _I[7] * s._w[3]) - (_I[6] * rw1 - _I[7] * rw0);

    // M = I * w + m * R
    // F = m * (v - rw)
    return dse3(_I[0] * s._w[0] + _I[3] * s._w[1] + _I[4] * s._w[2] + _I[9] * R0, // M[0]
                _I[3] * s._w[0] + _I[1] * s._w[1] + _I[5] * s._w[2] + _I[9] * R1, // M[1]
                _I[4] * s._w[0] + _I[5] * s._w[1] + _I[2] * s._w[2] + _I[9] * R2, // M[2]

                _I[9] * (s._w[3] - rw0),									 // F[0]
                _I[9] * (s._w[4] - rw1),									 // F[1]
                _I[9] * (s._w[5] - rw2));									 // F[2]
}

inline dse3 Inertia::operator*(const Axis& s) const
{
    return dse3(_I[0] * s._v[0] + _I[3] * s._v[1] + _I[4] * s._v[2],
                _I[3] * s._v[0] + _I[1] * s._v[1] + _I[5] * s._v[2],
                _I[4] * s._v[0] + _I[5] * s._v[1] + _I[2] * s._v[2],
                s._v[1] * _I[8] - s._v[2] * _I[7],
                s._v[2] * _I[6] - s._v[0] * _I[8],
                s._v[0] * _I[7] - s._v[1] * _I[6]);
}

inline dse3 Inertia::operator*(const Vec3& s) const
{
    return dse3(_I[7] * s._v[2] - _I[8] * s._v[1],
                _I[8] * s._v[0] - _I[6] * s._v[2],
                _I[6] * s._v[1] - _I[7] * s._v[0],
                _I[9] * s._v[0], _I[9] * s._v[1], _I[9] * s._v[2]);
}

inline double& Inertia::operator[](int i)
{
    return _I[i];
}

inline const double	&Inertia::operator[](int i) const
{
    return _I[i];
}

inline void Inertia::toDoubleArray(double M[]) const
{
    // G = | I - m * [r] * [r]   m * [r] |
    //     |          -m * [r]     m * 1 |

    // m * r
    double mr0 = _I[9] * _I[6];
    double mr1 = _I[9] * _I[7];
    double mr2 = _I[9] * _I[8];

    // m * [r] * [r]
    double mr0r0 = mr0 * _I[6];
    double mr1r1 = mr1 * _I[7];
    double mr2r2 = mr2 * _I[8];
    double mr0r1 = mr0 * _I[7];
    double mr1r2 = mr1 * _I[8];
    double mr2r0 = mr2 * _I[6];

    M[0] =  _I[0] + mr1r1 + mr2r2;   M[6]  =  _I[3] - mr0r1;           M[12] =  _I[4] - mr2r0;           M[18] =    0.0;   M[24] =   -mr2;   M[30] =    mr1;
    M[1] =  M[6];                    M[7]  =  _I[1] + mr2r2 + mr0r0;   M[13] =  _I[5] - mr1r2;           M[19] =    mr2;   M[25] =    0.0;   M[31] =   -mr0;
    M[2] =  M[12];                   M[8]  =  M[13];                   M[14] =  _I[2] + mr0r0 + mr1r1;   M[20] =   -mr1;   M[26] =    mr0;   M[32] =    0.0;

    M[3] =   0.0;                    M[9]  =    mr2;                   M[15] =   -mr1;                   M[21] =  _I[9];   M[27] =    0.0;   M[33] =    0.0;
    M[4] =  -mr2;                    M[10] =    0.0;                   M[16] =    mr0;                   M[22] =    0.0;   M[28] =  _I[9];   M[34] =    0.0;
    M[5] =   mr1;                    M[11] =   -mr0;                   M[17] =    0.0;                   M[23] =    0.0;   M[29] =    0.0;   M[35] =  _I[9];
}

inline Eigen::Matrix<double,6,6> Inertia::toTensor() const
{
    Eigen::Matrix<double,6,6> M;

    // G = | I - m * [r] * [r]   m * [r] |
    //     |          -m * [r]     m * 1 |

    // m * r
    double mr0 = _I[9] * _I[6];
    double mr1 = _I[9] * _I[7];
    double mr2 = _I[9] * _I[8];

    // m * [r] * [r]
    double mr0r0 = mr0 * _I[6];
    double mr1r1 = mr1 * _I[7];
    double mr2r2 = mr2 * _I[8];
    double mr0r1 = mr0 * _I[7];
    double mr1r2 = mr1 * _I[8];
    double mr2r0 = mr2 * _I[6];

    M(0,0) =  _I[0] + mr1r1 + mr2r2;   M(0,1) =  _I[3] - mr0r1;           M(0,2) =  _I[4] - mr2r0;           M(0,3) =    0.0;   M(0,4) =   -mr2;   M(0,5) =    mr1;
    M(1,0) =  M(0,1);                  M(1,1) =  _I[1] + mr2r2 + mr0r0;   M(1,2) =  _I[5] - mr1r2;           M(1,3) =    mr2;   M(1,4) =    0.0;   M(1,5) =   -mr0;
    M(2,0) =  M(0,2);                  M(2,1) =  M(1,2);                  M(2,2) =  _I[2] + mr0r0 + mr1r1;   M(2,3) =   -mr1;   M(2,4) =    mr0;   M(2,5) =    0.0;

    M(3,0) =   0.0;                    M(3,1) =    mr2;                   M(3,2) =   -mr1;                   M(3,3) =  _I[9];   M(3,4) =    0.0;   M(3,5) =    0.0;
    M(4,0) =  -mr2;                    M(4,1) =    0.0;                   M(4,2) =    mr0;                   M(4,3) =    0.0;   M(4,4) =  _I[9];   M(4,5) =    0.0;
    M(5,0) =   mr1;                    M(5,1) =   -mr0;                   M(5,2) =    0.0;                   M(5,3) =    0.0;   M(5,4) =    0.0;   M(5,5) =  _I[9];

    return M;
}

inline const Inertia& Inertia::operator=(const Inertia& I)
{
    _I[0] = I._I[0];
    _I[1] = I._I[1];
    _I[2] = I._I[2];
    _I[3] = I._I[3];
    _I[4] = I._I[4];
    _I[5] = I._I[5];
    _I[6] = I._I[6];
    _I[7] = I._I[7];
    _I[8] = I._I[8];
    _I[9] = I._I[9];

    return *this;
}

inline Inertia Inertia::operator+(const Inertia& I) const
{
    dterr << "Not implemented.\n";
    return Inertia();
}

inline void Inertia::setMass(double mass)
{
    assert(0.0 <= mass);

    _I[9] = mass;
}

inline double Inertia::getMass(void) const
{
    return _I[9];
}

inline void Inertia::setAngularMomentDiag(double Ixx, double Iyy, double Izz)
{
    setIxx(Ixx);
    setIyy(Iyy);
    setIzz(Izz);
}

inline Vec3 Inertia::getAngularMomentDiag() const
{
    return Vec3(getIxx(), getIyy(), getIzz());
}

inline void Inertia::setAngularMomentOffDiag(double Ixy, double Ixz, double Iyz)
{
    setIxy(Ixy);
    setIxz(Ixz);
    setIyz(Iyz);
}

inline Vec3 Inertia::getAngularMomentOffDiag() const
{
    return Vec3(getIxy(), getIxz(), getIxz());
}

inline void Inertia::setIxx(double Ixx)
{
    _I[0] = Ixx;
}

inline void Inertia::setIyy(double Iyy)
{
    _I[1] = Iyy;
}

inline void Inertia::setIzz(double Izz)
{
    _I[2] = Izz;
}

inline void Inertia::setIxy(double Ixy)
{
    _I[3] = Ixy;
}

inline void Inertia::setIxz(double Ixz)
{
    _I[4] = Ixz;
}

inline void Inertia::setIyz(double Iyz)
{
    _I[5] = Iyz;
}

inline double Inertia::getIxx() const
{
    return _I[0];
}

inline double Inertia::getIyy() const
{
    return _I[1];
}

inline double Inertia::getIzz() const
{
    return _I[2];
}

inline double Inertia::getIxy() const
{
    return _I[3];
}

inline double Inertia::getIxz() const
{
    return _I[4];
}

inline double Inertia::getIyz() const
{
    return _I[5];
}

inline void Inertia::setOffset(const Vec3& offset)
{
    _I[6] = offset._v[0];
    _I[7] = offset._v[1];
    _I[8] = offset._v[2];
}

inline Vec3 Inertia::getOffset() const
{
    return Vec3(_I[6], _I[7], _I[8]);
}

inline Inertia BoxInertia(double density, const Vec3& size)
{
    double mass = (double)8.0 * density * size._v[0] * size._v[1] * size._v[2];
    double ix = mass * (size._v[1] * size._v[1] + size._v[2] * size._v[2]) / SCALAR_3;
    double iy = mass * (size._v[0] * size._v[0] + size._v[2] * size._v[2]) / SCALAR_3;
    double iz = mass * (size._v[0] * size._v[0] + size._v[1] * size._v[1]) / SCALAR_3;
    return Inertia(mass, ix, iy, iz);
}

inline Inertia SphereInertia(double density, double rad)
{
    rad *= rad;
    double mass = density * M_PI * rad;
    double i = (double)0.4 * mass * rad;
    return Inertia(mass, i, i, i);
}

inline Inertia CylinderInertia(double density, double rad, double height)
{
    rad *= rad;
    double mass = density * M_PI * rad * height;
    double ix = mass * height * height  / (double)12.0 + SCALAR_1_4 * mass * rad;
    double iy = ix;
    double iz = SCALAR_1_2 * mass * rad;
    return Inertia(mass, ix, iy, iz);
}

inline Inertia TorusInertia(double density, double ring_rad, double tube_rad)
{
    double mass = density * SCALAR_2 * M_PI_SQR * ring_rad * tube_rad * tube_rad;
    double ix = mass * ((double)0.625 * tube_rad * tube_rad + SCALAR_1_2 * ring_rad + ring_rad);
    double iy = ix;
    double iz = mass * ((double)0.75 * tube_rad * tube_rad + ring_rad + ring_rad);
    return Inertia(mass, ix, iy, iz);
}

//==============================================================================
//
//==============================================================================
inline AInertia::AInertia()
{
    // Off-diagoanl terms
    _J[1] = _J[2] = _J[3] = _J[4]  = _J[5]  =
    _J[7] = _J[8] = _J[9] = _J[10] =
    _J[12] = _J[13] = _J[14] =
    _J[16] = _J[17] =
    _J[19] = 0.0;

    // Diagonal terms: Ixx, Iyy, Izz, mass are all 1.0.
    _J[0] = _J[6] = _J[11] = _J[15] = _J[18] = _J[20] = 1.0;
}

inline AInertia::AInertia(double d)
{
    _J[0] = _J[1] = _J[2] = _J[3] = _J[4] = _J[5] = _J[6] = _J[7] = _J[8] = _J[9] = _J[10] = _J[11] = _J[12] = _J[13] = _J[14] = _J[15] = _J[16] = _J[17] = _J[18] = _J[19] = _J[20] = d;
}

inline AInertia::AInertia(const Inertia& I)
{
    _J[0] = I[0];	_J[1] = I[3];	_J[2] = I[4];	_J[3] = SCALAR_0;	_J[4] = -I[8];		_J[5] = I[7];
                    _J[6] = I[1];	_J[7] = I[5];	_J[8] = I[8];		_J[9] = SCALAR_0;	_J[10] = -I[6];
                                    _J[11] = I[2];	_J[12] = -I[7];		_J[13] = I[6];		_J[14] = SCALAR_0;
                                                    _J[15] = I[9];		_J[16] = SCALAR_0;	_J[17] = SCALAR_0;
                                                                        _J[18] = I[9];		_J[19] = SCALAR_0;
                                                                                            _J[20] = I[9];
}

inline AInertia::AInertia(double a0, double a1, double a2,
                          double a3, double a4, double a5,
                          double a6, double a7, double a8,
                          double a9, double a10, double a11,
                          double a12, double a13, double a14,
                          double a15, double a16, double a17,
                          double a18, double a19, double a20)
{
    _J[0] = a0;		_J[1] = a1;		_J[2] = a2;
    _J[3] = a3;		_J[4] = a4;		_J[5] = a5;
    _J[6] = a6;		_J[7] = a7;		_J[8] = a8;
    _J[9] = a9;		_J[10] = a10;	_J[11] = a11;
    _J[12] = a12;	_J[13] = a13;	_J[14] = a14;
    _J[15] = a15;	_J[16] = a16;	_J[17] = a17;
    _J[18] = a18;	_J[19] = a19;	_J[20] = a20;
}

inline const AInertia& AInertia::operator+(void) const
{
    return *this;
}

inline AInertia AInertia::operator-(void) const
{
    return AInertia(-_J[0], -_J[1], -_J[2], -_J[3], -_J[4], -_J[5], -_J[6], -_J[7], -_J[8], -_J[9], -_J[10], -_J[11], -_J[12], -_J[13], -_J[14], -_J[15], -_J[16], -_J[17], -_J[18], -_J[19], -_J[20]);
}

inline dse3 AInertia::operator*(const se3& a) const
{
    return dse3(_J[0] * a._w[0] + _J[1] * a._w[1] + _J[2] * a._w[2] + _J[3] * a._w[3] + _J[4] * a._w[4] + _J[5] * a._w[5],
                _J[1] * a._w[0] + _J[6] * a._w[1] + _J[7] * a._w[2] + _J[8] * a._w[3] + _J[9] * a._w[4] + _J[10] * a._w[5],
                _J[2] * a._w[0] + _J[7] * a._w[1] + _J[11] * a._w[2] + _J[12] * a._w[3] + _J[13] * a._w[4] + _J[14] * a._w[5],
                _J[3] * a._w[0] + _J[8] * a._w[1] + _J[12] * a._w[2] + _J[15] * a._w[3] + _J[16] * a._w[4] + _J[17] * a._w[5],
                _J[4] * a._w[0] + _J[9] * a._w[1] + _J[13] * a._w[2] + _J[16] * a._w[3] + _J[18] * a._w[4] + _J[19] * a._w[5],
                _J[5] * a._w[0] + _J[10] * a._w[1] + _J[14] * a._w[2] + _J[17] * a._w[3] + _J[19] * a._w[4] + _J[20] * a._w[5]);
}

inline dse3 AInertia::operator*(const Vec3& a) const
{
    return dse3(_J[3] * a._v[0] + _J[4] * a._v[1] + _J[5] * a._v[2],
                _J[8] * a._v[0] + _J[9] * a._v[1] + _J[10] * a._v[2],
                _J[12] * a._v[0] + _J[13] * a._v[1] + _J[14] * a._v[2],
                _J[15] * a._v[0] + _J[16] * a._v[1] + _J[17] * a._v[2],
                _J[16] * a._v[0] + _J[18] * a._v[1] + _J[19] * a._v[2],
                _J[17] * a._v[0] + _J[19] * a._v[1] + _J[20] * a._v[2]);
}

inline dse3 AInertia::operator*(const Axis& a) const
{
    return dse3(_J[0] * a._v[0] + _J[1] * a._v[1] + _J[2] * a._v[2],
                _J[1] * a._v[0] + _J[6] * a._v[1] + _J[7] * a._v[2],
                _J[2] * a._v[0] + _J[7] * a._v[1] + _J[11] * a._v[2],
                _J[3] * a._v[0] + _J[8] * a._v[1] + _J[12] * a._v[2],
                _J[4] * a._v[0] + _J[9] * a._v[1] + _J[13] * a._v[2],
                _J[5] * a._v[0] + _J[10] * a._v[1] + _J[14] * a._v[2]);
}

inline double& AInertia::operator[](int i)
{
    return _J[i];
}

inline const double	&AInertia::operator[](int i) const
{
    return _J[i];
}

inline AInertia AInertia::operator+(const AInertia& J) const
{
    return AInertia(_J[0] + J[0], _J[1] + J[1], _J[2] + J[2], _J[3] + J[3], _J[4] + J[4], _J[5] + J[5], _J[6] + J[6], _J[7] + J[7], _J[8] + J[8], _J[9] + J[9], _J[10] + J[10], _J[11] + J[11], _J[12] + J[12], _J[13] + J[13], _J[14] + J[14], _J[15] + J[15], _J[16] + J[16], _J[17] + J[17], _J[18] + J[18], _J[19] + J[19], _J[20] + J[20]);
}

inline AInertia AInertia::operator+(const Inertia& J) const
{
    return AInertia(_J[0] + J[0], _J[1] + J[3], _J[2] + J[4], _J[3], _J[4] - J[8], _J[5] + J[7], _J[6] + J[1], _J[7] + J[5], _J[8] + J[8], _J[9], _J[10] - J[6], _J[11] + J[2], _J[12] - J[7], _J[13] + J[6], _J[14], _J[15] + J[9], _J[16], _J[17], _J[18] + J[9], _J[19], _J[20] + J[9]);
}

inline AInertia AInertia::operator-(const AInertia& J) const
{
    return AInertia(_J[0] - J[0], _J[1] - J[1], _J[2] - J[2], _J[3] - J[3], _J[4] - J[4], _J[5] - J[5], _J[6] - J[6], _J[7] - J[7], _J[8] - J[8], _J[9] - J[9], _J[10] - J[10], _J[11] - J[11], _J[12] - J[12], _J[13] - J[13], _J[14] - J[14], _J[15] - J[15], _J[16] - J[16], _J[17] - J[17], _J[18] - J[18], _J[19] - J[19], _J[20] - J[20]);
}

inline AInertia AInertia::operator-(const Inertia& J) const
{
    return AInertia(_J[0] - J[0], _J[1] - J[3], _J[2] - J[4], _J[3], _J[4] + J[8], _J[5] - J[7], _J[6] - J[1], _J[7] - J[5], _J[8] - J[8], _J[9], _J[10] + J[6], _J[11] - J[2], _J[12] + J[7], _J[13] - J[6], _J[14], _J[15] - J[9], _J[16], _J[17], _J[18] - J[9], _J[19], _J[20] - J[9]);
}

inline const AInertia& AInertia::operator += (const AInertia& J)
{
    _J[0] += J[0]; _J[1] += J[1]; _J[2] += J[2]; _J[3] += J[3]; _J[4] += J[4]; _J[5] += J[5]; _J[6] += J[6]; _J[7] += J[7]; _J[8] += J[8]; _J[9] += J[9]; _J[10] += J[10]; _J[11] += J[11]; _J[12] += J[12]; _J[13] += J[13]; _J[14] += J[14]; _J[15] += J[15]; _J[16] += J[16]; _J[17] += J[17]; _J[18] += J[18]; _J[19] += J[19]; _J[20] += J[20];
    return *this;
}

inline const AInertia& AInertia::operator-= (const AInertia& J)
{
    _J[0] -= J[0]; _J[1] -= J[1]; _J[2] -= J[2]; _J[3] -= J[3]; _J[4] -= J[4]; _J[5] -= J[5]; _J[6] -= J[6]; _J[7] -= J[7]; _J[8] -= J[8]; _J[9] -= J[9]; _J[10] -= J[10]; _J[11] -= J[11]; _J[12] -= J[12]; _J[13] -= J[13]; _J[14] -= J[14]; _J[15] -= J[15]; _J[16] -= J[16]; _J[17] -= J[17]; _J[18] -= J[18]; _J[19] -= J[19]; _J[20] -= J[20];
    return *this;
}

inline const AInertia& AInertia::operator += (const Inertia& J)
{
    _J[0] += J[0]; _J[1] += J[3]; _J[2] += J[4]; _J[4] -= J[8]; _J[5] += J[7]; _J[6] += J[1]; _J[7] += J[5]; _J[8] += J[8]; _J[10] -= J[6]; _J[11] += J[2]; _J[12] -= J[7]; _J[13] += J[6]; _J[15] += J[9]; _J[18] += J[9]; _J[20] += J[9];
    return *this;
}

inline const AInertia& AInertia::operator-= (const Inertia& J)
{
    _J[0] -= J[0]; _J[1] -= J[3]; _J[2] -= J[4]; _J[4] -= J[8]; _J[5] -= J[7]; _J[6] -= J[1]; _J[7] -= J[5]; _J[8] -= J[8]; _J[10] -= J[6]; _J[11] -= J[2]; _J[12] -= J[7]; _J[13] -= J[6]; _J[15] -= J[9]; _J[18] -= J[9]; _J[20] -= J[9];
    return *this;
}

template <class TYPE>
inline void AInertia::ToArray(TYPE M[]) const
{
    M[0] = (TYPE)_J[0]; M[6] = (TYPE)_J[1];  M[12] = (TYPE)_J[2];  M[18] = (TYPE)_J[3];  M[24] = (TYPE)_J[4];  M[30] = (TYPE)_J[5];
    M[1] = (TYPE)_J[1]; M[7] = (TYPE)_J[6];  M[13] = (TYPE)_J[7];  M[19] = (TYPE)_J[8];  M[25] = (TYPE)_J[9];  M[31] = (TYPE)_J[10];
    M[2] = (TYPE)_J[2]; M[8] = (TYPE)_J[7];  M[14] = (TYPE)_J[11]; M[20] = (TYPE)_J[12]; M[26] = (TYPE)_J[13]; M[32] = (TYPE)_J[14];
    M[3] = (TYPE)_J[3]; M[9] = (TYPE)_J[8];  M[15] = (TYPE)_J[12]; M[21] = (TYPE)_J[15]; M[27] = (TYPE)_J[16]; M[33] = (TYPE)_J[17];
    M[4] = (TYPE)_J[4]; M[10] = (TYPE)_J[9];  M[16] = (TYPE)_J[13]; M[22] = (TYPE)_J[16]; M[28] = (TYPE)_J[18]; M[34] = (TYPE)_J[19];
    M[5] = (TYPE)_J[5]; M[11] = (TYPE)_J[10]; M[17] = (TYPE)_J[14]; M[23] = (TYPE)_J[17]; M[29] = (TYPE)_J[19]; M[35] = (TYPE)_J[20];
}

inline AInertia AInertia::Transform(const SE3& T) const
{
    double d0 = _J[ 3] + T._T[11] * _J[16] - T._T[10] * _J[17];
    double d1 = _J[ 8] - T._T[11] * _J[15] + T._T[ 9] * _J[17];
    double d2 = _J[12] + T._T[10] * _J[15] - T._T[ 9] * _J[16];
    double d3 = _J[ 4] + T._T[11] * _J[18] - T._T[10] * _J[19];
    double d4 = _J[ 9] - T._T[11] * _J[16] + T._T[ 9] * _J[19];
    double d5 = _J[13] + T._T[10] * _J[16] - T._T[ 9] * _J[18];
    double d6 = _J[ 5] + T._T[11] * _J[19] - T._T[10] * _J[20];
    double d7 = _J[10] - T._T[11] * _J[17] + T._T[ 9] * _J[20];
    double d8 = _J[14] + T._T[10] * _J[17] - T._T[ 9] * _J[19];
    double e0 = _J[ 0] + T._T[11] * _J[ 4] - T._T[10] * _J[ 5] + d3 * T._T[11] - d6 * T._T[10];
    double e3 = _J[ 1] + T._T[11] * _J[ 9] - T._T[10] * _J[10] - d0 * T._T[11] + d6 * T._T[ 9];
    double e4 = _J[ 6] - T._T[11] * _J[ 8] + T._T[ 9] * _J[10] - d1 * T._T[11] + d7 * T._T[ 9];
    double e6 = _J[ 2] + T._T[11] * _J[13] - T._T[10] * _J[14] + d0 * T._T[10] - d3 * T._T[ 9];
    double e7 = _J[ 7] - T._T[11] * _J[12] + T._T[ 9] * _J[14] + d1 * T._T[10] - d4 * T._T[ 9];
    double e8 = _J[11] + T._T[10] * _J[12] - T._T[ 9] * _J[13] + d2 * T._T[10] - d5 * T._T[ 9];
    double f0 = T._T[0] * e0 + T._T[1] * e3 + T._T[2] * e6;
    double f1 = T._T[0] * e3 + T._T[1] * e4 + T._T[2] * e7;
    double f2 = T._T[0] * e6 + T._T[1] * e7 + T._T[2] * e8;
    double f3 = T._T[0] * d0 + T._T[1] * d1 + T._T[2] * d2;
    double f4 = T._T[0] * d3 + T._T[1] * d4 + T._T[2] * d5;
    double f5 = T._T[0] * d6 + T._T[1] * d7 + T._T[2] * d8;
    double f6 = T._T[3] * e0 + T._T[4] * e3 + T._T[5] * e6;
    double f7 = T._T[3] * e3 + T._T[4] * e4 + T._T[5] * e7;
    double f8 = T._T[3] * e6 + T._T[4] * e7 + T._T[5] * e8;
    double g0 = T._T[3] * d0 + T._T[4] * d1 + T._T[5] * d2;
    double g1 = T._T[3] * d3 + T._T[4] * d4 + T._T[5] * d5;
    double g2 = T._T[3] * d6 + T._T[4] * d7 + T._T[5] * d8;
    double g3 = T._T[6] * d0 + T._T[7] * d1 + T._T[8] * d2;
    double g4 = T._T[6] * d3 + T._T[7] * d4 + T._T[8] * d5;
    double g5 = T._T[6] * d6 + T._T[7] * d7 + T._T[8] * d8;
    double h0 = T._T[0] * _J[15] + T._T[1] * _J[16] + T._T[2] * _J[17];
    double h1 = T._T[0] * _J[16] + T._T[1] * _J[18] + T._T[2] * _J[19];
    double h2 = T._T[0] * _J[17] + T._T[1] * _J[19] + T._T[2] * _J[20];
    double h3 = T._T[3] * _J[15] + T._T[4] * _J[16] + T._T[5] * _J[17];
    double h4 = T._T[3] * _J[16] + T._T[4] * _J[18] + T._T[5] * _J[19];
    double h5 = T._T[3] * _J[17] + T._T[4] * _J[19] + T._T[5] * _J[20];

    return AInertia(f0 * T._T[0] + f1 * T._T[1] + f2 * T._T[2],
                    f0 * T._T[3] + f1 * T._T[4] + f2 * T._T[5],
                    f0 * T._T[6] + f1 * T._T[7] + f2 * T._T[8],
                    f3 * T._T[0] + f4 * T._T[1] + f5 * T._T[2],
                    f3 * T._T[3] + f4 * T._T[4] + f5 * T._T[5],
                    f3 * T._T[6] + f4 * T._T[7] + f5 * T._T[8],
                    f6 * T._T[3] + f7 * T._T[4] + f8 * T._T[5],
                    f6 * T._T[6] + f7 * T._T[7] + f8 * T._T[8],
                    g0 * T._T[0] + g1 * T._T[1] + g2 * T._T[2],
                    g0 * T._T[3] + g1 * T._T[4] + g2 * T._T[5],
                    g0 * T._T[6] + g1 * T._T[7] + g2 * T._T[8],
                    (T._T[6] * e0 + T._T[7] * e3 + T._T[8] * e6) * T._T[6] + (T._T[6] * e3 + T._T[7] * e4 + T._T[8] * e7) * T._T[7] + (T._T[6] * e6 + T._T[7] * e7 + T._T[8] * e8) * T._T[8],
                    g3 * T._T[0] + g4 * T._T[1] + g5 * T._T[2],
                    g3 * T._T[3] + g4 * T._T[4] + g5 * T._T[5],
                    g3 * T._T[6] + g4 * T._T[7] + g5 * T._T[8],
                    h0 * T._T[0] + h1 * T._T[1] + h2 * T._T[2],
                    h0 * T._T[3] + h1 * T._T[4] + h2 * T._T[5],
                    h0 * T._T[6] + h1 * T._T[7] + h2 * T._T[8],
                    h3 * T._T[3] + h4 * T._T[4] + h5 * T._T[5],
                    h3 * T._T[6] + h4 * T._T[7] + h5 * T._T[8],
                    (T._T[6] * _J[15] + T._T[7] * _J[16] + T._T[8] * _J[17]) * T._T[6] + (T._T[6] * _J[16] + T._T[7] * _J[18] + T._T[8] * _J[19]) * T._T[7] + (T._T[6] * _J[17] + T._T[7] * _J[19] + T._T[8] * _J[20]) * T._T[8]);
}

inline void AInertia::AddTransform(const AInertia& J, const SE3& T)
{
    double d0 = J[3] + T._T[11] * J[16] - T._T[10] * J[17];
    double d1 = J[8] - T._T[11] * J[15] + T._T[9] * J[17];
    double d2 = J[12] + T._T[10] * J[15] - T._T[9] * J[16];
    double d3 = J[4] + T._T[11] * J[18] - T._T[10] * J[19];
    double d4 = J[9] - T._T[11] * J[16] + T._T[9] * J[19];
    double d5 = J[13] + T._T[10] * J[16] - T._T[9] * J[18];
    double d6 = J[5] + T._T[11] * J[19] - T._T[10] * J[20];
    double d7 = J[10] - T._T[11] * J[17] + T._T[9] * J[20];
    double d8 = J[14] + T._T[10] * J[17] - T._T[9] * J[19];
    double e0 = J[0] + T._T[11] * J[4] - T._T[10] * J[5] + d3 * T._T[11] - d6 * T._T[10];
    double e3 = J[1] + T._T[11] * J[9] - T._T[10] * J[10] - d0 * T._T[11] + d6 * T._T[9];
    double e4 = J[6] - T._T[11] * J[8] + T._T[9] * J[10] - d1 * T._T[11] + d7 * T._T[9];
    double e6 = J[2] + T._T[11] * J[13] - T._T[10] * J[14] + d0 * T._T[10] - d3 * T._T[9];
    double e7 = J[7] - T._T[11] * J[12] + T._T[9] * J[14] + d1 * T._T[10] - d4 * T._T[9];
    double e8 = J[11] + T._T[10] * J[12] - T._T[9] * J[13] + d2 * T._T[10] - d5 * T._T[9];
    double f0 = T._T[0] * e0 + T._T[1] * e3 + T._T[2] * e6;
    double f1 = T._T[0] * e3 + T._T[1] * e4 + T._T[2] * e7;
    double f2 = T._T[0] * e6 + T._T[1] * e7 + T._T[2] * e8;
    double f3 = T._T[0] * d0 + T._T[1] * d1 + T._T[2] * d2;
    double f4 = T._T[0] * d3 + T._T[1] * d4 + T._T[2] * d5;
    double f5 = T._T[0] * d6 + T._T[1] * d7 + T._T[2] * d8;
    double f6 = T._T[3] * e0 + T._T[4] * e3 + T._T[5] * e6;
    double f7 = T._T[3] * e3 + T._T[4] * e4 + T._T[5] * e7;
    double f8 = T._T[3] * e6 + T._T[4] * e7 + T._T[5] * e8;
    double g0 = T._T[3] * d0 + T._T[4] * d1 + T._T[5] * d2;
    double g1 = T._T[3] * d3 + T._T[4] * d4 + T._T[5] * d5;
    double g2 = T._T[3] * d6 + T._T[4] * d7 + T._T[5] * d8;
    double g3 = T._T[6] * d0 + T._T[7] * d1 + T._T[8] * d2;
    double g4 = T._T[6] * d3 + T._T[7] * d4 + T._T[8] * d5;
    double g5 = T._T[6] * d6 + T._T[7] * d7 + T._T[8] * d8;
    double h0 = T._T[0] * J[15] + T._T[1] * J[16] + T._T[2] * J[17];
    double h1 = T._T[0] * J[16] + T._T[1] * J[18] + T._T[2] * J[19];
    double h2 = T._T[0] * J[17] + T._T[1] * J[19] + T._T[2] * J[20];
    double h3 = T._T[3] * J[15] + T._T[4] * J[16] + T._T[5] * J[17];
    double h4 = T._T[3] * J[16] + T._T[4] * J[18] + T._T[5] * J[19];
    double h5 = T._T[3] * J[17] + T._T[4] * J[19] + T._T[5] * J[20];

    _J[0] += f0 * T._T[0] + f1 * T._T[1] + f2 * T._T[2];
    _J[1] += f0 * T._T[3] + f1 * T._T[4] + f2 * T._T[5];
    _J[2] += f0 * T._T[6] + f1 * T._T[7] + f2 * T._T[8];
    _J[3] += f3 * T._T[0] + f4 * T._T[1] + f5 * T._T[2];
    _J[4] += f3 * T._T[3] + f4 * T._T[4] + f5 * T._T[5];
    _J[5] += f3 * T._T[6] + f4 * T._T[7] + f5 * T._T[8];
    _J[6] += f6 * T._T[3] + f7 * T._T[4] + f8 * T._T[5];
    _J[7] += f6 * T._T[6] + f7 * T._T[7] + f8 * T._T[8];
    _J[8] += g0 * T._T[0] + g1 * T._T[1] + g2 * T._T[2];
    _J[9] += g0 * T._T[3] + g1 * T._T[4] + g2 * T._T[5];
    _J[10] += g0 * T._T[6] + g1 * T._T[7] + g2 * T._T[8];
    _J[11] += (T._T[6] * e0 + T._T[7] * e3 + T._T[8] * e6) * T._T[6] + (T._T[6] * e3 + T._T[7] * e4 + T._T[8] * e7) * T._T[7] + (T._T[6] * e6 + T._T[7] * e7 + T._T[8] * e8) * T._T[8];
    _J[12] += g3 * T._T[0] + g4 * T._T[1] + g5 * T._T[2];
    _J[13] += g3 * T._T[3] + g4 * T._T[4] + g5 * T._T[5];
    _J[14] += g3 * T._T[6] + g4 * T._T[7] + g5 * T._T[8];
    _J[15] += h0 * T._T[0] + (T._T[0] * J[16] + T._T[1] * J[18] + T._T[2] * J[19]) * T._T[1] + (T._T[0] * J[17] + T._T[1] * J[19] + T._T[2] * J[20]) * T._T[2];
    _J[16] += h0 * T._T[3] + (T._T[0] * J[16] + T._T[1] * J[18] + T._T[2] * J[19]) * T._T[4] + (T._T[0] * J[17] + T._T[1] * J[19] + T._T[2] * J[20]) * T._T[5];
    _J[17] += h0 * T._T[6] + (T._T[0] * J[16] + T._T[1] * J[18] + T._T[2] * J[19]) * T._T[7] + (T._T[0] * J[17] + T._T[1] * J[19] + T._T[2] * J[20]) * T._T[8];
    _J[18] += h3 * T._T[3] + (T._T[3] * J[16] + T._T[4] * J[18] + T._T[5] * J[19]) * T._T[4] + (T._T[3] * J[17] + T._T[4] * J[19] + T._T[5] * J[20]) * T._T[5];
    _J[19] += h3 * T._T[6] + (T._T[3] * J[16] + T._T[4] * J[18] + T._T[5] * J[19]) * T._T[7] + (T._T[3] * J[17] + T._T[4] * J[19] + T._T[5] * J[20]) * T._T[8];
    _J[20] += (T._T[6] * J[15] + T._T[7] * J[16] + T._T[8] * J[17]) * T._T[6] + (T._T[6] * J[16] + T._T[7] * J[18] + T._T[8] * J[19]) * T._T[7] + (T._T[6] * J[17] + T._T[7] * J[19] + T._T[8] * J[20]) * T._T[8];
}

inline se3 AInertia::operator*(const dse3& f) const
{
    return se3(	_J[0] * f._m[0] + _J[1] * f._m[1] + _J[2] * f._m[2] + _J[3] * f._m[3] + _J[4] * f._m[4] + _J[5] * f._m[5],
                _J[1] * f._m[0] + _J[6] * f._m[1] + _J[7] * f._m[2] + _J[8] * f._m[3] + _J[9] * f._m[4] + _J[10] * f._m[5],
                _J[2] * f._m[0] + _J[7] * f._m[1] + _J[11] * f._m[2] + _J[12] * f._m[3] + _J[13] * f._m[4] + _J[14] * f._m[5],
                _J[3] * f._m[0] + _J[8] * f._m[1] + _J[12] * f._m[2] + _J[15] * f._m[3] + _J[16] * f._m[4] + _J[17] * f._m[5],
                _J[4] * f._m[0] + _J[9] * f._m[1] + _J[13] * f._m[2] + _J[16] * f._m[3] + _J[18] * f._m[4] + _J[19] * f._m[5],
                _J[5] * f._m[0] + _J[10] * f._m[1] + _J[14] * f._m[2] + _J[17] * f._m[3] + _J[19] * f._m[4] + _J[20] * f._m[5]);
}

// SCALAR_1_2 * ( x * ~y + y * ~x )
inline AInertia KroneckerProduct(const dse3& x, const dse3& y)
{
    double y_m0 = SCALAR_1_2 * y._m[0];
    double y_m1 = SCALAR_1_2 * y._m[1];
    double y_m2 = SCALAR_1_2 * y._m[2];
    double y_m3 = SCALAR_1_2 * y._m[3];
    double y_m4 = SCALAR_1_2 * y._m[4];
    double y_m5 = SCALAR_1_2 * y._m[5];

    return AInertia(x._m[0] * y._m[0],
                    x._m[0] * y_m1 + x._m[1] * y_m0,
                    x._m[0] * y_m2 + x._m[2] * y_m0,
                    x._m[0] * y_m3 + x._m[3] * y_m0,
                    x._m[0] * y_m4 + x._m[4] * y_m0,
                    x._m[0] * y_m5 + x._m[5] * y_m0,
                    x._m[1] * y._m[1],
                    x._m[1] * y_m2 + x._m[2] * y_m1,
                    x._m[1] * y_m3 + x._m[3] * y_m1,
                    x._m[1] * y_m4 + x._m[4] * y_m1,
                    x._m[1] * y_m5 + x._m[5] * y_m1,
                    x._m[2] * y._m[2],
                    x._m[2] * y_m3 + x._m[3] * y_m2,
                    x._m[2] * y_m4 + x._m[4] * y_m2,
                    x._m[2] * y_m5 + x._m[5] * y_m2,
                    x._m[3] * y._m[3],
                    x._m[3] * y_m4 + x._m[4] * y_m3,
                    x._m[3] * y_m5 + x._m[5] * y_m3,
                    x._m[4] * y._m[4],
                    x._m[4] * y_m5 + x._m[5] * y_m4,
                    x._m[5] * y._m[5]);
}

// *this -= KroneckerProduct(x, y)
inline void AInertia::SubtractKroneckerProduct(const dse3& x, const dse3& y)
{
    double y_m0 = SCALAR_1_2 * y._m[0];
    double y_m1 = SCALAR_1_2 * y._m[1];
    double y_m2 = SCALAR_1_2 * y._m[2];
    double y_m3 = SCALAR_1_2 * y._m[3];
    double y_m4 = SCALAR_1_2 * y._m[4];
    double y_m5 = SCALAR_1_2 * y._m[5];

    _J[0]  -= x._m[0] * y._m[0];
    _J[1]  -= x._m[0] * y_m1 + x._m[1] * y_m0;
    _J[2]  -= x._m[0] * y_m2 + x._m[2] * y_m0;
    _J[3]  -= x._m[0] * y_m3 + x._m[3] * y_m0;
    _J[4]  -= x._m[0] * y_m4 + x._m[4] * y_m0;
    _J[5]  -= x._m[0] * y_m5 + x._m[5] * y_m0;
    _J[6]  -= x._m[1] * y._m[1];
    _J[7]  -= x._m[1] * y_m2 + x._m[2] * y_m1;
    _J[8]  -= x._m[1] * y_m3 + x._m[3] * y_m1;
    _J[9]  -= x._m[1] * y_m4 + x._m[4] * y_m1;
    _J[10] -= x._m[1] * y_m5 + x._m[5] * y_m1;
    _J[11] -= x._m[2] * y._m[2];
    _J[12] -= x._m[2] * y_m3 + x._m[3] * y_m2;
    _J[13] -= x._m[2] * y_m4 + x._m[4] * y_m2;
    _J[14] -= x._m[2] * y_m5 + x._m[5] * y_m2;
    _J[15] -= x._m[3] * y._m[3];
    _J[16] -= x._m[3] * y_m4 + x._m[4] * y_m3;
    _J[17] -= x._m[3] * y_m5 + x._m[5] * y_m3;
    _J[18] -= x._m[4] * y._m[4];
    _J[19] -= x._m[4] * y_m5 + x._m[5] * y_m4;
    _J[20] -= x._m[5] * y._m[5];
}

inline se3 AInertia::operator % (const dse3& b) const
{
    double a00 = _J[6] * _J[11] - _J[7] * _J[7];
    double a01 = _J[2] * _J[7] - _J[1] * _J[11];
    double a02 = _J[1] * _J[7] - _J[2] * _J[6];
    double idet = SCALAR_1 / (_J[0] * a00 + _J[1] * a01 + _J[2] * a02);
    double a11 = idet * (_J[0] * _J[11] - _J[2] * _J[2]);
    double a12 = idet * (_J[2] * _J[1] - _J[0] * _J[7]);
    double a22 = idet * (_J[0] * _J[6] - _J[1] * _J[1]);
    a00 *= idet;
    a01 *= idet;
    a02 *= idet;
    double t00 = a00 * _J[3] + a01 * _J[8] + a02 * _J[12];
    double t01 = a00 * _J[4] + a01 * _J[9] + a02 * _J[13];
    double t02 = a00 * _J[5] + a01 * _J[10] + a02 * _J[14];
    double t10 = a01 * _J[3] + a11 * _J[8] + a12 * _J[12];
    double t11 = a01 * _J[4] + a11 * _J[9] + a12 * _J[13];
    double t12 = a01 * _J[5] + a11 * _J[10] + a12 * _J[14];
    double t20 = a02 * _J[3] + a12 * _J[8] + a22 * _J[12];
    double t21 = a02 * _J[4] + a12 * _J[9] + a22 * _J[13];
    double t22 = a02 * _J[5] + a12 * _J[10] + a22 * _J[14];
    double r0 = a00 * b._m[0] + a01 * b._m[1] + a02 * b._m[2];
    double r1 = a01 * b._m[0] + a11 * b._m[1] + a12 * b._m[2];
    double r2 = a02 * b._m[0] + a12 * b._m[1] + a22 * b._m[2];
    a00 = r0;
    a01 = r1;
    a02 = r2;
    double x0 = b._m[3] - _J[3] * r0 - _J[8] * r1 - _J[12] * r2;
    double x1 = b._m[4] - _J[4] * r0 - _J[9] * r1 - _J[13] * r2;
    double x2 = b._m[5] - _J[5] * r0 - _J[10] * r1 - _J[14] * r2;
    double c00 = _J[15] - _J[3] * t00 - _J[8] * t10 - _J[12] * t20;
    double c01 = _J[16] - _J[3] * t01 - _J[8] * t11 - _J[12] * t21;
    double c11 = _J[18] - _J[4] * t01 - _J[9] * t11 - _J[13] * t21;
    double c02 = _J[17] - _J[3] * t02 - _J[8] * t12 - _J[12] * t22;
    double c12 = _J[19] - _J[4] * t02 - _J[9] * t12 - _J[13] * t22;
    double c22 = _J[20] - _J[5] * t02 - _J[10] * t12 - _J[14] * t22;
    double r3 = c02 * c01 - c00 * c12;
    r0 = c11 * c22 - c12 * c12;
    r1 = c02 * c12 - c01 * c22;
    r2 = c01 * c12 - c02 * c11;
    idet = SCALAR_1 / (c00 * r0 + c01 * r1 + c02 * r2);
    a22 = idet * (r0 * x0 + r1 * x1 + r2 * x2);
    a11 = idet * (r1 * x0 + (c00 * c22 - c02 * c02) * x1 + r3 * x2);
    a12 = idet * (r2 * x0 + r3 * x1 + (c00 * c11 - c01 * c01) * x2);
    a00 -= t00 * a22 + t01 * a11 + t02 * a12;
    a01 -= t10 * a22 + t11 * a11 + t12 * a12;
    a02 -= t20 * a22 + t21 * a11 + t22 * a12;

    return se3(a00, a01 ,a02, a22, a11, a12);
}


inline AInertia Inv(const Inertia& I)
{
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

inline const AInertia& AInertia::operator = (const AInertia& J)
{
    _J[0] = J[0];	_J[1] = J[1];	_J[2] = J[2];
    _J[3] = J[3];	_J[4] = J[4];	_J[5] = J[5];
    _J[6] = J[6];	_J[7] = J[7];	_J[8] = J[8];
    _J[9] = J[9];	_J[10] = J[10];	_J[11] = J[11];
    _J[12] = J[12];	_J[13] = J[13];	_J[14] = J[14];
    _J[15] = J[15];	_J[16] = J[16];	_J[17] = J[17];
    _J[18] = J[18];	_J[19] = J[19];	_J[20] = J[20];
    return *this;
}

inline const AInertia& AInertia::operator = (const Inertia& I)
{
    _J[0] = I[0];	_J[1] = I[3];	_J[2] = I[4];	_J[3] = SCALAR_0;	_J[4] = -I[8];			_J[5] = I[7];
                    _J[6] = I[1];	_J[7] = I[5];	_J[8] = I[8];			_J[9] = SCALAR_0;	_J[10] = -I[6];
                                    _J[11] = I[2];	_J[12] = -I[7];			_J[13] = I[6];			_J[14] = SCALAR_0;
                                                    _J[15] = I[9];			_J[16] = SCALAR_0;	_J[17] = SCALAR_0;
                                                                            _J[18] = I[9];			_J[19] = SCALAR_0;
                                                                                                    _J[20] = I[9];
    return *this;
}

inline Axis::Axis()
{
    _v[0] = _v[1] = _v[2] = 0.0;
}

inline Axis::Axis(double d)
{
    _v[0] = _v[1] = _v[2] = d;
}

inline Axis::Axis(const double v[])
{
    _v[0] = v[0];
    _v[1] = v[1];
    _v[2] = v[2];
}

inline Axis::Axis(double v0, double v1, double v2)
{
    _v[0] = v0;
    _v[1] = v1;
    _v[2] = v2;
}

inline Axis::Axis(const Vec3& v)
{
    _v[0] = v._v[0];
    _v[1] = v._v[1];
    _v[2] = v._v[2];
}

inline Axis::~Axis()
{
}

inline const Axis& Axis::operator+(void) const
{
    return *this;
}

inline Axis Axis::operator-(void) const
{
    return Axis(-_v[0], -_v[1], -_v[2]);
}

//inline double& Axis::operator[](int i)
//{
//    return _v[i];
//}

//inline const double& Axis::operator[](int i) const
//{
//    return _v[i];
//}

inline double& Axis::operator()(int i)
{
    return _v[i];
}

inline const double& Axis::operator()(int i) const
{
    return _v[i];
}

inline const Axis& Axis::operator = (const Axis& v)
{
    _v[0] = v._v[0];
    _v[1] = v._v[1];
    _v[2] = v._v[2];
    return *this;
}

inline const Axis& Axis::operator = (const se3& v)
{
    _v[0] = v._w[0];
    _v[1] = v._w[1];
    _v[2] = v._w[2];
    return *this;
}

inline const Axis& Axis::operator = (double d)
{
    _v[0] = _v[1] = _v[2] = d;
    return *this;
}

inline const Axis& Axis::operator *= (double d)
{
    _v[0] *= d;
    _v[1] *= d;
    _v[2] *= d;
    return *this;
}

inline bool Axis::operator==(const Axis& v) const
{
    if ((_v[0] != v._v[0])
            || (_v[1] != v._v[1])
            || (_v[2] != v._v[2]))
        return false;

    return true;
}

inline bool Axis::operator!=(const Axis& v) const
{
    return !(*this == v);
}

inline Axis Axis::operator*(double d) const
{
    return Axis(d * _v[0], d * _v[1], d * _v[2]);
}

inline double Axis::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
    {
        _v[0] = _v[1] = SCALAR_0;
        _v[2] = SCALAR_1;
    } else
    {
        _v[0] /= mag;
        _v[1] /= mag;
        _v[2] /= mag;
    }
    return mag;
}

inline void Axis::Reparameterize(void)
{
    double theta = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
    if ( theta > LIE_EPS )
    {
        double eta = 1.0 - (double)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
        _v[0] *= eta;
        _v[1] *= eta;
        _v[2] *= eta;
    }
}

inline Axis Reparameterize(const Axis& s)
{
    double theta = sqrt(s._v[0] * s._v[0] + s._v[1] * s._v[1] + s._v[2] * s._v[2]);
    double eta = theta < LIE_EPS ? 1.0 : 1.0 - (double)((int)(theta / M_PI + 1.0) / 2) * M_2PI / theta;
    return eta * s;
}

inline Axis Rotate(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v._v[0] + T._T[3] * v._v[1] + T._T[6] * v._v[2],
                T._T[1] * v._v[0] + T._T[4] * v._v[1] + T._T[7] * v._v[2],
                T._T[2] * v._v[0] + T._T[5] * v._v[1] + T._T[8] * v._v[2]);
}

inline Axis InvRotate(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

inline Axis operator*(double d, const Axis& v)
{
    return Axis(d * v._v[0], d * v._v[1], d * v._v[2]);
}

inline double Norm(const Axis& v)
{
    return sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
}

inline Axis Normalize(const Axis& v)
{
    double mag = sqrt(v._v[0] * v._v[0] + v._v[1] * v._v[1] + v._v[2] * v._v[2]);
    if ( mag < LIE_EPS )	// make a unit vector in z-direction
        return Axis(SCALAR_0, SCALAR_0, SCALAR_1);

    mag = SCALAR_1 / mag;
    return Axis(mag * v._v[0], mag * v._v[1], mag * v._v[2]);
}

inline Axis Cross(const Axis& p, const Axis& q)
{
    return Axis(p._v[1] * q._v[2] - p._v[2] * q._v[1],
                p._v[2] * q._v[0] - p._v[0] * q._v[2],
                p._v[0] * q._v[1] - p._v[1] * q._v[0]);
}

inline double Inner(const Axis& p, const Axis& q)
{
    return (p._v[0] * q._v[0] + p._v[1] * q._v[1] + p._v[2] * q._v[2]);
}

inline double Inner(const Vec3& p, const Axis& q)
{
    return (p._v[0] * q._v[0] + p._v[1] * q._v[1] + p._v[2] * q._v[2]);
}

inline double Inner(const Axis& p, const Vec3& q)
{
    return (p._v[0] * q._v[0] + p._v[1] * q._v[1] + p._v[2] * q._v[2]);
}

inline double SquareSum(const Axis& p)
{
    return (p._v[0] * p._v[0] + p._v[1] * p._v[1] + p._v[2] * p._v[2]);
}

inline Axis Square(const Axis& p)
{
    return Axis(p._v[0] * p._v[0], p._v[1] * p._v[1], p._v[2] * p._v[2]);
}

inline Axis InvAd(const SE3& T, const Axis& v)
{
    return Axis(T._T[0] * v._v[0] + T._T[1] * v._v[1] + T._T[2] * v._v[2],
                T._T[3] * v._v[0] + T._T[4] * v._v[1] + T._T[5] * v._v[2],
                T._T[6] * v._v[0] + T._T[7] * v._v[1] + T._T[8] * v._v[2]);
}

inline Axis ad(const Axis& s1, const se3& s2)
{
    return Axis(s2._w[2] * s1._v[1] - s2._w[1] * s1._v[2],
                s2._w[0] * s1._v[2] - s2._w[2] * s1._v[0],
                s2._w[1] * s1._v[0] - s2._w[0] * s1._v[1]);
}

inline Axis ad(const Axis& s1, const Axis& s2)
{
    return Axis(s2._v[2] * s1._v[1] - s2._v[1] * s1._v[2],
                s2._v[0] * s1._v[2] - s2._v[2] * s1._v[0],
                s2._v[1] * s1._v[0] - s2._v[0] * s1._v[1]);
}

inline Axis Axis::operator+(const Axis& v) const
{
    return Axis(_v[0] + v._v[0], _v[1] + v._v[1], _v[2] + v._v[2]);
}

inline se3 Axis::operator+(const Vec3& v) const
{
    return se3(_v[0], _v[1], _v[2], v._v[0], v._v[1], v._v[2]);
}

inline Axis Axis::operator-(const Axis& v) const
{
    return Axis(_v[0] - v._v[0], _v[1] - v._v[1], _v[2] - v._v[2]);
}

inline const Axis& Axis::operator += (const Axis& v)
{
    _v[0] += v._v[0];
    _v[1] += v._v[1];
    _v[2] += v._v[2];
    return *this;
}

inline const Axis& Axis::operator-= (const Axis& v)
{
    _v[0] -= v._v[0];
    _v[1] -= v._v[1];
    _v[2] -= v._v[2];
    return *this;
}

//==============================================================================
//
//==============================================================================
inline Jacobian::Jacobian()
{
}

inline Jacobian::Jacobian(unsigned int _size)
{
    mJ.resize(_size);
}

//Jacobian::Jacobian(const Eigen::MatrixXd& _J)
//{
//    setMatrix(_J);
//}

inline Jacobian::~Jacobian()
{
}

inline const Jacobian& Jacobian::operator=(const Jacobian& T)
{
    mJ = T.mJ;
    return *this;
}

inline void Jacobian::setFromEigenMatrix(const Eigen::MatrixXd& _J)
{
    assert(_J.rows() == 6);

    setSize(_J.cols());

    for (int i = 0; i < getSize(); ++i)
        mJ[i].setEigenVector(_J.col(i));
}

inline Eigen::MatrixXd Jacobian::getEigenMatrix() const
{
    Eigen::MatrixXd J(6, getSize());

    for (int i = 0; i < getSize(); i++)
        J.col(i) = mJ[i].getEigenVector();

    return J;
}

inline void Jacobian::setZero()
{
    for (int i = 0; i < getSize(); i++)
        mJ[i].setZero();
}

inline se3 Jacobian::getColumn(int _idx)
{
    return mJ[_idx];
}

inline Jacobian Jacobian::getColumns(int _idx, int _size)
{
    // TODO: NEED TEST
    assert(0 <= _idx);
    assert(_idx + _size <= mJ.size());

    Jacobian J(_size);

    for (int i = 0; i < _size; ++i)
        J.setColumn(i, getColumn(i));

    return J;
}

inline Jacobian Jacobian::getAdjointed(const SE3& _T) const
{
    Jacobian ret;

    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

inline Jacobian Jacobian::getAdjointedInv(const SE3& _Tinv) const
{
    Jacobian ret;

    dterr << "NOT IMPLEMENTED.\n";

    return ret;
}

inline se3& Jacobian::operator[](int _i)
{
    assert(0 <= _i && _i < getSize());

    return mJ[_i];
}

inline const se3& Jacobian::operator[](int _i) const
{
    assert(0 <= _i && _i < getSize());

    return mJ[_i];
}

inline se3 Jacobian::operator*(const Eigen::VectorXd& _qdot)
{
    assert(_qdot.size() == getSize());

    se3 ret;

    for (int i = 0; i < getSize(); ++i)
        ret += mJ[i] * _qdot(i);

    return ret;
}

inline bool Jacobian::operator==(const Jacobian& _rhs) const
{
    for (int i = 0; i < getSize(); ++i)
        if (mJ[i] != _rhs.mJ[i])
            return false;

    return true;
}

inline bool Jacobian::operator!=(const Jacobian& _rhs) const
{
    return !(*this == _rhs);
}

inline Eigen::VectorXd Jacobian::getInnerProduct(const dse3& _F) const
{
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(getSize());

    assert(ret.size() == getSize());

    int size = getSize();
    for (int i = 0; i < size; ++i)
        ret(i) = Inner(mJ[i], _F);

    return ret;
}

inline double Inner(const se3& V, const dse3& f)
{
    return (f._m[0] * V._w[0] + f._m[1] * V._w[1] + f._m[2] * V._w[2]
            + f._m[3] * V._w[3] + f._m[4] * V._w[4] + f._m[5] * V._w[5]);
}

inline double Inner(const dse3& F, const se3& V)
{
    return (F._m[0] * V._w[0] + F._m[1] * V._w[1] + F._m[2] * V._w[2]
            + F._m[3] * V._w[3] + F._m[4] * V._w[4] + F._m[5] * V._w[5]);
}

inline double Inner(const dse3& f, const Vec3& v)
{
    return (f._m[3] * v._v[0] + f._m[4] * v._v[1] + f._m[5] * v._v[2]);
}

inline double Inner(const dse3& F, const Axis& w)
{
    return (F._m[0] * w._v[0] + F._m[1] * w._v[1] + F._m[2] * w._v[2]);
}

inline double Distance(const SE3& T1, const SE3& T2)
{
    return Norm(Log(Inv(T1)*T2));
}

inline double Norm(const se3& s)
{
    return sqrt(s._w[0] * s._w[0]
            + s._w[1] * s._w[1]
            + s._w[2] * s._w[2]
            + s._w[3] * s._w[3]
            + s._w[4] * s._w[4]
            + s._w[5] * s._w[5]);
}

inline double Norm(const dse3& f)
{
    return sqrt(f._m[0] * f._m[0]
            + f._m[1] * f._m[1]
            + f._m[2] * f._m[2]
            + f._m[3] * f._m[3]
            + f._m[4] * f._m[4]
            + f._m[5] * f._m[5]);
}

} // namespace math
} // namespace dart

#endif

