/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/23/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "utils/Timer.h"
#include "math/LieGroup.h"
#include "math/UtilsMath.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"

using namespace dart;
using namespace utils;
using namespace math;
using namespace dynamics;
using namespace simulation;

#define MATH_TOL 0.000001
#define MATH_EPS 0.000001

class EigenSE3
{
public:
    explicit EigenSE3(const Eigen::Matrix4d& T)
        : mT(T)
    {}

    /// @brief multiplication operator
    inline EigenSE3 operator*(const EigenSE3& T) const
    {
        return EigenSE3(mT * T.mT);
    }

    /// @brief multiplication operator
    inline const EigenSE3& operator*=(const EigenSE3& T)
    {
        mT *= T.mT;
        return *this;
    }

protected:
private:
    Eigen::Matrix4d mT;
};



using namespace Eigen;
using namespace std;

EIGEN_DONT_INLINE
void concatenate(const Affine3d& A1, const Affine3d& A2, Affine3d& res) {
   res(0,0) = A1(0,0) * A2(0,0) + A1(0,1) * A2(1,0) + A1(0,2) * A2(2,0);
   res(1,0) = A1(1,0) * A2(0,0) + A1(1,1) * A2(1,0) + A1(1,2) * A2(2,0);
   res(2,0) = A1(2,0) * A2(0,0) + A1(2,1) * A2(1,0) + A1(2,2) * A2(2,0);

   res(0,1) = A1(0,0) * A2(0,1) + A1(0,1) * A2(1,1) + A1(0,2) * A2(2,1);
   res(1,1) = A1(1,0) * A2(0,1) + A1(1,1) * A2(1,1) + A1(1,2) * A2(2,1);
   res(2,1) = A1(2,0) * A2(0,1) + A1(2,1) * A2(1,1) + A1(2,2) * A2(2,1);

   res(0,2) = A1(0,0) * A2(0,2) + A1(0,1) * A2(1,2) + A1(0,2) * A2(2,2);
   res(1,2) = A1(1,0) * A2(0,2) + A1(1,1) * A2(1,2) + A1(1,2) * A2(2,2);
   res(2,2) = A1(2,0) * A2(0,2) + A1(2,1) * A2(1,2) + A1(2,2) * A2(2,2);

   res(0,3) = A1(0,0) * A2(0,3) + A1(0,1) * A2(1,3) + A1(0,2) * A2(2,3) + A1(0,3);
   res(1,3) = A1(1,0) * A2(0,3) + A1(1,1) * A2(1,3) + A1(1,2) * A2(2,3) + A1(1,3);
   res(2,3) = A1(2,0) * A2(0,3) + A1(2,1) * A2(1,3) + A1(2,2) * A2(2,3) + A1(2,3);

   res(3,0) = 0.0;
   res(3,1) = 0.0;
   res(3,2) = 0.0;
   res(3,3) = 1.0;
}

template<typename T>
EIGEN_DONT_INLINE
void prod(const T& a, const T& b, T& c) { c = a*b; }

TEST(MATH, TRANSFORMATION)
{
    const int iterations = 100000000;

    Affine3d A1 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Affine3d A2 = A1, A3;
    Matrix4d M1 = A1.matrix();
    Matrix4d M2 = M1, M3;
    SE3 S1;
    S1.setEigenMatrix(M1);
    SE3 S2 = S1, S3;

    clock_t start = clock();
    for(int i = 0; i < iterations; i++) {
       prod(S2, S1, S3);
    }
    cout << "SE3: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for(int i = 0; i < iterations; i++) {
       prod( A2 , A1, A3);
    }
    cout << "Affine3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for(int i = 0; i < iterations; i++) {
       prod ( M2 , M1, M3 );
    }
    cout << "Matrix4d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    A2 = A1;
    start = clock();
    for(int i = 0; i < iterations; i++) {
       concatenate(A2, A1, A3);
    }
    cout << "Hard-coded: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
}

/******************************************************************************/
//TEST(MATH, SE3_VS_EIGENMATRIX4D)
//{
//    int n = 10000;
//    double min = -100;
//    double max = 100;

//    SE3 T1 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));
//    SE3 T2 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));
//    SE3 T3 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));
//    SE3 T4 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));

//    Eigen::Matrix4d E1 = T1.getEigenMatrix();
//    Eigen::Matrix4d E2 = T2.getEigenMatrix();
//    Eigen::Matrix4d E3 = T3.getEigenMatrix();
//    Eigen::Matrix4d E4 = T4.getEigenMatrix();

//    Eigen::Affine3d A1(E1);
//    Eigen::Affine3d A2(E2);
//    Eigen::Affine3d A3(E3);
//    Eigen::Affine3d A4(E4);

//    Eigen::Isometry3d I1(E1);
//    Eigen::Isometry3d I2(E2);
//    Eigen::Isometry3d I3(E3);
//    Eigen::Isometry3d I4(E4);

//    EigenSE3 ESE3_1(E1);
//    EigenSE3 ESE3_2(E2);
//    EigenSE3 ESE3_3(E3);
//    EigenSE3 ESE3_4(E4);

//    {
//        Timer SE3Timer("SE3 timer");
//        SE3Timer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                T3 = T3 * T2;
//        SE3Timer.stopTimer();
//    }
//    std::cout << T3 << std::endl;

//    {
//        Timer EigenTimer("Eigen::Matrix4d timer");
//        EigenTimer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                E3 = E3 * E2;
//        EigenTimer.stopTimer();
//    }
//    std::cout << E3 << std::endl;

////    {
////        Timer EigenSE3Timer("EigenSE3 timer");
////        EigenSE3Timer.startTimer();
////        for (int i = 0; i < n; ++i)
////            for (int j = 0; j < n; ++j)
////                ESE3_3 *= ESE3_1 * ESE3_2;
////        EigenSE3Timer.stopTimer();
////    }
////    //std::cout << E3 << std::endl;

//    {
//        Timer AffineTimer("Eigen::Affine3d timer");
//        AffineTimer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                A3 = A3 * A2;
//        AffineTimer.stopTimer();
//    }
//    std::cout << A3.matrix() << std::endl;

//    {
//        Timer IsometryTimer("Eigen::Isometry3d timer");
//        IsometryTimer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                I3 = I3 * I2;
//        IsometryTimer.stopTimer();
//    }
//    std::cout << I3.matrix() << std::endl;

////    {
////        Timer SE3Timer("SE3 inverse timer");
////        SE3Timer.startTimer();
////        for (int i = 0; i < n; ++i)
////            for (int j = 0; j < n; ++j)
////                T4 *= Inv(T1) * Inv(T2);
////        SE3Timer.stopTimer();
////    }
////    std::cout << T4 << std::endl;

////    {
////        Timer EigenTimer("Eigen::Matrix4d inverse timer");
////        EigenTimer.startTimer();
////        for (int i = 0; i < n; ++i)
////            for (int j = 0; j < n; ++j)
////                E4 *= E1.inverse() * E2.inverse();
////        EigenTimer.stopTimer();
////    }
////    std::cout << E4 << std::endl;
//}

/******************************************************************************/
//TEST(MATH, SO3)
//{
//    // Exponential and Logarithm mapping
//    for (int i = 0; i < 100; ++i)
//    {
//        double min = -100;
//        double max = 100;

//        // Log(Exp(w)) = w
//        so3 w(random(min,max), random(min,max), random(min,max));
//        SO3 Expw = Exp(w);
//        so3 LogExpw = Log(Expw);
//        SO3 ExpLogExpw = Exp(LogExpw);
//        so3 LogExpLogExpw = Log(ExpLogExpw);

//        for (int j = 0; j < 3; ++j)
//        {
//            for (int k = 0; k < 3; ++k)
//            {
//                EXPECT_NEAR(Expw(j,k), ExpLogExpw(j,k), MATH_EPS);
//            }
//        }

//        for (int j = 0; j < 3; ++j)
//            EXPECT_NEAR(LogExpLogExpw[j], LogExpw[j], MATH_EPS);

//    }
//}

/******************************************************************************/
TEST(MATH, SE3)
{
    // Exponential and Logarithm mapping
    for (int i = 0; i < 100; ++i)
    {
        double min = -100;
        double max = 100;

        // Log(Exp(S)) = S
        se3 S(random(min,max), random(min,max), random(min,max),
              random(min,max), random(min,max), random(min,max));
        SE3 ExpS = Exp(S);
        se3 LogExpS = Log(ExpS);
        SE3 ExpLogExpS = Exp(S);
        se3 ExpLogLogExpS = Log(ExpS);

        for (int j = 0; j < 4; ++j)
        {
            for (int k = 0; k < 4; ++k)
                EXPECT_NEAR(ExpS(j,k), ExpLogExpS(j,k), MATH_EPS);
        }

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(LogExpS(j), ExpLogLogExpS(j), MATH_EPS);
    }

//    // Exp(Log(T)) = T

//    // Ad(T,V) == T * V * invT
//    se3 V(random(-1,1), random(-1,1), random(-1,1),
//          random(-1,1), random(-1,1), random(-1,1));
//    SE3 T = Exp(V);

//    se3 AdTV = Ad(T,V);
//    Matrix4d AdTVmat_eig = AdTV.getEigenMatrix();

//    Matrix4d Teig = T.getEigenMatrix();
//    Matrix4d Veig = V.getEigenMatrix();
//    Matrix4d invTeig = Inv(T).getEigenMatrix();
//    Matrix4d TVinvT = Teig * Veig * invTeig;

//    //EXPECT_EQ(AdTVmat_eig, TVinvT);

//    EXPECT_EQ(Ad(T, Ad(Inv(T), V)), V);
}

/// @brief
Eigen::Matrix<double,6,6> Ad(const SE3& T)
{
    Eigen::Matrix<double,6,6> AdT = Eigen::Matrix<double,6,6>::Zero();

    // R
    AdT(0,0) =  T(0,0);   AdT(0,1) =  T(0,1);   AdT(0,2) =  T(0,2);
    AdT(1,0) =  T(1,0);   AdT(1,1) =  T(1,1);   AdT(1,2) =  T(1,2);
    AdT(2,0) =  T(2,0);   AdT(2,1) =  T(2,1);   AdT(2,2) =  T(2,2);

    // R
    AdT(3,3) =  T(0,0);   AdT(3,4) =  T(0,1);   AdT(3,5) =  T(0,2);
    AdT(4,3) =  T(1,0);   AdT(4,4) =  T(1,1);   AdT(4,5) =  T(1,2);
    AdT(5,3) =  T(2,0);   AdT(5,4) =  T(2,1);   AdT(5,5) =  T(2,2);

    // [p]R
    AdT(3,0) = T(1,3)*T(2,0) - T(2,3)*T(1,0);   AdT(3,1) = T(1,3)*T(2,1) - T(2,3)*T(1,1);   AdT(3,2) = T(1,3)*T(2,2) - T(2,3)*T(1,2);
    AdT(4,0) = T(2,3)*T(0,0) - T(0,3)*T(2,0);   AdT(4,1) = T(2,3)*T(0,1) - T(0,3)*T(2,1);   AdT(4,2) = T(2,3)*T(0,2) - T(0,3)*T(2,2);
    AdT(5,0) = T(0,3)*T(1,0) - T(1,3)*T(0,0);   AdT(5,1) = T(0,3)*T(1,1) - T(1,3)*T(0,1);   AdT(5,2) = T(0,3)*T(1,2) - T(1,3)*T(0,2);

    return AdT;
}

/// @brief
Eigen::Matrix<double,6,6> dAd(const SE3& T)
{
    return Ad(T).transpose();
}

/******************************************************************************/
TEST(MATH, ADJOINT_MAPPING)
{
    double min = -100;
    double max = 100;
    math::se3 t(random(min,max), random(min,max), random(min,max),
                random(min,max), random(min,max), random(min,max));
    math::SE3 T = math::Exp(t);
    math::se3 S(random(min,max), random(min,max), random(min,max),
                random(min,max), random(min,max), random(min,max));

    Eigen::VectorXd AdT_S = math::Ad(T, S).getEigenVector();
    Eigen::VectorXd AdT_S2 = Ad(T) * S.getEigenVector();

    Eigen::MatrixXd I = Ad(T) * Ad(math::Inv(T));
    Eigen::MatrixXd SE3Identity = T.getEigenMatrix() * (math::Inv(T)).getEigenMatrix();

    for (int i = 0; i < 6; i++)
        EXPECT_NEAR(AdT_S(i), AdT_S2(i), MATH_TOL);
}

/******************************************************************************/
TEST(MATH, INERTIA)
{
    for (int k = 0; k < 100; ++k)
    {
        double min = -10;
        double max = 10;
        math::Vec3 r(random(min,max), random(min,max), random(min,max));
        math::SE3 Tr(r);
        math::Inertia I(random(0.1,max), random(0.1,max), random(0.1,max),
                        random(0.1,max), random(0.1,max), random(0.1,max),
                        random(min,max), random(min,max), random(min,max),
                        random(0.1,max));
        math::Inertia Ioffset = I;

        I.setOffset(math::Vec3(0, 0, 0));
        Ioffset.setOffset(r);

        Eigen::MatrixXd G = I.toTensor();
        Eigen::MatrixXd Goffset = Ioffset.toTensor();
        Eigen::MatrixXd dAdinvTGAdinvT = dAd(math::Inv(Tr)) * G * Ad(math::Inv(Tr));

        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                EXPECT_NEAR(Goffset(i,j), dAdinvTGAdinvT(i,j), MATH_TOL);
    }
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
