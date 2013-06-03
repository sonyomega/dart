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

#define MATH_TOL 0.01
#define MATH_EPS 0.000001

/******************************************************************************/
TEST(MATH, SE3_VS_EIGENMATRIX4D)
{
    int n = 5000;
    double min = -100;
    double max = 100;

    SE3 T1 = Exp(se3(random(min,max), random(min,max), random(min,max),
                     random(min,max), random(min,max), random(min,max)));
    SE3 T2 = Exp(se3(random(min,max), random(min,max), random(min,max),
                     random(min,max), random(min,max), random(min,max)));
    SE3 T3 = Exp(se3(random(min,max), random(min,max), random(min,max),
                     random(min,max), random(min,max), random(min,max)));
    SE3 T4 = Exp(se3(random(min,max), random(min,max), random(min,max),
                     random(min,max), random(min,max), random(min,max)));

    Eigen::Matrix4d E1 = T1.getEigenMatrix();
    Eigen::Matrix4d E2 = T2.getEigenMatrix();
    Eigen::Matrix4d E3 = T3.getEigenMatrix();
    Eigen::Matrix4d E4 = T4.getEigenMatrix();

    {
        Timer SE3Timer("SE3 timer");
        SE3Timer.startTimer();
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                T3 *= T1 * T2;
        SE3Timer.stopTimer();
    }
    std::cout << T3 << std::endl;

    {
        Timer EigenTimer("Eigen::Matrix4d timer");
        EigenTimer.startTimer();
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                E3 *= E1 * E2;
        EigenTimer.stopTimer();
    }
    std::cout << E3 << std::endl;

    {
        Timer SE3Timer("SE3 inverse timer");
        SE3Timer.startTimer();
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                T4 *= Inv(T1) * Inv(T2);
        SE3Timer.stopTimer();
    }
    std::cout << T4 << std::endl;

    {
        Timer EigenTimer("Eigen::Matrix4d inverse timer");
        EigenTimer.startTimer();
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                E4 *= E1.inverse() * E2.inverse();
        EigenTimer.stopTimer();
    }
    std::cout << E4 << std::endl;
}

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





    // Exp(Log(T)) = T

    // Ad(T,V) == T * V * invT
    se3 V(random(-1,1), random(-1,1), random(-1,1),
          random(-1,1), random(-1,1), random(-1,1));
    se3 t(random(-1,1), random(-1,1), random(-1,1),
          random(-1,1), random(-1,1), random(-1,1));
    SE3 T = Exp(t);

    se3 AdTV = Ad(T,V);
    Matrix4d AdTVmat_eig = AdTV.getEigenMatrix();

    Matrix4d Teig = T.getEigenMatrix();
    Matrix4d Veig = V.getEigenMatrix();
    Matrix4d invTeig = Inv(T).getEigenMatrix();
    Matrix4d TVinvT = Teig * Veig * invTeig;

    //EXPECT_EQ(AdTVmat_eig, TVinvT);

    EXPECT_EQ(Ad(T, Ad(Inv(T), V)), V);
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
