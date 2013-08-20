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

#include "common/UtilsCode.h"
#include "math/LieGroup.h"
#include "math/UtilsMath.h"
#include "dynamics/BallJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/PrismaticJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/WeldJoint.h"
#include "dynamics/TranslationalJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

#define LIE_GROUP_OPT_TOL 1e-12

/******************************************************************************/
Eigen::Matrix4d toMatrixForm(const Eigen::Vector6d& v) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Zero();

    result(0, 1) = -v(2);
    result(1, 0) =  v(2);
    result(0, 2) =  v(1);
    result(2, 0) = -v(1);
    result(1, 2) = -v(0);
    result(2, 1) =  v(0);

    result(0, 3) = v(3);
    result(1, 3) = v(4);
    result(2, 3) = v(5);

    return result;
}

/******************************************************************************/
Eigen::Vector6d fromMatrixForm(const Eigen::Matrix4d& m) {
    Eigen::Vector6d ret;
    ret << m(2,1), m(0,2), m(1,0), m(0,3), m(1,3), m(2,3);
    return ret;
}

/******************************************************************************/
#define EPSILON_EXPMAP_THETA 1.0e-3
TEST(LIE_GROUP_OPERATORS, EXPONENTIAL_MAPPINGS)
{
    int numTest = 100;

    // Exp
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 s = math::se3::Random();
        math::SE3 Exp_s = math::Exp(s);
        Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

        double theta = s.head<3>().norm();
        Eigen::Matrix3d R = Matrix3d::Zero();
        Eigen::Matrix3d qss =  math::makeSkewSymmetric(s.head<3>());
        Eigen::Matrix3d qss2 =  qss*qss;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

        if (theta < EPSILON_EXPMAP_THETA)
        {
            R = Matrix3d::Identity() + qss + 0.5*qss2;
            P = Matrix3d::Identity() + 0.5*qss + (1/6)*qss2;
        }
        else
        {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
            P = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }

        Exp_s_2.topLeftCorner<3,3>() = R;
        Exp_s_2.topRightCorner<3,1>() = P*s.tail<3>();

        //
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                EXPECT_NEAR(Exp_s(i,j), Exp_s_2(i,j), LIE_GROUP_OPT_TOL);
    }

    // ExpAngular
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 s = math::se3::Random();
        s.tail<3>() = Eigen::Vector3d::Zero();
        math::SE3 Exp_s = math::ExpAngular(s.head<3>());
        Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

        double theta = s.head<3>().norm();
        Eigen::Matrix3d R = Matrix3d::Zero();
        Eigen::Matrix3d qss =  math::makeSkewSymmetric(s.head<3>());
        Eigen::Matrix3d qss2 =  qss*qss;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

        if (theta < EPSILON_EXPMAP_THETA)
        {
            R = Matrix3d::Identity() + qss + 0.5*qss2;
            P = Matrix3d::Identity() + 0.5*qss + (1/6)*qss2;
        }
        else
        {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
            P = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }

        Exp_s_2.topLeftCorner<3,3>() = R;
        Exp_s_2.topRightCorner<3,1>() = P*s.tail<3>();

        //
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                EXPECT_NEAR(Exp_s(i,j), Exp_s_2(i,j), LIE_GROUP_OPT_TOL);
    }

    // ExpLinear
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 s = math::se3::Random();
        s.head<3>() = Eigen::Vector3d::Zero();
        math::SE3 Exp_s = math::ExpLinear(s.tail<3>());
        Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

        double theta = s.head<3>().norm();
        Eigen::Matrix3d R = Matrix3d::Zero();
        Eigen::Matrix3d qss =  math::makeSkewSymmetric(s.head<3>());
        Eigen::Matrix3d qss2 =  qss*qss;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

        if (theta < EPSILON_EXPMAP_THETA)
        {
            R = Matrix3d::Identity() + qss + 0.5*qss2;
            P = Matrix3d::Identity() + 0.5*qss + (1/6)*qss2;
        }
        else
        {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
            P = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }

        Exp_s_2.topLeftCorner<3,3>() = R;
        Exp_s_2.topRightCorner<3,1>() = P*s.tail<3>();

        //
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                EXPECT_NEAR(Exp_s(i,j), Exp_s_2(i,j), LIE_GROUP_OPT_TOL);
    }
}

/******************************************************************************/
TEST(LIE_GROUP_OPERATORS, ADJOINT_MAPPINGS)
{
    int numTest = 100;

    // AdT(V) == T * V * InvT
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::se3 V = math::se3::Random();

        math::se3 AdTV = AdT(T, V);

        // Ad(T, V) = T * [V] * InvT
        Eigen::Matrix4d T_V_InvT = T.matrix() * toMatrixForm(V) * T.inverse().matrix();
        math::se3 T_V_InvT_se3 = fromMatrixForm(T_V_InvT);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), T_V_InvT_se3(j), LIE_GROUP_OPT_TOL);

        // Ad(T, V) = [R 0; [p]R R] * V
        Eigen::Matrix6d AdTMatrix = Eigen::Matrix6d::Zero();
        AdTMatrix.topLeftCorner<3,3>() = T.rotation();
        AdTMatrix.bottomRightCorner<3,3>() = T.rotation();
        AdTMatrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(T.translation()) * T.rotation();
        math::se3 AdTMatrix_V = AdTMatrix * V;
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTMatrix_V(j), LIE_GROUP_OPT_TOL);
    }

    // AdR == AdT([R 0; 0 1], V)
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::SE3 R = math::SE3::Identity();
        R = T.rotation();
        math::se3 V = math::se3::Random();

        math::se3 AdTV = AdT(R, V);
        math::se3 AdRV = AdR(T, V);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdRV(j), LIE_GROUP_OPT_TOL);
    }

    // AdTAngular == AdT(T, se3(w, 0))
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::Axis w = math::Axis::Random();
        math::se3 V = math::se3::Zero();
        V.head<3>() = w;

        math::se3 AdTV = AdT(T, V);
        math::se3 AdTAng = AdTAngular(T, w);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTAng(j), LIE_GROUP_OPT_TOL);
    }

    // AdTLinear == AdT(T, se3(w, 0))
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::Vec3 v = math::Vec3::Random();
        math::se3 V = math::se3::Zero();
        V.tail<3>() = v;

        math::se3 AdTV = AdT(T, V);
        math::se3 AdTLin = AdTLinear(T, v);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTLin(j), LIE_GROUP_OPT_TOL);
    }

    // AdTJac
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::Vec3 v = math::Vec3::Random();
        math::se3 V = math::se3::Zero();
        V.tail<3>() = v;

        math::se3 AdTV = AdT(T, V);
        math::se3 AdTLin = AdTLinear(T, v);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTLin(j), LIE_GROUP_OPT_TOL);
    }

    // AdInvT
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::SE3 InvT = T.inverse();
        math::se3 V = math::se3::Random();

        math::se3 Ad_InvT = AdT(InvT, V);
        math::se3 AdInv_T = AdInvT(T, V);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(Ad_InvT(j), AdInv_T(j), LIE_GROUP_OPT_TOL);
    }

    // AdInvRLinear
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::Vec3 v = math::Vec3::Random();
        math::se3 V = math::se3::Zero();
        V.tail<3>() = v;
        math::SE3 R = math::SE3::Identity();
        R = T.rotation();

        math::se3 AdT_ = AdT(R.inverse(), V);
        math::se3 AdInvRLinear_ = AdInvRLinear(T, v);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdT_(j), AdInvRLinear_(j), LIE_GROUP_OPT_TOL);
    }

    // dAdT
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::dse3 F = math::se3::Random();

        math::dse3 dAdTF = dAdT(T, F);

        // dAd(T, F) = [R 0; [p]R R]^T * F
        Eigen::Matrix6d AdTMatrix = Eigen::Matrix6d::Zero();
        AdTMatrix.topLeftCorner<3,3>() = T.rotation();
        AdTMatrix.bottomRightCorner<3,3>() = T.rotation();
        AdTMatrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(T.translation()) * T.rotation();
        math::se3 AdTTransMatrix_V = AdTMatrix.transpose() * F;
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdTF(j), AdTTransMatrix_V(j), LIE_GROUP_OPT_TOL);
    }

    // dAdInvT
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::SE3 InvT = T.inverse();
        math::dse3 F = math::se3::Random();

        math::dse3 dAdInvT_F = dAdInvT(T, F);

        //
        math::dse3 dAd_InvTF = dAdT(InvT, F);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdInvT_F(j), dAd_InvTF(j), LIE_GROUP_OPT_TOL);

        // dAd(T, F) = [R 0; [p]R R]^T * F
        Eigen::Matrix6d AdInvTMatrix = Eigen::Matrix6d::Zero();
        AdInvTMatrix.topLeftCorner<3,3>() = InvT.rotation();
        AdInvTMatrix.bottomRightCorner<3,3>() = InvT.rotation();
        AdInvTMatrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(InvT.translation()) * InvT.rotation();
        math::se3 AdInvTTransMatrix_V = AdInvTMatrix.transpose() * F;
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdInvT_F(j), AdInvTTransMatrix_V(j), LIE_GROUP_OPT_TOL);
    }

    // dAdInvR
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 t = math::se3::Random();
        math::SE3 T = math::Exp(t);
        math::SE3 InvT = T.inverse();
        math::SE3 InvR = math::SE3::Identity();
        InvR = InvT.rotation();
        math::dse3 F = math::se3::Random();

        math::dse3 dAdInvR_F = dAdInvR(T, F);

        //
        math::dse3 dAd_InvTF = dAdT(InvR, F);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdInvR_F(j), dAd_InvTF(j), LIE_GROUP_OPT_TOL);
    }

    // ad
    for (int i = 0; i < numTest; ++i)
    {
        math::se3 V = math::se3::Random();
        math::se3 W = math::se3::Random();

        math::se3 ad_V_W = ad(V, W);

        //
        Eigen::Matrix6d adV_Matrix = Eigen::Matrix6d::Zero();
        adV_Matrix.topLeftCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        adV_Matrix.bottomRightCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        adV_Matrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(V.tail<3>());
        math::se3 adV_Matrix_W = adV_Matrix * W;

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(ad_V_W(j), adV_Matrix_W(j), LIE_GROUP_OPT_TOL);
    }

    // dad
    for (int i = 0; i < numTest; ++i)
    {
        math::dse3 V = math::se3::Random();
        math::dse3 F = math::dse3::Random();

        math::dse3 dad_V_F = dad(V, F);

        //
        Eigen::Matrix6d dadV_Matrix = Eigen::Matrix6d::Zero();
        dadV_Matrix.topLeftCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        dadV_Matrix.bottomRightCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        dadV_Matrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(V.tail<3>());
        math::se3 dadV_Matrix_F= dadV_Matrix.transpose() * F;

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dad_V_F(j), dadV_Matrix_F(j), LIE_GROUP_OPT_TOL);
    }
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

