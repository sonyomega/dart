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

#include "utils/UtilsCode.h"
#include "math/LieGroup.h"
#include "math/UtilsMath.h"
#include "kinematics/BallJoint.h"
#include "kinematics/RevoluteJoint.h"
#include "kinematics/FreeJoint.h"
#include "kinematics/TranslationalJoint.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "simulation/World.h"

using namespace dart;
using namespace math;
using namespace kinematics;
using namespace dynamics;
using namespace simulation;

#define JOINT_TOL 0.01

/******************************************************************************/
class JOINTS : public testing::Test
{
public:
    void kinematicsTest(Joint* _joint);
};

/******************************************************************************/
void JOINTS::kinematicsTest(Joint* _joint)
{
    int dof = _joint->getDOF();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    VectorXd q = VectorXd::Zero(dof);
    VectorXd dq = VectorXd::Zero(dof);
    VectorXd ddq = VectorXd::Zero(dof);

    double dt = 0.000001;

    for (int i = 0; i < dof; ++i)
    {
        q(i) = random(-M_PI, M_PI);
        dq(i) = random(-M_PI, M_PI);
        ddq(i) = random(-M_PI, M_PI);
    }

    _joint->set_q(q);
    _joint->set_dq(dq);
    _joint->set_ddq(ddq);

    _joint->updateKinematics();

    SE3 T = _joint->getLocalTransformation();
    se3 V = _joint->getLocalVelocity();
    Jacobian J = _joint->getLocalJacobian();
    se3 dV = _joint->getLocalAcceleration();
    Jacobian dJ = _joint->getLocalJacobianFirstDerivative();

    // Test V == J * dq
    se3 Jdq = J * _joint->get_dq();
    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(V(i), Jdq(i), JOINT_TOL);

    // Test dV == dJ * dq + J * ddq
    se3 dJdq = dJ * _joint->get_dq();
    se3 Jddq = J * _joint->get_ddq();
    se3 dJdq_Jddq = dJdq + Jddq;
    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(dV(i), dJdq_Jddq(i), JOINT_TOL);

    //--------------------------------------------------------------------------
    // Test analytic Jacobian and numerical Jacobian
    //--------------------------------------------------------------------------
    Jacobian numericJ(dof);
    for (int i = 0; i < dof; ++i)
    {
        // a
        VectorXd q_a = q;
        _joint->set_q(q_a);
        _joint->updateKinematics();
        SE3 T_a = _joint->getLocalTransformation();

        // b
        VectorXd q_b = q;
        q_b(i) += dt;
        _joint->set_q(q_b);
        _joint->updateKinematics();
        SE3 T_b = _joint->getLocalTransformation();

        //
        SE3 Tinv_a = Inv(T_a);
        Matrix4d Tinv_a_eigen = Tinv_a.getEigenMatrix();

        // dTdq
        Matrix4d T_a_eigen = T_a.getEigenMatrix();
        Matrix4d T_b_eigen = T_b.getEigenMatrix();
        Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / dt;
        //Matrix4d dTdq_eigen = (T_b_eigen * T_a_eigen.inverse()) / dt;

        // J(i)
        Matrix4d Ji_4x4matrix_eigen = Tinv_a_eigen * dTdq_eigen;
        se3 Ji;
        Ji.setFromMatrixForm(Ji_4x4matrix_eigen);
        numericJ.setColumn(i, Ji);
    }

    for (int i = 0; i < dof; ++i)
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(J[i](j), numericJ[i](j), JOINT_TOL);
}

TEST_F(JOINTS, REVOLUTE_JOINT)
{
    RevoluteJoint revJoint;

    kinematicsTest(&revJoint);
}

TEST_F(JOINTS, BALL_JOINT)
{
    BallJoint ballJoint;

    kinematicsTest(&ballJoint);
}

TEST_F(JOINTS, TRANSLATIONAL_JOINT)
{
    TranslationalJoint translationalJoint;

    kinematicsTest(&translationalJoint);
}

TEST_F(JOINTS, FREE_JOINT)
{
    FreeJoint freeJoint;

    kinematicsTest(&freeJoint);
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


