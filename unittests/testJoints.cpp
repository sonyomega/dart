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

#include "dart/common/UtilsCode.h"
#include "dart/math/LieGroup.h"
#include "dart/math/UtilsMath.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"

using namespace dart;
using namespace math;
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

    for (int idxTest = 0; idxTest < 100; ++idxTest)
    {
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

        if (_joint->getDOF() == 0)
            return;

        SE3 T = _joint->getLocalTransformation();
        se3 V = _joint->getLocalVelocity();
        Jacobian J = _joint->getLocalJacobian();
        se3 dV = _joint->getLocalAcceleration();
        Jacobian dJ = _joint->getLocalJacobianFirstDerivative();

        //--------------------------------------------------------------------------
        // Test V == J * dq
        //--------------------------------------------------------------------------
        se3 Jdq = J * _joint->get_dq();
        for (int i = 0; i < 6; ++i)
            EXPECT_NEAR(V(i), Jdq(i), JOINT_TOL);

        //--------------------------------------------------------------------------
        // Test dV == dJ * dq + J * ddq
        //--------------------------------------------------------------------------
        se3 dJdq = dJ * _joint->get_dq();
        se3 Jddq = J * _joint->get_ddq();
        se3 dJdq_Jddq = dJdq + Jddq;
        for (int i = 0; i < 6; ++i)
            EXPECT_NEAR(dV(i), dJdq_Jddq(i), JOINT_TOL);

        //--------------------------------------------------------------------------
        // Test analytic Jacobian and numerical Jacobian
        // J == numericalJ
        //--------------------------------------------------------------------------
        Jacobian numericJ = Jacobian::Zero(6,dof);
        for (int i = 0; i < dof; ++i)
        {
            // a
            Eigen::VectorXd q_a = q;
            _joint->set_q(q_a);
            _joint->updateKinematics();
            SE3 T_a = _joint->getLocalTransformation();

            // b
            Eigen::VectorXd q_b = q;
            q_b(i) += dt;
            _joint->set_q(q_b);
            _joint->updateKinematics();
            SE3 T_b = _joint->getLocalTransformation();

            //
            SE3 Tinv_a = Inv(T_a);
            Eigen::Matrix4d Tinv_a_eigen = Tinv_a.matrix();

            // dTdq
            Eigen::Matrix4d T_a_eigen = T_a.matrix();
            Eigen::Matrix4d T_b_eigen = T_b.matrix();
            Eigen::Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / dt;
            //Matrix4d dTdq_eigen = (T_b_eigen * T_a_eigen.inverse()) / dt;

            // J(i)
            Eigen::Matrix4d Ji_4x4matrix_eigen = Tinv_a_eigen * dTdq_eigen;
            se3 Ji;
            Ji[0] = Ji_4x4matrix_eigen(2,1);
            Ji[1] = Ji_4x4matrix_eigen(0,2);
            Ji[2] = Ji_4x4matrix_eigen(1,0);
            Ji[3] = Ji_4x4matrix_eigen(0,3);
            Ji[4] = Ji_4x4matrix_eigen(1,3);
            Ji[5] = Ji_4x4matrix_eigen(2,3);
            numericJ.col(i) = Ji;
        }

        for (int i = 0; i < dof; ++i)
            for (int j = 0; j < 6; ++j)
                EXPECT_NEAR(J.col(i)(j), numericJ.col(i)(j), JOINT_TOL);
    }
}

// 0-dof joint
TEST_F(JOINTS, WELD_JOINT)
{
    WeldJoint weldJoint;

    kinematicsTest(&weldJoint);
}

// 1-dof joint
TEST_F(JOINTS, REVOLUTE_JOINT)
{
    RevoluteJoint revJoint;

    kinematicsTest(&revJoint);
}

// 1-dof joint
TEST_F(JOINTS, PRISMATIC_JOINT)
{
    PrismaticJoint priJoint;

    kinematicsTest(&priJoint);
}

// 3-dof joint
TEST_F(JOINTS, BALL_JOINT)
{
    BallJoint ballJoint;

    kinematicsTest(&ballJoint);
}

// 3-dof joint
TEST_F(JOINTS, EULER_XYZ_JOINT)
{
    EulerXYZJoint eulerXYZJoint;

    kinematicsTest(&eulerXYZJoint);
}

// 3-dof joint
TEST_F(JOINTS, TRANSLATIONAL_JOINT)
{
    TranslationalJoint translationalJoint;

    kinematicsTest(&translationalJoint);
}

// 6-dof joint
//TEST_F(JOINTS, FREE_JOINT)
//{
//    FreeJoint freeJoint;

//    kinematicsTest(&freeJoint);
//}

TEST_F(JOINTS, BALL_JOINT_JACOBIAN)
{
    Eigen::Vector3d q;
    q << 0, 0, 0;
    bool isInvertible = true;

    do
    {
        q = Eigen::Vector3d::Random();
        double theta = q.norm();

        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d qss = math::makeSkewSymmetric(q);
        Eigen::Matrix3d qss2 =  qss*qss;

        if(theta < 1e-6)
            J = Eigen::Matrix3d::Identity() + 0.5*qss +  (1.0/6.0)*qss2;
        else
            J = Eigen::Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;

        Eigen::MatrixXd S(6,3);

        S.col(0) << J(0,0), J(0,1), J(0,2), 0, 0, 0;
        S.col(1) << J(1,0), J(1,1), J(1,2), 0, 0, 0;
        S.col(2) << J(2,0), J(2,1), J(2,2), 0, 0, 0;

        Eigen::MatrixXd Psi = S.transpose() * S;

        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(Psi);
        isInvertible = lu_decomp.isInvertible();

        Eigen::MatrixXd invPsi = Psi.inverse();

//        for (int i = 0; i < 3; ++i)
//            for (int j = 0; j < 3; ++j)
//                if (invPsi(i,j) != invPsi(i,j))
//                    assert(0);


        if (lu_decomp.rank() < 3)
            assert(0);

    } while (1);
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


