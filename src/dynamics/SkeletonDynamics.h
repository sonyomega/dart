/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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

#ifndef DART_DYNAMICS_SKELETONDYNAMICS_H
#define DART_DYNAMICS_SKELETONDYNAMICS_H

#include <vector>
#include <Eigen/Dense>

#include "dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

class BodyNode;

/// @brief
class SkeletonDynamics : public dynamics::Skeleton
{
public:
    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// @brief
    SkeletonDynamics();

    /// @brief
    virtual ~SkeletonDynamics();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    BodyNode* createBodyNode() const;
    BodyNode* getBody(int i) const;

    //--------------------------------------------------------------------------
    // Dynamical Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setImmobileState(bool _immobile) { mImmobile = _immobile; }

    /// @brief
    bool getImmobileState() const { return mImmobile; }

    /// @brief
    /// @todo Let's set this state for each joints
    DEPRECATED bool getJointLimitState() const { return mJointLimit; }

    /// @brief
    /// @todo Let's set this state for each joints
    DEPRECATED void setJointLimitState(bool _s) { mJointLimit = _s; }

    /// @brief
    double getTotalMass() const { return mTotalMass; }

    //--------------------------------------------------------------------------
    // Dynamics Equation
    //--------------------------------------------------------------------------
    /// @brief
    Eigen::MatrixXd getMassMatrix() const { return mM; }

    /// @brief
    Eigen::MatrixXd getInvMassMatrix() const { return mMInv; }

    Eigen::MatrixXd getCoriolisMatrix() const { return mC; }
    Eigen::VectorXd getCoriolisVector() const { return mCvec; }
    Eigen::VectorXd getGravityVector() const { return mG; }
    Eigen::VectorXd getCombinedVector() const { return mCg; }
    Eigen::VectorXd getExternalForces() const { return mFext; }
    Eigen::VectorXd getInternalForces() const { return get_tau(); }

    //--------------------------------------------------------------------------
    // Recursive Dynamics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    void initDynamics();

    /// @brief (q, dq, ddq) --> (tau)
    void computeInverseDynamics(const Eigen::Vector3d& _gravity);
    void _inverseDynamicsFwdRecursion();
    void _inverseDynamicsBwdRecursion(const Eigen::Vector3d& _gravity);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamics(const Eigen::Vector3d& _gravity,
                                bool _equationsOfMotion = true);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamicsID(const Eigen::Vector3d& _gravity,
                                  bool _equationsOfMotion = true);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamicsFS(const Eigen::Vector3d& _gravity,
                                  bool _equationsOfMotion = true);

    /// @brief (q, dq, ddq_v, tau_u) --> (tau_v, ddq_u)
    void computeHybridDynamicsFS(const Eigen::Vector3d& _gravity,
                                 bool _equationsOfMotion = true);

    /// @brief (q, dq) --> M, C, G
    void computeEquationsOfMotionID(const Eigen::Vector3d& _gravity);

    /// @brief (q, dq) --> M, C, G
    void computeEquationsOfMotionRecursive(const Eigen::Vector3d& _gravity);

protected:
    /// @brief

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief If the skeleton is immobile, its dynamic effect is equivalent to
    /// having infinite mass. If the DOFs of an immobile skeleton are manually
    /// changed, the collision results might not be correct.
    bool mImmobile;

    /// @brief True if the joint limits are enforced in dynamic simulation.
    bool mJointLimit;
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    double mTotalMass;

    /// @brief Mass matrix for the skeleton.
    Eigen::MatrixXd mM;

    /// @brief Inverse of mass matrix for the skeleton.
    Eigen::MatrixXd mMInv;

    Eigen::MatrixXd mC;    ///< Coriolis matrix for the skeleton; not being used currently
    Eigen::VectorXd mCvec;    ///< Coriolis vector for the skeleton == mC*qdot
    Eigen::VectorXd mG;    ///< Gravity vector for the skeleton; computed in nonrecursive dynamics only
    Eigen::VectorXd mCg;   ///< combined coriolis and gravity term == mC*qdot + g
    Eigen::VectorXd mFext; ///< external forces vector for the skeleton
    //Eigen::VectorXd mFint; ///< internal forces vector for the skeleton; computed by an external controller

    /// @brief
    //std::vector<BodyNodeDynamics*> mDynamicsBodies;

private:

public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_SKELETONDYNAMICS_H
