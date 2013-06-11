/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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

#ifndef DART_KINEMATICS_SKELETON_H
#define DART_KINEMATICS_SKELETON_H

#include <vector>

#include <Eigen/Dense>

#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include "math/LieGroup.h"
#include "utils/Deprecated.h"
#include "dynamics/System.h"

namespace dart {

namespace renderer {
class RenderInterface;
}

namespace dynamics {

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> SkeletonGraph;
class BodyNode;
class Joint;

/// @brief
class Skeleton : public GenCoordSystem
{
public:
    //--------------------------------------------------------------------------
    // Constructor and Destructor
    //--------------------------------------------------------------------------
    /// @brief
    Skeleton(const std::string& _name = "");

    /// @brief
    virtual ~Skeleton();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setName(const std::string& _name) { mName = _name; }

    /// @brief
    const std::string& getName() const { return mName; }

    /// @brief
    void setSelfCollidable(bool _selfCollidable)
    { mSelfCollidable = _selfCollidable; }

    /// @brief
    bool getSelfCollidable() const { return mSelfCollidable; }

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
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    void addBody(BodyNode* _body, bool _addParentJoint = true);

    /// @brief
    void addJoint(Joint* _joint);

    /// @brief
    //void setRootBody(BodyNode* _body) { mRootBody = _body; }

    /// @brief
    // TODO: rename
    int getNumBodies() const { return mBodies.size(); }
    int getNumNodes() const { return mBodies.size(); }

    /// @brief
    int getNumJoints() const { return mJoints.size(); }

    /// @brief
    // TODO: rename
    BodyNode* getBody(int _idx) const { return mBodies[_idx]; }
    BodyNode* getNode(int _idx) const { return mBodies[_idx]; }

    /// @brief
    // TODO: rename
    //BodyNode* findBody(const std::string& _name) const;
    BodyNode* findBody(const std::string& _name) const;

    /// @brief
    // TODO: Not implemented.
    Eigen::VectorXd getDependentConfiguration(BodyNode* _beginBody,
                                              BodyNode* _endBody) const;

    //--------------------------------------------------------------------------
    // Properties updated by dynamics (kinematics)
    //--------------------------------------------------------------------------
    /// @brief
    /// @todo Use set_q(const Eigen::VectorXd& _state) instead.
    DEPRECATED void setPose(const Eigen::VectorXd& _pose,
                            bool bCalcTrans = true,
                            bool bCalcDeriv = true);

    /// @brief
    /// @todo Use get_q() instead.
    DEPRECATED Eigen::VectorXd getPose() const { return get_q(); }

    /// @brief
    /// @todo Use get_dq() instead.
    DEPRECATED Eigen::VectorXd getPoseVelocity() const { return get_dq(); }

    /// @brief
    // TODO: Not implemented.
    math::Jacobian getJacobian(BodyNode* _beginBody,
                               BodyNode* _endBody) const;

    // Dynamics equation
    Eigen::MatrixXd getMassMatrix() const { return mM; }
    Eigen::MatrixXd getInvMassMatrix() const { return mMInv; }
    Eigen::MatrixXd getCoriolisMatrix() const { return mC; }
    Eigen::VectorXd getCoriolisVector() const { return mCvec; }
    Eigen::VectorXd getGravityVector() const { return mG; }
    Eigen::VectorXd getCombinedVector() const { return mCg; }
    Eigen::VectorXd getExternalForces() const { return mFext; }
    Eigen::VectorXd getInternalForces() const { return get_tau(); }
    Eigen::VectorXd getDampingForces() const;
    Eigen::VectorXd getConstraintForces() const { return mFc; }

    void setInternalForces(const Eigen::VectorXd& _forces) { set_tau(_forces); }
    void clearInternalForces() { set_tau(Eigen::VectorXd::Zero(getDOF())); }
    void setConstraintForces(const Eigen::VectorXd& _Fc) { mFc = _Fc; }

    /// @brief
    double getKineticEnergy() const;

    // TODO: Not implemented.
    /// @brief
    double getPotentialEnergy() const;

    // TODO: Not implemented.
    /// @brief
    math::Vec3 getPositionCOMGlobal();

    // TODO: Not implemented.
    /// @brief
    math::Vec3 getVelocityCOMGlobal();

    // TODO: Not implemented.
    /// @brief
    math::Vec3 getAccelerationCOMGlobal();

    // TODO: Not implemented.
    /// @brief
    math::dse3 getMomentumGlobal();

    // TODO: Not implemented.
    /// @brief
    math::dse3 getMomentumCOM();

    //--------------------------------------------------------------------------
    // Recursive kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    void initKinematics();

    /// @brief Update joint and body kinematics.
    void updateForwardKinematics(bool _firstDerivative = true,
                                 bool _secondDerivative = true);

    /// @brief Update joint dynamics (T, S, V, dS, dV)
    void _updateJointKinematics(bool _firstDerivative = true,
                                bool _secondDerivative = true);

    /// @brief Update body dynamics (W, V, dV)
    void _updateBodyForwardKinematics(bool _firstDerivative = true,
                                      bool _secondDerivative = true);

    // TODO: Inverse Kinematics

    //--------------------------------------------------------------------------
    // Recursive dynamics Algorithms
    //--------------------------------------------------------------------------

    /// @brief
    void initDynamics();

    /// @brief (q, dq, ddq) --> (tau)
    void computeInverseDynamics(const Eigen::Vector3d& _gravity,
                                bool _computeJacobian = true,
                                bool _computeJacobianDeriv = true,
                                bool _withExternalForces = false,
                                bool _withDampingForces = false);

    // TODO: Not implemeted.
    /// @brief
    void computeInverseDynamicsWithZeroAcceleration(
            const Eigen::Vector3d& _gravity,
            bool _withExternalForces = false);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamics(const Eigen::Vector3d& _gravity,
                                bool _equationsOfMotion = true);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamicsID(const Eigen::Vector3d& _gravity,
                                  bool _equationsOfMotion = true);

    /// @brief (q, dq, tau) --> (ddq)
    void computeForwardDynamicsID2(const Eigen::Vector3d& _gravity,
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

    //--------------------------------------------------------------------------
    // Rendering
    //--------------------------------------------------------------------------
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true) const;

protected:
    //--------------------------------------------------------------------------
    // Sub-functions for Recursive Algorithms
    //--------------------------------------------------------------------------

protected:
    /// @brief
    std::string mName;

    /// @brief
    bool mSelfCollidable;

    //--------------------------------------------------------------------------
    // Structual Properties
    //--------------------------------------------------------------------------
    /// @brief
    // TODO: rename
    //BodyNode* mRootBody;
    BodyNode* mRoot;

    /// @brief
    std::vector<BodyNode*> mBodies;

    /// @brief
    std::vector<Joint*> mJoints;



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
    Eigen::VectorXd mFc;

    /// @brief
    //std::vector<BodyNodeDynamics*> mDynamicsBodies;


    SkeletonGraph* mGraph;

private:

public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_SKELETON_H

