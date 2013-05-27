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

#ifndef DART_DYNAMICS_BODYNODE_DYNAMICS_H
#define DART_DYNAMICS_BODYNODE_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

#include "kinematics/BodyNode.h"
#include "dynamics/Inertia.h"

namespace dart {
namespace dynamics {

/// @brief BodyNodeDynamics class represents a single node of the skeleton for
/// dynamics.
class BodyNodeDynamics : public kinematics::BodyNode
{
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTORS AND DESTRUCTOR
    //--------------------------------------------------------------------------
    /// @brief Default constructor.
    BodyNodeDynamics();

    /// @brief Default destructor
    virtual ~BodyNodeDynamics();

    //--------------------------------------------------------------------------
    // Dynamics Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setMass(double _mass) { mI.setMass(_mass); }

    /// @brief
    double getMass() const { return mI.getMass(); }

    /// @brief
    DEPRECATED void setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                                    double _Ixy, double _Ixz, double _Iyz);
    void setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                            double _Ixy, double _Ixz, double _Iyz);

    /// @brief
    const Inertia& getInertia() const { return mI; }

    //void setLocalInertia(const Eigen::Matrix3d& _inertia) { mI = _inertia; }
    //Eigen::Matrix3d getLocalInertia() const { return mI; }
    //Eigen::Matrix3d getWorldInertia() const { return mIc; }
    //DEPRECATED Eigen::Matrix3d getInertia() const { return mIc; } ///< Superseded by getWorldInertia()
    /// @brief Computes the "mass tensor" in lagrangian dynamics from the
    /// inertia matrix
    //Eigen::Matrix4d getMassTensor();

    /// @brief
    DEPRECATED Eigen::Vector3d getLocalCOM() const { return getCenterOfMass(); }
    void setCenterOfMass(const Eigen::Vector3d& _com)
    { mI.setCenterOfMass(_com); }

    /// @brief
    DEPRECATED void setLocalCOM(const Eigen::Vector3d& _off)
    { setCenterOfMass(_off); }
    Eigen::Vector3d getCenterOfMass() const { return mI.getCenterOfMass(); }

    //DEPRECATED Eigen::Vector3d getWorldCOM() { return evalWorldPos(mCOMLocal); }

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setGravityMode(bool _onoff) { mGravityMode = _onoff; }

    /// @brief
    bool getGravityMode() const { return mGravityMode; }

    /// @brief
    void setExternalForceLocal(const math::dse3& _FextLocal);

    /// @brief
    void setExternalForceGlobal(const math::dse3& _FextWorld);

    /// @brief
    void setExternalForceLocal(const Eigen::Vector3d& _posLocal,
                               const Eigen::Vector3d& _linearForceGlobal);

    /// @brief apply linear Cartesian forces to this node.
    ///
    /// A force is defined by a point of application and a force vector. The
    /// last two parameters specify frames of the first two parameters.
    /// Coordinate transformations are applied when needed. The point of
    /// application and the force in local coordinates are stored in mContacts.
    /// When conversion is needed, make sure the transformations are avaialble.
    void addExtForce(const Eigen::Vector3d& _offset,
                     const Eigen::Vector3d& _force,
                     bool _isOffsetLocal = true, bool _isForceLocal = false );

    /// @brief apply Cartesian torque to the node.
    ///
    /// The torque in local coordinates is accumulated in mExtTorqueBody.
    void addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal);

    /// @brief Clean up structures that store external forces: mContacts, mFext,
    /// mExtForceBody and mExtTorqueBody.
    ///
    /// Called from @SkeletonDynamics::clearExternalForces.
    void clearExternalForces();

    /// @brief
    void addExternalForceLocal(const math::dse3& _FextLocal);

    /// @brief
    void addExternalForceGlobal(const math::dse3& _FextWorld);

    /// @brief
    void addExternalForceLocal(const Eigen::Vector3d& _posLocal,
                               const Eigen::Vector3d& _linearForceGlobal);

    /// @brief
    const math::dse3& getExternalForceLocal() const { return mFext; }

    /// @brief
    math::dse3 getExternalForceGlobal() const;

    /// @brief
    const math::dse3& getBodyForce() const { return mF; }

    //--------------------------------------------------------------------------
    // Recursive Dynamics Algorithms
    //--------------------------------------------------------------------------
    /// @brief

    //--------------------------------------------------------------------------
    // Sub-functions for Recursive Dynamics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    /// childBodies.F, V, dV --> F, Fgravity
    void _updateBodyForce(const Eigen::Vector3d& _gravity);

    /// @brief
    /// parentJoint.S, F --> tau
    void _updateGeneralizedForce();

    /// @brief
    void _updateDampingForce();

protected:
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// TODO: Not implemented yet!
    bool mGravityMode;

    /// @brief Generalized inertia.
    dynamics::Inertia mI;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief Generalized body force w.r.t. body frame.
    math::dse3 mF;

    /// @brief
    math::dse3 mFext;

    /// @brief
    math::dse3 mFgravity;

private:

public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_BODYNODE_DYNAMICS_H
