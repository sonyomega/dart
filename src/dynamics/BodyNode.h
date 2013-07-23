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

/*! \mainpage Dynamic Animation and Robotics Toolkits

DART is an open source library for developing kinematic and dynamic
applications in robotics and computer animation. DART features two
main components: a multibody dynamic simulator developed by Georgia
Tech Graphics Lab and a variety of motion planning algorithms
developed by Georgia Tech Humanoid Robotics Lab. This document focuses
only on the dynamic simulator.

The multibody dynamics simulator in DART is designed to aid
development of motion control algorithms. DART uses generalized
coordinates to represent articulated rigid body systems and computes
Lagrange’s equations derived from D’Alembert’s principle to describe
the dynamics of motion. In contrast to many popular physics engines
which view the simulator as a black box, DART provides full access to
internal kinematic and dynamic quantities, such as mass matrix,
Coriolis and centrifugal force, transformation matrices and their
derivatives, etc. DART also provides efficient computation of Jacobian
matrices for arbitrary body points and coordinate frames.

The contact and collision are handled using an implicit time-stepping,
velocity-based LCP (linear-complementarity problem) to guarantee
non-penetration, directional friction, and approximated Coulombs
friction cone conditions. The LCP problem is solved efficiently by
Lemke's algorithm. For the collision detection, DART directly uses FCL
package developed by UNC Gamma Lab.

In addition, DART supports various joint types (ball-and-socket,
universal, hinge, and prismatic joints) and arbitrary meshes. DART
also provides two explicit integration methods: first-order
Runge-Kutta and fourth-order Runge Kutta.

*/

#ifndef DART_KINEMATICS_BODYNODE_H
#define DART_KINEMATICS_BODYNODE_H

#include <vector>

#include <Eigen/Dense>

#include "common/Deprecated.h"
#include "math/LieGroup.h"

namespace dart {
namespace renderer {
class RenderInterface;
}
}

namespace dart {
namespace dynamics {

class GenCoord;
class Skeleton;
class Joint;
class Shape;

/// @brief BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
/// Mostly automatically constructed by FileInfoSkel.
/// @see FileInfoSkel.
///
/// [Members]
/// W: world transformation (4x4 matrix)
/// J: world Jacobian (6xn matrix)
/// dJ: world Jacobian derivative (6xn matrix)
/// V: generalized body velocity (6x1 vector)
/// dV: generalized body acceleration (6x1 vector)
/// F: generalized body force (6x1 vector)
/// I: generalized body inertia (6x6 matrix)
class BodyNode
{
public:
    //--------------------------------------------------------------------------
    // Constructor and Desctructor
    //--------------------------------------------------------------------------
    /// @brief
    BodyNode(const std::string& _name = "");

    /// @brief
    virtual ~BodyNode();

    //--------------------------------------------------------------------------
    // Static properties
    //--------------------------------------------------------------------------
    /// @brief
    void setName(const std::string& _name);

    /// @brief
    const std::string& getName() const;

    /// @brief
    void setGravityMode(bool _onoff) { mGravityMode = _onoff; }

    /// @brief
    bool getGravityMode() const { return mGravityMode; }

    /// @brief
    bool getCollideState() const { return mCollidable; }

    /// @brief
    void setCollideState(bool _c) { mCollidable = _c; }

    /// @brief
    void setMass(double _mass);

    /// @brief
    double getMass() const;

    /// @brief
    DEPRECATED void setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                                    double _Ixy, double _Ixz, double _Iyz);
    void setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                            double _Ixy, double _Ixz, double _Iyz);

    /// @brief
    void setInertia(const math::Inertia& _I) { mI = _I; }

    /// @brief
    const math::Inertia& getInertia() const { return mI; }

    //void setLocalInertia(const Eigen::Matrix3d& _inertia) { mI = _inertia; }
    //Eigen::Matrix3d getLocalInertia() const { return mI; }
    //Eigen::Matrix3d getWorldInertia() const { return mIc; }
    //DEPRECATED Eigen::Matrix3d getInertia() const { return mIc; } ///< Superseded by getWorldInertia()
    /// @brief Computes the "mass tensor" in lagrangian dynamics from the
    /// inertia matrix
    //Eigen::Matrix4d getMassTensor();

    /// @brief
    DEPRECATED void setLocalCOM(const Eigen::Vector3d& _off)
    { setCenterOfMass(math::Vec3(_off[0], _off[1], _off[2])); }
    void setCenterOfMass(const math::Vec3& _com);


    /// @brief
    DEPRECATED Eigen::Vector3d getLocalCOM() const { return getCenterOfMass(); }
    Eigen::Vector3d getCenterOfMass() const;

    //DEPRECATED Eigen::Vector3d getWorldCOM() { return evalWorldPos(mCOMLocal); }

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setSkelIndex(int _idx) { mSkelIndex = _idx; }

    /// @brief
    int getSkelIndex() const { return mSkelIndex; }

    /// @brief
    void setVisualizationShape(Shape* _shape) { mVizShape = _shape; }

    /// @brief
    Shape* getVisualizationShape() const { return mVizShape; }

    /// @brief
    void setCollisionShape(Shape* _shape) { mColShape = _shape; }

    /// @brief
    Shape* getCollisionShape() const { return mColShape; }

    /// @brief
    DEPRECATED void setSkel(Skeleton* _skel) { mSkeleton = _skel; }
    void setSkeleton(Skeleton* _skel) { mSkeleton = _skel; }

    /// @brief
    DEPRECATED Skeleton* getSkel() const { return mSkeleton; }
    Skeleton* getSkeleton() const { return mSkeleton; }

    /// @brief
    void setParentJoint(Joint* _joint) { mParentJoint = _joint; }

    /// @brief
    Joint* getParentJoint() const { return mParentJoint; }

    /// @brief
    void addChildJoint(Joint* _joint);

    /// @brief
    Joint* getChildJoint(int _idx) const;

    /// @brief
    const std::vector<Joint*>& getChildJoints() const { return mJointsChild; }

    /// @brief
    int getNumChildJoints() const { return mJointsChild.size(); }

    /// @brief
    void setParentBody(BodyNode* _body) { mParentBody = _body; }

    /// @brief
    BodyNode* getParentBody() const { return mParentBody; }

    /// @brief
    void addChildBody(BodyNode* _body);

    /// @brief
    DEPRECATED BodyNode* getChildNode(int _idx) const;
    BodyNode* getChildBody(int _idx) const;

    /// @brief
    const std::vector<BodyNode*>& getChildBodies() const { return mChildBodies; }

    // TODO: Check
    void setDependDofList();

    // TODO: Check
    int getNumLocalDofs() const;

    // TODO: Check
    GenCoord* getLocalDof(int _idx) const;

    // TODO: Check
    /// @brief The number of the dofs by which this node is affected.
    int getNumDependentDofs() const { return mDependentDofs.size(); }

    // TODO: Check
    /// @brief Return an dof index from the array index (< getNumDependentDofs).
    int getDependentDof(int _arrayIndex) const { return mDependentDofs[_arrayIndex]; }

    // TODO: Check
    /// @brief
    Eigen::VectorXd getDependDofs() const;

    //--------------------------------------------------------------------------
    // Properties updated by dynamics (kinematics)
    //--------------------------------------------------------------------------

    /// @brief
    /// This function should be called only in modeling process. The
    /// transformation of this link will be updating by dynamics algorithms
    /// automatically.
    void setWorldTransformation(const math::SE3& _W) { mW = _W; }

    /// @brief Transformation from the local coordinates of this body node to
    /// the world coordinates.
    DEPRECATED const math::SE3& getWorldTransform() const { return mW; }
    const math::SE3& getTransformationWorld() const { return mW; }

    /// @brief Transformation from the world coordinates to the local
    /// coordinates of this body node.
//    DEPRECATED Eigen::Matrix4d getWorldInvTransform() const
//    { return mW.getInverse().getMatrix(); }
    math::SE3 getWorldInvTransformation() const { return math::Inv(mW); }

    /// @brief Given a 3D vector lp in the local coordinates of this body node.
    /// @return The world coordinates of this vector
    Eigen::Vector3d evalWorldPos(const Eigen::Vector3d& _lp) const;

    /// @brief
    const math::se3& getVelocityBody() const { return mV; }

    /// @brief
    math::se3 getVelocityWorld() const;

    /// @brief
    math::se3 getVelocityWorldAtCOG() const;

    /// @breif
    math::se3 getVelocityWorldAtPoint(const math::Vec3& _pointBody) const;

    /// @breif
    math::se3 getVelocityWorldAtFrame(const math::SE3& _T) const;

    /// @brief
    const math::se3& getAcceleration() const { return mdV; }

    /// @brief
    math::se3 getAccelerationWorld() const;

    /// @brief
    math::se3 getAccelerationWorldAtCOG() const;

    /// @breif
    math::se3 getAccelerationWorldAtPoint(const math::Vec3& _pointBody) const;

    /// @breif
    math::se3 getAccelerationWorldAtFrame(const math::SE3& _T) const;

    /// @brief
    const math::Jacobian& getJacobianBody() const { return mBodyJacobian; }

    /// @brief
    math::Jacobian getJacobianWorld() const;

    /// @brief Get body Jacobian at contact point.
    math::Jacobian getJacobianWorldAtPoint(const math::Vec3& r_world) const;

    // TODO: Speed up here.
    // TODO: Eigne?
    /// @brief
    Eigen::MatrixXd getJacobianWorldAtPoint_LinearPartOnly(
            const math::Vec3& r_world) const;

    /// @brief
    void setColliding(bool _colliding) { mColliding = _colliding; }

    /// @brief
    bool getColliding() { return mColliding; }

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
    /// Called from @Skeleton::clearExternalForces.
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

    /// @brief
    double getKineticEnergy() const { return 0.5 * mV.dot(mI * mV); }

    // TODO: Not implemented.
    /// @brief
    double getPotentialEnergy() const {}


    //--------------------------------------------------------------------------
    // Rendering
    //--------------------------------------------------------------------------
    /// @brief Render the entire subtree rooted at this body node.
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true, int _depth = 0) const;

    //--------------------------------------------------------------------------
    // Sub-functions for Recursive Kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief Initialize the vector memebers with proper sizes.
    void init();

    /// @brief
    /// parentJoint.T, parentBody.W --> W
    void updateTransformation();

    /// @brief
    /// parentJoint.V, parentBody.V --> V
    /// parentJoint.S --> J
    void updateVelocity(bool _updateJacobian = true);

    /// @brief
    void updateEta();

    /// @brief
    /// parentJoint.V, parentJoint.dV, parentBody.dV, V --> dV
    /// parentJoint.dS --> dJ
    void updateAcceleration(bool _updateJacobianDeriv = false);

    /// @brief
    /// childBodies.F, V, dV --> F, Fgravity
    void updateBodyForce(const Eigen::Vector3d& _gravity,
                         bool _withExternalForces = false);

    /// @brief
    /// parentJoint.S, F --> tau
    void updateGeneralizedForce(bool _withDampingForces = false);

    /// @brief
    void updateArticulatedInertia();

    /// @brief
    void updatePsi();

    /// @brief
    void updatePi();

    /// @brief
    void updateBeta();

    /// @brief
    void updateBiasForce(const Eigen::Vector3d& _gravity);

    /// @brief
    void update_ddq();

    /// @brief
    void update_F_fs();

    /// @brief
    void updateDampingForce();

    /// @brief
    void updateMassMatrix();

    /// @brief
    void aggregateMass(Eigen::MatrixXd& M);

protected:
    //--------------------------------------------------------------------------
    // General properties
    //--------------------------------------------------------------------------
    /// @brief A unique ID of this node globally.
    int mID;

    /// @brief Counts the number of nodes globally.
    static int msBodyNodeCount;

    /// @brief
    std::string mName;

    /// @brief Index in the model
    int mSkelIndex;

    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// TODO: Not implemented yet!
    bool mGravityMode;

    /// @brief Generalized inertia.
    math::Inertia mI;

    /// @brief Generalized inertia at center of mass.
    Eigen::Vector3d mCenterOfMass;
    double mIxx;
    double mIyy;
    double mIzz;
    double mIxy;
    double mIxz;
    double mIyz;
    double mMass;

    /// @brief
    //std::vector<Shape*> mVizShapes;
    Shape* mVizShape;

    /// @brief
    //std::vector<Shape*> mColShapes;
    Shape* mColShape;

    /// @brief Indicating whether this node is collidable.
    bool mCollidable;

    /// @brief Whether the node is currently in collision with another node.
    bool mColliding;

    // TODO: This will be used soon.
    /// @brief Restitution coefficient
    double mRestitutionCoeff;

    // TODO: This will be used soon.
    /// @brief Friction coefficient.
    double mFrictionCoeff;

    //--------------------------------------------------------------------------
    // Structual Properties
    //--------------------------------------------------------------------------
    /// @brief Pointer to the model this body node belongs to.
    Skeleton* mSkeleton;

    /// @brief
    // TODO: rename
    //Joint* mParentJoint;
    Joint* mParentJoint;

    /// @brief
    // TODO: rename
    //std::vector<Joint*> mChildJoints;
    std::vector<Joint*> mJointsChild;

    /// @brief
    BodyNode* mParentBody;

    /// @brief
    std::vector<BodyNode*> mChildBodies;

    /// @brief A list of dependent dof indices
    std::vector<int> mDependentDofs;

    //--------------------------------------------------------------------------
    // Dynamical Properties
    //--------------------------------------------------------------------------
    /// @brief World transformation.
    math::SE3 mW;

    /// @brief
    math::Jacobian mBodyJacobian;

    /// @brief
    math::Jacobian mBodyJacobianDeriv;

    /// @brief Generalized body velocity w.r.t. body frame.
    math::se3 mV;

    /// @brief
    math::se3 mEta;

    /// @brief Generalized body acceleration w.r.t. body frame.
    math::se3 mdV;

    /// @brief Generalized body force w.r.t. body frame.
    math::dse3 mF;

    /// @brief
    math::dse3 mFext;

    /// @brief
    math::dse3 mFgravity;

    struct BodyState
    {
        math::SE3 W;
        math::se3 V;
        math::se3 dV;
        math::dse3 Fjoint;
        math::dse3 Fgravity;
        math::dse3 Fext;
    };

    struct ForwardDynamicsCacheData
    {
        math::Inertia mAI;      ///< Articulated inertia
        math::dse3 mB;          ///< Bias force
        Eigen::MatrixXd mAI_S;
        Eigen::MatrixXd mPsi;
        math::Inertia mPi;
        math::dse3 mBeta;
    };
    //ForwardDynamicsCacheData mFwdDynData;
    math::Inertia mAI;      ///< Articulated inertia
    math::dse3 mB;          ///< Bias force
    Eigen::MatrixXd mAI_S;
    Eigen::MatrixXd mPsi;
    math::Inertia mPi;
    Eigen::VectorXd mAlpha;
    math::dse3 mBeta;

    /// @brief
    Eigen::MatrixXd mM;

private:
    void _updateGeralizedInertia();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_BODYNODE_H
