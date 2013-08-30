/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include "dynamics/BodyNode.h"

#include <iostream>

#include "common/UtilsCode.h"
#include "math/Helpers.h"
#include "renderer/RenderInterface.h"
#include "dynamics/Joint.h"
#include "dynamics/Shape.h"
#include "dynamics/Skeleton.h"

using namespace std;
using namespace Eigen;

namespace dart {
namespace dynamics {

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode(const std::string& _name)
    : mSkelIndex(-1),
      mName(_name),
      mCollidable(true),
      mColliding(false),
      mSkeleton(NULL),
      mParentJoint(NULL),
      mJointsChild(std::vector<Joint*>(0)),
      mParentBodyNode(NULL),
      mChildBodyNodes(std::vector<BodyNode*>(0)),
      mGravityMode(true),
      mCenterOfMass(Eigen::Vector3d::Zero()),
      mI(Eigen::Matrix6d::Identity()),
      mW(Eigen::Isometry3d::Identity()),
      mV(Eigen::Vector6d::Zero()),
      mEta(Eigen::Vector6d::Zero()),
      mdV(Eigen::Vector6d::Zero()),
      mF(Eigen::Vector6d::Zero()),
      mFext(Eigen::Vector6d::Zero()),
      mFgravity(Eigen::Vector6d::Zero()),
      mB(Eigen::Vector6d::Zero()),
      mBeta(Eigen::Vector6d::Zero()),
      mRestitutionCoeff(0.5),
      mFrictionCoeff(0.4),
      mID(BodyNode::msBodyNodeCount++)
{
}

BodyNode::~BodyNode()
{
    for(int i = 0; i < mVizShapes.size(); i++)
        delete mVizShapes[i];
    for(int i = 0; i < mColShapes.size(); i++)
        if(mColShapes[i] != mVizShapes[i])
            delete mColShapes[i];
}

void BodyNode::setName(const std::string& _name)
{
    mName = _name;
}

const std::string& BodyNode::getName() const
{
    return mName;
}

void BodyNode::setMass(double _mass)
{
    assert(_mass >= 0.0 && "Negative mass is not allowable.");
    mMass = _mass;
    _updateGeralizedInertia();
}

double BodyNode::getMass() const
{
    return mMass;
}

void BodyNode::addChildJoint(Joint* _joint)
{
    assert(_joint != NULL);

    mJointsChild.push_back(_joint);
}

Joint*BodyNode::getChildJoint(int _idx) const
{
    assert(0 <= _idx && _idx < mJointsChild.size());

    return mJointsChild[_idx];
}

void BodyNode::addChildBody(BodyNode* _body)
{
    assert(_body != NULL);

    mChildBodyNodes.push_back(_body);
}

BodyNode* BodyNode::getChildBodyNode(int _idx) const
{
    assert(0 <= _idx && _idx < mChildBodyNodes.size());

    return mChildBodyNodes[_idx];
}

void BodyNode::setDependDofList()
{
    mDependentDofs.clear();
    if (mParentBodyNode != NULL)
    {
        mDependentDofs.insert(mDependentDofs.end(),
                              mParentBodyNode->mDependentDofs.begin(),
                              mParentBodyNode->mDependentDofs.end());
    }

    for (int i = 0; i < getNumLocalDofs(); i++)
    {
        int dofID = getLocalDof(i)->getSkelIndex();
        mDependentDofs.push_back(dofID);
    }

#if _DEBUG
    for (int i = 0; i < (int)mDependentDofs.size() - 1; i++)
    {
        int now = mDependentDofs[i];
        int next = mDependentDofs[i + 1];
        if (now > next)
        {
            cerr << "Array not sorted!!!" << endl;
            exit(0);
        }
    }
#endif
}

int BodyNode::getNumLocalDofs() const
{
    return mParentJoint->getDOF();
}

GenCoord* BodyNode::getLocalDof(int _idx) const
{
    return mParentJoint->getDof(_idx);
}

void BodyNode::setWorldTransform(const Eigen::Isometry3d &_W)
{
    assert(math::VerifySE3(_W));

    mW = _W;
}

Eigen::Vector3d BodyNode::evalWorldPos(const Eigen::Vector3d& _lp) const
{
    return math::xformHom(mW, _lp);
}

Eigen::Vector6d BodyNode::getVelocityWorld() const
{
    //return math::Rotate(mW, mV);
    return math::AdR(mW, mV);
}

Eigen::Vector6d BodyNode::getVelocityWorldAtCOG() const
{
    Eigen::Isometry3d worldFrameAtCOG = mW;
    worldFrameAtCOG.translation() = math::Rotate(mW, -mCenterOfMass);
    return math::AdT(worldFrameAtCOG, mV);
}

Eigen::Vector6d BodyNode::getVelocityWorldAtPoint(const Eigen::Vector3d& _pointBody) const
{
    Eigen::Isometry3d worldFrameAtPoint = mW;
    worldFrameAtPoint.translation() = math::Rotate(mW, -_pointBody);
    return math::AdT(worldFrameAtPoint, mV);
}

Eigen::Vector6d BodyNode::getVelocityWorldAtFrame(const Eigen::Isometry3d& _T) const
{
    assert(math::VerifySE3(_T));

    return math::AdT(math::Inv(_T) * mW, mV);
}

Eigen::Vector6d BodyNode::getAccelerationWorld() const
{
    //return math::Rotate(mW, mdV);
    return math::AdR(mW, mdV);
}

Eigen::Vector6d BodyNode::getAccelerationWorldAtCOG() const
{
    Eigen::Isometry3d worldFrameAtCOG = mW;
    worldFrameAtCOG.translation() = math::Rotate(mW, -mCenterOfMass);
    return math::AdT(worldFrameAtCOG, mdV);
}

Eigen::Vector6d BodyNode::getAccelerationWorldAtPoint(const Eigen::Vector3d& _pointBody) const
{
    Eigen::Isometry3d worldFrameAtPoint = mW;
    worldFrameAtPoint.translation() = math::Rotate(mW, -_pointBody);
    return math::AdT(worldFrameAtPoint, mdV);
}

Eigen::Vector6d BodyNode::getAccelerationWorldAtFrame(const Eigen::Isometry3d& _T) const
{
    assert(math::VerifySE3(_T));

    return math::AdT(math::Inv(_T) * mW, mdV);
}

math::Jacobian BodyNode::getJacobianWorld() const
{
    return math::AdR(mW, mBodyJacobian);
}

math::Jacobian BodyNode::getJacobianWorldAtPoint(
        const Eigen::Vector3d& r_world) const
{
    //--------------------------------------------------------------------------
    // Jb                : body jacobian
    //
    // X = | I r_world | : frame whose origin is at contact point
    //     | 0       1 |
    //
    // X^{-1} = | I -r_world |
    //          | 0        1 |
    //
    // W = | R p |       : body frame
    //     | 0 1 |
    //
    // body_jacobian_at_contact_point = Ad(X^{-1} * W, Jb)
    //--------------------------------------------------------------------------
    return math::AdTJac(math::ExpLinear(-r_world) * mW, mBodyJacobian);
}

Eigen::MatrixXd BodyNode::getJacobianWorldAtPoint_LinearPartOnly(
        const Eigen::Vector3d& r_world) const
{
    //--------------------------------------------------------------------------
    // Jb                : body jacobian
    //
    // X = | I r_world | : frame whose origin is at contact point
    //     | 0       1 |
    //
    // X^{-1} = | I -r_world |
    //          | 0        1 |
    //
    // W = | R p |       : body frame
    //     | 0 1 |
    //
    // body_jacobian_at_contact_point = Ad(X^{-1} * W, Jb)
    //--------------------------------------------------------------------------

    // TODO: Speed up here.
    Eigen::MatrixXd JcLinear = Eigen::MatrixXd::Zero(3, getNumDependentDofs());

    JcLinear = getJacobianWorldAtPoint(r_world).bottomLeftCorner(3,getNumDependentDofs());

    return JcLinear;
}

void BodyNode::init()
{
    assert(mSkeleton != NULL);

    const int numDepDofs = getNumDependentDofs();

    mBodyJacobian = math::Jacobian::Zero(6,numDepDofs);
    mBodyJacobianDeriv = math::Jacobian::Zero(6,numDepDofs);
    mM = Eigen::MatrixXd::Zero(numDepDofs, numDepDofs);
}

void BodyNode::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor,
                    int _depth) const
{
    if (_ri == NULL)
        return;

    _ri->pushMatrix();

    // render the self geometry
    mParentJoint->applyGLTransform(_ri);

    _ri->pushName((unsigned)mID);
    for(int i = 0; i < mVizShapes.size(); i++)
    {
        _ri->pushMatrix();
        mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
        _ri->popMatrix();
    }
    _ri->popName();

    // render the subtree
    for (unsigned int i = 0; i < mJointsChild.size(); i++)
    {
        mJointsChild[i]->getChildNode()->draw(_ri, _color, _useDefaultColor);
    }

    _ri->popMatrix();
}

void BodyNode::updateTransformation()
{
    if (mParentBodyNode)
    {
        mW = mParentBodyNode->getWorldTransform()
             * mParentJoint->getLocalTransformation();
    }
    else
    {
        mW = mParentJoint->getLocalTransformation();
    }

    assert(math::VerifySE3(mW));
}

void BodyNode::updateVelocity(bool _updateJacobian)
{
    //--------------------------------------------------------------------------
    // Body velocity update
    //
    // V(i) = Ad(T(i, i-1), V(i-1)) + S * dq
    //--------------------------------------------------------------------------

    if (mParentBodyNode)
    {
        mV = math::AdInvT(mParentJoint->getLocalTransformation(),
                          mParentBodyNode->getVelocityBody()) +
                mParentJoint->getLocalVelocity();
    }
    else
    {
        mV = mParentJoint->getLocalVelocity();
    }

    assert(math::Verifyse3(mV));

    if (_updateJacobian == false)
        return;

    //--------------------------------------------------------------------------
    // Jacobian update
    //
    // J = | J1 J2 ... Jn |
    //   = | Ad(T(i,i-1), J_parent) J_local |
    //
    //   J_parent: (6 x parentDOF)
    //    J_local: (6 x localDOF)
    //         Ji: (6 x 1) se3
    //          n: number of dependent coordinates
    //--------------------------------------------------------------------------

    const int numLocalDOFs = getNumLocalDofs();
    const int numParentDOFs = getNumDependentDofs()-numLocalDOFs;

    // Parent Jacobian
    if (mParentBodyNode != NULL)
    {
        assert(mParentBodyNode->mBodyJacobian.cols() + getNumLocalDofs()
               == mBodyJacobian.cols());

        for (int i = 0; i < numParentDOFs; ++i)
        {
            assert(mParentJoint);
            mBodyJacobian.col(i) = math::AdInvT(
                                       mParentJoint->getLocalTransformation(),
                                       mParentBodyNode->mBodyJacobian.col(i));
        }
    }

    // Local Jacobian
    for(int i = 0; i < numLocalDOFs; i++)
    {
        mBodyJacobian.col(numParentDOFs + i) =
                mParentJoint->getLocalJacobian().col(i);
    }
}

void BodyNode::updateEta()
{
    if (mParentJoint->getDOF() > 0)
    {
        mEta = math::ad(mV, mParentJoint->mS*mParentJoint->get_dq()) +
           mParentJoint->mdS*mParentJoint->get_dq();

        assert(math::Verifyse3(mEta));
    }
}

void BodyNode::updateAcceleration(bool _updateJacobianDeriv)
{
    // dV(i) = Ad(T(i, i-1), dV(i-1))
    //         + ad(V(i), S * dq) + dS * dq
    //         + S * ddq
    //       = Ad(T(i, i-1), dV(i-1))
    //         + eta
    //         + S * ddq

    if (mParentJoint->getDOF() > 0)
    {
        if (mParentBodyNode)
        {
            mdV = math::AdInvT(mParentJoint->getLocalTransformation(),
                               mParentBodyNode->getAcceleration()) +
                  mEta + mParentJoint->mS*mParentJoint->get_ddq();
        }
        else
        {
            mdV = mEta + mParentJoint->mS*mParentJoint->get_ddq();
        }
    }

    assert(math::Verifyse3(mdV));

    if (_updateJacobianDeriv == false)
        return;

    //--------------------------------------------------------------------------
    // Jacobian first derivative update
    //
    // dJ = | dJ1 dJ2 ... dJn |
    //   = | Ad(T(i,i-1), dJ_parent) dJ_local |
    //
    //   dJ_parent: (6 x parentDOF)
    //    dJ_local: (6 x localDOF)
    //         dJi: (6 x 1) se3
    //          n: number of dependent coordinates
    //--------------------------------------------------------------------------

    const int numLocalDOFs = getNumLocalDofs();
    const int numParentDOFs = getNumDependentDofs() - numLocalDOFs;

    // Parent Jacobian
    if (mParentBodyNode != NULL)
    {
        assert(mParentBodyNode->mBodyJacobianDeriv.cols() + getNumLocalDofs()
               == mBodyJacobianDeriv.cols());

        for (int i = 0; i < numParentDOFs; ++i)
        {
            assert(mParentJoint);
            Eigen::Vector6d dJi = math::AdInvT(mParentJoint->getLocalTransformation(),
                                         mParentBodyNode->mBodyJacobianDeriv.col(i));
            mBodyJacobianDeriv.col(i) = dJi;
        }
    }

    // Local Jacobian
    for(int i = 0; i < numLocalDOFs; i++)
    {
        mBodyJacobianDeriv.col(numParentDOFs + i) =
                mParentJoint->getLocalJacobianFirstDerivative().col(i);
    }
}

void BodyNode::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                                  double _Ixy, double _Ixz, double _Iyz)
{
    assert(_Ixx >= 0.0);
    assert(_Iyy >= 0.0);
    assert(_Izz >= 0.0);

    assert(_Ixy >= 0.0);
    assert(_Ixz >= 0.0);
    assert(_Iyz >= 0.0);

    mIxx = _Ixx;
    mIyy = _Iyy;
    mIzz = _Izz;

    mIxy = _Ixy;
    mIxz = _Ixz;
    mIyz = _Iyz;

    _updateGeralizedInertia();
}

void BodyNode::setLocalCOM(const Eigen::Vector3d& _com)
{
    mCenterOfMass = _com;

    _updateGeralizedInertia();
}

const Eigen::Vector3d& BodyNode::getLocalCOM() const
{
    return mCenterOfMass;
}

Eigen::Vector3d BodyNode::getWorldCOM() const
{
    return evalWorldPos(mCenterOfMass);
}

Matrix6d BodyNode::getGeneralizedInertia() const
{
    return mI;
}

//void BodyNode::setExternalForceLocal(const math::dse3& _FextLocal)
//{
//    mFext = _FextLocal;
//}
//
//void BodyNode::setExternalForceGlobal(const math::dse3& _FextWorld)
//{
//    mFext = _FextWorld;
//}
//
//void BodyNode::setExternalForceLocal(
//        const Eigen::Vector3d& _posLocal,
//        const Eigen::Vector3d& _linearForceGlobal)
//{
//}

void BodyNode::addExtForce(const Eigen::Vector3d& _offset,
                           const Eigen::Vector3d& _force,
                           bool _isOffsetLocal, bool _isForceLocal)
{
    Vector3d pos = _offset;
    Vector3d force = _force;

//    if (!_isOffsetLocal)
//        pos = math::xformHom(getWorldInvTransform(), _offset);

//    if (!_isForceLocal)
//        force = mW.rotation().transpose()*_force;

//    mContacts.push_back(pair<Vector3d, Vector3d>(pos, force));
}

void BodyNode::addExternalForceLocal(const Eigen::Vector6d& _FextLocal)
{
    mFext += _FextLocal;
}

void BodyNode::addExternalForceGlobal(const Eigen::Vector6d& _FextWorld)
{
//    mFext += math::dAdT(mW, _FextWorld);
}

void BodyNode::addExternalForceLocal(
        const Eigen::Vector3d& _offset,
        const Eigen::Vector3d& _linearForce,
        bool _isOffsetLocal,
        bool _isLinearForceLocal)
{
    Vector3d offset = _offset;
    Vector3d linearForce = _linearForce;

//    if (!_isOffsetLocal)
//        offset = math::xformHom(getWorldInvTransform(), _offset);

//    if (!_isLinearForceLocal)
//        linearForce = mW.rotation().transpose()*_linearForce;

//    math::dse3 contactForce;
//    contactForce.head<3>() = offset.cross(linearForce);
//    contactForce.tail<3>() = linearForce;

//    if (linearForce != Eigen::Vector3d::Zero())
//    {
//        int a = 10;
//    }

//    mFext += contactForce;
}

const Eigen::Vector6d& BodyNode::getExternalForceLocal() const
{
    return mFext;
}

Eigen::Vector6d BodyNode::getExternalForceGlobal() const
{
    //return math::dAdInvT(mW, mFext);
    return Eigen::Vector6d();
}

void BodyNode::updateBodyForce(const Eigen::Vector3d& _gravity,
                               bool _withExternalForces)
{
    if (mGravityMode == true)
        mFgravity = mI * math::AdInvRLinear(mW, _gravity);
    else
        mFgravity.setZero();

    mF = mI * mdV;                // Inertial force
    if (_withExternalForces)
        mF -= mFext;              // External force
    mF -= mFgravity;              // Gravity force
    mF -= math::dad(mV, mI * mV); // Coriolis force

    for (std::vector<BodyNode*>::iterator iChildBody = mChildBodyNodes.begin();
         iChildBody != mChildBodyNodes.end();
         ++iChildBody)
    {
        dynamics::Joint* childJoint = (*iChildBody)->getParentJoint();
        assert(childJoint != NULL);
        BodyNode* bodyDyn = dynamic_cast<BodyNode*>(*iChildBody);
        assert(bodyDyn != NULL);

        mF += math::dAdInvT(childJoint->getLocalTransformation(),
                            bodyDyn->getBodyForce());
    }

    assert(math::Verifyse3(mF));
}

void BodyNode::updateGeneralizedForce(bool _withDampingForces)
{
    assert(mParentJoint != NULL);

    const math::Jacobian& J = mParentJoint->getLocalJacobian();

//    if (_withDampingForces)
//        mF -= mFDamp;

    mParentJoint->set_tau(J.transpose()*mF);
}

void BodyNode::updateArticulatedInertia()
{
    mAI = mI;

    for (std::vector<Joint*>::iterator itrJoint = mJointsChild.begin();
         itrJoint != mJointsChild.end();
         ++itrJoint)
    {
        mAI += math::Transform(
                   math::Inv((*itrJoint)->getLocalTransformation()),
                   (*itrJoint)->getChildBody()->mPi);
    }
}

void BodyNode::updateBiasForce(const Eigen::Vector3d& _gravity)
{
    if (mGravityMode == true)
        mFgravity = mI * math::AdInvRLinear(mW, Eigen::Vector3d(_gravity));
    else
        mFgravity.setZero();

    mB = -math::dad(mV, mI*mV) - mFext - mFgravity;

    for (std::vector<Joint*>::iterator itrJoint = mJointsChild.begin();
         itrJoint != mJointsChild.end();
         ++itrJoint)
    {
        mB += math::dAdInvT((*itrJoint)->getLocalTransformation(),
                            (*itrJoint)->getChildBody()->mBeta);
    }

    assert(math::Verifyse3(mB));
}

void BodyNode::updatePsi()
{
    assert(mParentJoint != NULL);

    //int n = mParentJoint->getDOF();
    //mAI_S = Eigen::MatrixXd::Zero(6, n);
    //mPsi = Eigen::MatrixXd::Zero(n, n);

    mAI_S.noalias() = mAI*mParentJoint->mS;
    mPsi = (mParentJoint->mS.transpose()*mAI_S).inverse();
}

void BodyNode::updatePi()
{
    mPi            = mAI;
    mPi.noalias() -= mAI_S*mPsi*mAI_S.transpose();
}

void BodyNode::updateBeta()
{
    mAlpha           = mParentJoint->get_tau();

    // TODO: Need to find more efficient way in architecture
    // Add constraint force
    if (mParentJoint->getDOF() > 0)
    {
        mAlpha          += mParentJoint->getDampingForce();
        Eigen::VectorXd Fc = Eigen::VectorXd::Zero(mParentJoint->getDOF());
        for (int i = 0; i < mParentJoint->getDOF(); i++)
            Fc(i) = mSkeleton->getConstraintForces()[(mParentJoint->getGenCoords()[i])->getSkelIndex()];
        mAlpha          += Fc;
    }

    mAlpha          -= mParentJoint->mS.transpose()*(mAI*mEta + mB);
    mBeta            = mB;
    if (mParentJoint->getDOF() > 0)
        mBeta += mAI*(mEta + mParentJoint->mS*mPsi*(mAlpha));
    else
        mBeta += mAI*mEta;

    assert(math::Verifyse3(mBeta));
}

void BodyNode::update_ddq()
{
    Eigen::VectorXd ddq;
    if (mParentBodyNode)
    {
        ddq.noalias() = mPsi*
                        (mAlpha -
                         mParentJoint->mS.transpose()*mAI*
                         math::AdInvT(mParentJoint->getLocalTransformation(),
                                      mParentBodyNode->getAcceleration())
                         );
    }
    else
    {
        ddq.noalias() = mPsi*mAlpha;
    }

    mParentJoint->set_ddq(ddq);
}

void BodyNode::update_F_fs()
{
    mF.noalias() = mAI*mdV;
    mF          += mB;

    assert(math::Verifyse3(mF));
}

void BodyNode::updateDampingForce()
{
    dterr << "Not implemented.\n";
}

void BodyNode::updateMassMatrix()
{
    mM.triangularView<Eigen::Upper>() = mBodyJacobian.transpose() *
                                        mI *
                                        mBodyJacobian;
    mM.triangularView<Eigen::StrictlyLower>() = mM.transpose();
}

void BodyNode::evalExternalForcesRecursive(Eigen::VectorXd& _extForce)
{
//    assert(mParentJoint != NULL);

//    const math::Jacobian& J = mParentJoint->getLocalJacobian();

//    Eigen::VectorXd localForce = J.transpose()*mFext;

//    for(int i = 0; i < getNumDependentDofs(); i++)
//        _extForce(mDependentDofs[i]) += mFext(i);
}

void BodyNode::aggregateMass(Eigen::MatrixXd& _M)
{
    for(int i=0; i<getNumDependentDofs(); i++)
    {
        for(int j=0; j<getNumDependentDofs(); j++)
        {
            _M(mDependentDofs[i], mDependentDofs[j]) += mM(i, j);
        }
    }
}

void BodyNode::_updateGeralizedInertia()
{
    // G = | I - m * [r] * [r]   m * [r] |
    //     |          -m * [r]     m * 1 |

    // m * r
    double mr0 = mMass * mCenterOfMass[0];
    double mr1 = mMass * mCenterOfMass[1];
    double mr2 = mMass * mCenterOfMass[2];

    // m * [r] * [r]
    double mr0r0 = mr0 * mCenterOfMass[0];
    double mr1r1 = mr1 * mCenterOfMass[1];
    double mr2r2 = mr2 * mCenterOfMass[2];
    double mr0r1 = mr0 * mCenterOfMass[1];
    double mr1r2 = mr1 * mCenterOfMass[2];
    double mr2r0 = mr2 * mCenterOfMass[0];

    mI(0,0) =  mIxx + mr1r1 + mr2r2;   mI(0,1) =  mIxy - mr0r1;           mI(0,2) =  mIxz - mr2r0;           assert(mI(0,3) == 0.0);   mI(0,4) = -mr2;           mI(0,5) =  mr1;
                                       mI(1,1) =  mIyy + mr2r2 + mr0r0;   mI(1,2) =  mIyz - mr1r2;           mI(1,3) =  mr2;           assert(mI(1,4) == 0.0);   mI(1,5) = -mr0;
                                                                          mI(2,2) =  mIzz + mr0r0 + mr1r1;   mI(2,3) = -mr1;           mI(2,4) =  mr0;           assert(mI(2,5) == 0.0);
                                                                                                             mI(3,3) =  mMass;         assert(mI(3,4) == 0.0);   assert(mI(3,5) == 0.0);
                                                                                                                                       mI(4,4) =  mMass;         assert(mI(4,5) == 0.0);
                                                                                                                                                                 mI(5,5) =  mMass;

    mI.triangularView<Eigen::StrictlyLower>() = mI.transpose();
}

void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNode::clearExternalForces()
{
    mContacts.clear();
    mFext.setZero();
    //mExtForceBody.setZero();
    //mExtTorqueBody.setZero();
}

} // namespace dynamics
} // namespace dart

