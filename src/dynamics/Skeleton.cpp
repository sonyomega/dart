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

#include "math/UtilsMath.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Dof.h"
#include "dynamics/Joint.h"
#include "dynamics/Skeleton.h"
#include "dynamics/Skeleton.h"

using namespace std;
using namespace Eigen;

namespace dart {
namespace dynamics {

Skeleton::Skeleton(const std::string& _name)
    : System(),
      mName(_name),
      mTotalMass(0.0),
      mGraph(NULL)
{

}

Skeleton::~Skeleton()
{
    if (mGraph != NULL)
        delete mGraph;
}

void Skeleton::addBody(BodyNode* _body, bool _addParentJoint)
{
    assert(_body != NULL);

    mBodies.push_back(_body);
    _body->setSkelIndex(mBodies.size() - 1);

    // The parent joint possibly be null
    if (_addParentJoint)
        addJoint(_body->getParentJoint());
}

void Skeleton::addJoint(Joint* _joint)
{
    assert(_joint != NULL);

    mJoints.push_back(_joint);
}

BodyNode*Skeleton::findBody(const string& _name) const
{
    assert(!_name.empty());

    for (std::vector<BodyNode*>::const_iterator itrBody = mBodies.begin();
         itrBody != mBodies.end();
         ++itrBody)
    {
        if ((*itrBody)->getName() == _name)
            return *itrBody;
    }

    return NULL;
}

Eigen::VectorXd Skeleton::getDependentConfiguration(BodyNode* _beginBody,
                                                    BodyNode* _endBody) const
{
    assert(_beginBody != NULL);
    assert(_endBody != NULL);

    int beginBodyID = _beginBody->getSkelIndex();
    int endBodyID = _endBody->getSkelIndex();


}

void Skeleton::setPose(const Eigen::VectorXd& _pose,
                       bool bCalcTrans,
                       bool bCalcDeriv)
{
    for (int i = 0; i < getNumDofs(); i++)
        mDofs.at(i)->set_q(_pose[i]);

    if (bCalcTrans)
    {
        if (bCalcDeriv)
            updateForwardKinematics(true, false);
        else
            updateForwardKinematics(false, false);
    }
}

void Skeleton::initKinematics()
{
    mRoot = mBodies[0];
    mGraph = new SkeletonGraph(getNumNodes());

	//--------------------------------------------------------------------------
	// Set dofs
	//--------------------------------------------------------------------------
	mDofs.clear();

    // calculate mass
    // init the dependsOnDof stucture for each bodylink
    for(int i = 0; i < getNumNodes(); i++)
    {
        mBodies.at(i)->setSkel(this);
        mBodies.at(i)->setDependDofList();
        mBodies.at(i)->init();
    }

    for (std::vector<Joint*>::iterator itrJoint = mJoints.begin();
		 itrJoint != mJoints.end();
		 ++itrJoint)
	{
        const std::vector<Dof*>& dofs = (*itrJoint)->getDofs();

        for (std::vector<Dof*>::const_iterator itrDof = dofs.begin();
			 itrDof != dofs.end();
			 ++itrDof)
		{
			mDofs.push_back((*itrDof));
		}

        if ((*itrJoint)->getParentBody() != NULL
                && (*itrJoint)->getChildBody() != NULL)
        {
            boost::add_edge((*itrJoint)->getParentBody()->getSkelIndex(),
                            (*itrJoint)->getChildBody()->getSkelIndex(),
                            *mGraph);
        }
	}

    //boost::write_graphviz(std::cout, *mGraph);
}

void Skeleton::updateForwardKinematics(bool _firstDerivative,
                                       bool _secondDerivative)
{
    _updateJointKinematics(_firstDerivative, _secondDerivative);
    _updateBodyForwardKinematics(_firstDerivative, _secondDerivative);
}

void Skeleton::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const
{
    mRoot->draw(_ri, _color, _useDefaultColor);
}

void Skeleton::_updateJointKinematics(bool _firstDerivative,
                                      bool _secondDerivative)
{
    for (std::vector<Joint*>::iterator itrJoint = mJoints.begin();
         itrJoint != mJoints.end();
         ++itrJoint)
    {
        (*itrJoint)->updateKinematics(_firstDerivative,
                                      _secondDerivative);
    }
}

void Skeleton::_updateBodyForwardKinematics(bool _firstDerivative,
                                            bool _secondDerivative)
{
    for (std::vector<BodyNode*>::iterator itrBody = mBodies.begin();
         itrBody != mBodies.end();
         ++itrBody)
    {
        (*itrBody)->updateTransformation();

        if (_firstDerivative)
            (*itrBody)->updateVelocity();

        if (_secondDerivative)
            (*itrBody)->updateAcceleration();
    }
}

void Skeleton::initDynamics()
{
    initKinematics();

    mM = MatrixXd::Zero(getDOF(), getDOF());
    mC = MatrixXd::Zero(getDOF(), getDOF());
    mCvec = VectorXd::Zero(getDOF());
    mG = VectorXd::Zero(getDOF());
    mCg = VectorXd::Zero(getDOF());
    set_tau(VectorXd::Zero(getDOF()));
    mFext = VectorXd::Zero(getNumDofs());

    // calculate mass
    // init the dependsOnDof stucture for each bodylink
    mTotalMass = 0.0;
    for(int i = 0; i < getNumNodes(); i++)
        mTotalMass += getBody(i)->getMass();
}

void Skeleton::computeInverseDynamics(const Eigen::Vector3d& _gravity)
{
    _updateJointKinematics();
    _inverseDynamicsFwdRecursion();
    _inverseDynamicsBwdRecursion(_gravity);
}

void Skeleton::computeForwardDynamics(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void Skeleton::computeForwardDynamicsID(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    int n = getNumDofs();

    // Save current tau
    Eigen::VectorXd tau_old = get_tau();

    // Set ddq as zero
    set_ddq(Eigen::VectorXd::Zero(n));

    //
    mM = Eigen::MatrixXd::Zero(n,n);

    // M(q) * ddq + b(q,dq) = tau
    computeInverseDynamics(_gravity);
    Eigen::VectorXd b = get_tau();
    mCg = b;

    // Calcualtion M
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorXd basis = Eigen::VectorXd::Zero(n);
        basis(i) = 1;
        set_ddq(basis);
        computeInverseDynamics(_gravity);
        mM.col(i) = get_tau() - b;
    }

    //
    set_tau(tau_old);

    // TODO:
    mMInv = mM.inverse();
    //mMInv = mM.ldlt().solve(MatrixXd::Identity(n,n));

//    Eigen::VectorXd new_ddq = mMInv * (tau_old - b);
//    set_ddq(new_ddq);
}

void Skeleton::computeForwardDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void Skeleton::computeHybridDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void Skeleton::computeEquationsOfMotionID(
        const Eigen::Vector3d& _gravity)
{
}

void Skeleton::computeEquationsOfMotionRecursive(
        const Eigen::Vector3d& _gravity)
{
}

void Skeleton::_inverseDynamicsFwdRecursion()
{
    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodies.begin();
         itrBody != mBodies.end();
         ++itrBody)
    {
        (*itrBody)->updateTransformation();
        (*itrBody)->updateVelocity();
        (*itrBody)->updateAcceleration();
    }
}

void Skeleton::_inverseDynamicsBwdRecursion(const Eigen::Vector3d& _gravity)
{
    // Backward recursion
    for (std::vector<dynamics::BodyNode*>::reverse_iterator ritrBody = mBodies.rbegin();
         ritrBody != mBodies.rend();
         ++ritrBody)
    {
        dynamics::BodyNode* body
                = dynamic_cast<dynamics::BodyNode*>(*ritrBody);

        body->updateBodyForce(_gravity);
        body->updateGeneralizedForce();
    }
}



} // namespace dynamics
} // namespace dart
