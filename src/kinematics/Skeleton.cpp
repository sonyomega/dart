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
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"

using namespace std;
using namespace Eigen;

namespace dart {
namespace kinematics {

Skeleton::Skeleton()
    : System()
{
}

Skeleton::~Skeleton()
{
}

BodyNode* Skeleton::createBodyNode() const
{
    return new BodyNode();
}

void Skeleton::addBody(BodyNode* _body, bool _addParentJoint)
{
    assert(_body != NULL);

    mBodies.push_back(_body);

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

	//--------------------------------------------------------------------------
	// Set dofs
	//--------------------------------------------------------------------------
	mDofs.clear();

	for (std::vector<Joint*>::iterator itrJoint = mJoints.begin();
		 itrJoint != mJoints.end();
		 ++itrJoint)
	{
        const std::vector<Coordinate*>& dofs = (*itrJoint)->getDofs();

        for (std::vector<Coordinate*>::const_iterator itrDof = dofs.begin();
			 itrDof != dofs.end();
			 ++itrDof)
		{
			mDofs.push_back((*itrDof));
		}
	}
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
        (*itrBody)->updateForwardKinematics(_firstDerivative,
                                            _secondDerivative);
    }
}


} // namespace kinematics
} // namespace dart
