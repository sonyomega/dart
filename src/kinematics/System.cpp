/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/11/2013
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

#include <algorithm>

#include "kinematics/Dof.h"
#include "kinematics/System.h"

namespace kinematics
{

System::System()
{
}

System::~System()
{
}

unsigned int System::getNumDOFs()
{
	return mDofs.size();
}

int System::getIndexOfDOF(const Dof* _dof) const
{
	std::vector<Dof*>::const_iterator itrDof
			= std::find(mDofs.begin(), mDofs.end(), _dof);

	if (itrDof == mDofs.end())
	{
		return -1;
	}
	else
	{
		return (int)std::distance(mDofs.begin(), itrDof);
	}
}

void System::set_q(const Eigen::VectorXd& _q)
{
	assert(mDofs.size() == _q.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		mDofs[i]->set_q(_q(i));
	}
}

void System::set_dq(const Eigen::VectorXd& _dq)
{
	assert(mDofs.size() == _dq.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		mDofs[i]->set_dq(_dq(i));
	}
}

void System::set_ddq(const Eigen::VectorXd& _ddq)
{
	assert(mDofs.size() == _ddq.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		mDofs[i]->set_ddq(_ddq(i));
	}
}

void System::setTorque(const Eigen::VectorXd& _tau)
{
	assert(mDofs.size() == _tau.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		mDofs[i]->setTorque(_tau(i));
	}
}

Eigen::VectorXd System::get_q() const
{
	Eigen::VectorXd q(mDofs.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		q(i) = mDofs[i]->get_q();
	}

	return q;
}

Eigen::VectorXd System::get_dq() const
{
	Eigen::VectorXd dq(mDofs.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		dq(i) = mDofs[i]->get_dq();
	}

	return dq;
}

Eigen::VectorXd System::get_ddq() const
{
	Eigen::VectorXd ddq(mDofs.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		ddq(i) = mDofs[i]->get_ddq();
	}

	return ddq;
}

Eigen::VectorXd System::getTorque() const
{
	Eigen::VectorXd tau(mDofs.size());

	unsigned int size = mDofs.size();

	for (int i = 0; i < size; i++)
	{
		tau(i) = mDofs[i]->getTorque();
	}

	return tau;
}

} // namespace kinematics

