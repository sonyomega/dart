/* Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 03/25/2013
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "kinematics/Dof.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "simulation/World.h"

namespace dart {
namespace simulation {

////////////////////////////////////////////////////////////////////////////////
World::World()
    : mTime(0.0),
      mTimeStep(0.001)
{
    mIndices.push_back(0);
    mCollisionHandle = new dynamics::ConstraintDynamics(mSkeletons, mTimeStep);
}

////////////////////////////////////////////////////////////////////////////////
World::~World()
{
}

////////////////////////////////////////////////////////////////////////////////
void World::setTimeStep(double _timeStep)
{
    mTimeStep = _timeStep;
    mCollisionHandle->setTimeStep(_timeStep);
}

////////////////////////////////////////////////////////////////////////////////
void World::reset()
{
//    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
//        mSkeletons[i]->restoreInitState();

    // Reset time and number of frames.
    mTime = 0;
    mFrame = 0;
}

////////////////////////////////////////////////////////////////////////////////
void World::step()
{
    //--------------------------------------------------------------------------
    // Apply forces (internal force + external force)

    //--------------------------------------------------------------------------
    // Forward dynamics: [q(k), dq(k), tau(k)] --> ddq(k)
    _computeForwardDynamics();

    //--------------------------------------------------------------------------
    // Integration: [q(k), dq(k), ddq(k)] --> [q(k+1), dq(k+1)]
    _integrate();

    //--------------------------------------------------------------------------
    // Forward kinematics: [q(k+1), dq(k+1)] --> W, V, dV, ...
    _updateForwardKinematics();

    mTime += mTimeStep;
    mFrame++;
}

////////////////////////////////////////////////////////////////////////////////
dynamics::SkeletonDynamics* World::getSkeleton(int _index) const
{
    return mSkeletons[_index];
}

////////////////////////////////////////////////////////////////////////////////
dynamics::SkeletonDynamics* World::getSkeleton(const std::string& _name) const
{
    for (std::vector<dynamics::SkeletonDynamics*>::const_iterator itrSkeleton
         = mSkeletons.begin();
         itrSkeleton != mSkeletons.end();
         ++itrSkeleton)
    {
        if ((*itrSkeleton)->getName() == _name)
            return *itrSkeleton;
    }

    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
void World::addSkeleton(dynamics::SkeletonDynamics* _skeleton)
{
    assert(_skeleton != NULL);

    mSkeletons.push_back(_skeleton);

    _skeleton->initKinematics();
    _skeleton->initDynamics();

    mIndices.push_back(mIndices.back() + _skeleton->getNumDofs());
}

bool World::checkCollision(bool checkAllCollisions)
{
    return mCollisionHandle->getCollisionChecker()->checkCollision(checkAllCollisions, false);
}

void World::_computeForwardDynamics()
{
}

void World::_integrate()
{
}

void World::_updateForwardKinematics()
{
}

} // namespace simulation
} // namespace dart
