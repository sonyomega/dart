/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/01/2013
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

#ifndef DART_COLLISION_FCL_CONLLISION_DETECTOR_H
#define DART_COLLISION_FCL_CONLLISION_DETECTOR_H

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include "collision/CollisionDetector.h"

namespace dart {
namespace collision {

class FCLCollisionNode;

/// @brief
class FCLCollisionDetector : public CollisionDetector {
public:
    /// @brief
    FCLCollisionDetector();

    /// @brief
    virtual ~FCLCollisionDetector();

    // Documentation inherited
    virtual CollisionNode* createCollisionNode(dynamics::BodyNode* _bodyNode);

    // Documentation inherited
    virtual void addCollisionSkeletonNode(dynamics::BodyNode* _bodyNode,
                                          bool _recursive);
    // Documentation inherited
    virtual bool checkCollision(bool _checkAllCollisions,
                                bool _calculateContactPoints);

    CollisionNode* findCollisionNode(
            const fcl::CollisionGeometry* _fclCollGeom) const;

protected:
    virtual bool checkCollision(CollisionNode* _node1,
                                CollisionNode* _node2,
                                bool _calculateContactPoints);

    /// @brief Broad phase collision manager
    fcl::BroadPhaseCollisionManager* mBroadPhaseCollMgr;

private:
    /// @brief
    void _updateCollisionObjects();
};

/// @brief Collision data stores the collision request and the result given by
/// collision algorithm.
struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  fcl::CollisionRequest request;

  /// @brief Collision result
  fcl::CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;

  FCLCollisionDetector* mCollDetecter;
};

bool defaultCollisionFunction(fcl::CollisionObject* o1,
                              fcl::CollisionObject* o2,
                              void* cdata_);

fcl::Transform3f convTransform(const Eigen::Isometry3d& _T);

} // namespace collision
} // namespace dart

#endif // #ifndef DART_COLLISION_FCL_CONLLISION_DETECTOR_H
