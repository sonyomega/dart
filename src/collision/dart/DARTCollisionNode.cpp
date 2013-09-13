/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 09/13/2013
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

#include <cmath>
#include <iostream>

#include <assimp/scene.h>

#include "dynamics/BodyNode.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/CylinderShape.h"
#include "dynamics/MeshShape.h"
#include "collision/dart/DARTCollisionNode.h"

namespace dart {
namespace collision {

DARTCollisionNode::DARTCollisionNode(dynamics::BodyNode* _bodyNode)
    : CollisionNode(_bodyNode)
{
    for(int i = 0; i < _bodyNode->getNumCollisionShapes(); i++)
    {
        dynamics::Shape* shape = _bodyNode->getCollisionShape(i);
        mShapes.push_back(shape);
        switch (shape->getShapeType())
        {
            case dynamics::Shape::P_BOX:
            {

                break;
            }
            case dynamics::Shape::P_ELLIPSOID:
            {
                dynamics::EllipsoidShape* ellipsoid
                        = dynamic_cast<dynamics::EllipsoidShape*>(shape);

                break;
            }
            case dynamics::Shape::P_CYLINDER:
            {
                dynamics::CylinderShape* cylinder
                        = dynamic_cast<dynamics::CylinderShape*>(shape);

                break;
            }
            case dynamics::Shape::P_MESH:
            {
                dynamics::MeshShape *shapeMesh
                        = dynamic_cast<dynamics::MeshShape *>(shape);

                break;
            }
            default:
            {
                std::cout << "ERROR: Collision checking does not support "
                          << _bodyNode->getName()
                          << "'s Shape type\n";
                break;
            }
        }
    }
}

DARTCollisionNode::~DARTCollisionNode()
{
}

} // namespace collision
} // namespace dart
