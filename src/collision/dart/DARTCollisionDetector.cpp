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

#include "dynamics/Shape.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"

#include "collision/dart/DARTCollide.h"
#include "collision/dart/DARTCollisionNode.h"
#include "collision/dart/DARTCollisionDetector.h"

namespace dart {
namespace collision {

DARTCollisionDetector::DARTCollisionDetector()
    : CollisionDetector()
{
}

DARTCollisionDetector::~DARTCollisionDetector()
{
}

CollisionNode* DARTCollisionDetector::createCollisionNode(
        dynamics::BodyNode* _bodyNode)
{
    return new DARTCollisionNode(_bodyNode);
}

bool DARTCollisionDetector::checkCollision(bool _checkAllCollisions,
                                           bool _calculateContactPoints)
{
    clearAllContacts();

    std::vector<Contact> contacts;

    for(int i = 0; i < mCollisionNodes.size(); i++)
    for(int j = i + 1; j < mCollisionNodes.size(); j++)
    {
        DARTCollisionNode* collNode1 = dynamic_cast<DARTCollisionNode*>(mCollisionNodes[i]);
        DARTCollisionNode* collNode2 = dynamic_cast<DARTCollisionNode*>(mCollisionNodes[j]);

        if (!isCollidable(collNode1, collNode2))
            continue;

        //for(int k = 0; k < collNode1->getNumCollisionGeometries(); k++)
        //for(int l = 0; l < collNode2->getNumCollisionGeometries(); l++)
        for(int k = 0; k < 1; k++)
        for(int l = 0; l < 1; l++)
        {
            int currContactNum = mContacts.size();

            contacts.clear();
            collide(collNode1->getBodyNode()->getCollisionShape(0),
                    collNode1->getBodyNode()->getWorldTransform(),
                    collNode2->getBodyNode()->getCollisionShape(0),
                    collNode2->getBodyNode()->getWorldTransform(),
                    contacts);

            unsigned int numContacts = contacts.size();

            for (unsigned int m = 0; m < numContacts; ++m)
            {
                Contact contactPair;
                contactPair = contacts[m];
                contactPair.collisionNode1 = collNode1;
                contactPair.collisionNode2 = collNode2;
                assert(contactPair.collisionNode1 != NULL);
                assert(contactPair.collisionNode2 != NULL);

                mContacts.push_back(contactPair);
            }

            std::vector<bool> markForDeletion(numContacts, false);
            for (int m = 0; m < numContacts; m++)
            {
                for (int n = m + 1; n < numContacts; n++)
                {
                    Eigen::Vector3d diff =
                            mContacts[currContactNum + m].point -
                            mContacts[currContactNum + n].point;
                    if (diff.dot(diff) < 1e-6)
                    {
                        markForDeletion[m] = true;
                        break;
                    }
                }
            }
            for (int m = numContacts - 1; m >= 0; m--)
            {
                if (markForDeletion[m])
                    mContacts.erase(mContacts.begin() + currContactNum + m);
            }
        }
    }

    return !mContacts.empty();
}

bool DARTCollisionDetector::checkCollision(CollisionNode* _node1,
                                          CollisionNode* _node2,
                                          bool _calculateContactPoints)
{
    assert(false); // function not implemented
    return false;
}

} // namespace collision
} // namespace dart
