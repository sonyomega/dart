/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
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

#include "dynamics/Shape.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"

#include "collision/fcl/FCLCollisionNode.h"
#include "collision/fcl/FCLCollisionDetector.h"
#include "collision/fcl/PrimitiveCollision.h"

namespace dart {
namespace collision {

FCLCollisionDetector::FCLCollisionDetector()
    : CollisionDetector(),
      //mBroadPhaseCollMgr(new fcl::DynamicAABBTreeCollisionManager())
      mBroadPhaseCollMgr(new fcl::NaiveCollisionManager())
{
    mNumMaxContacts = 500;
}

FCLCollisionDetector::~FCLCollisionDetector()
{
    delete mBroadPhaseCollMgr;
}

CollisionNode* FCLCollisionDetector::createCollisionNode(
        dynamics::BodyNode* _bodyNode)
{
    return new FCLCollisionNode(_bodyNode);
}

void FCLCollisionDetector::addCollisionSkeletonNode(
        dynamics::BodyNode* _bodyNode, bool _recursive)
{
    CollisionNode* collNode = createCollisionNode(_bodyNode);
    FCLCollisionNode* fclCollNode = static_cast<FCLCollisionNode*>(collNode);
    collNode->setIndex(mCollisionNodes.size());
    mCollisionNodes.push_back(collNode);
    mBodyCollisionMap[_bodyNode] = collNode;
    mCollidablePairs.push_back(std::vector<bool>(mCollisionNodes.size() - 1, true));
    for (int i = 0; i < fclCollNode->getNumCollisionObjects(); i++)
    {
        mBroadPhaseCollMgr->registerObject(fclCollNode->getCollisionObject(i));
        std::vector<fcl::CollisionObject*> tmp;
        mBroadPhaseCollMgr->getObjects(tmp);
        std::cout << "num: " << tmp.size() << std::endl;
    }

    std::cout << _bodyNode->getName() << std::endl;

    if(_recursive)
    {
        for (int i = 0; i < _bodyNode->getNumChildJoints(); i++)
            addCollisionSkeletonNode(_bodyNode->getChildBodyNode(i), true);
    }
}

#define BROAD

bool FCLCollisionDetector::checkCollision(bool _checkAllCollisions,
                                          bool _calculateContactPoints)
{
    // TODO: _checkAllCollisions
    clearAllContacts();

#ifdef BROAD
    static bool tmpCHK = false;
    if (tmpCHK == false)
    {
        mBroadPhaseCollMgr->setup();
        tmpCHK = true;
    }

    CollisionData colData;
    colData.request.enable_contact = _calculateContactPoints;
    colData.request.num_max_contacts = mNumMaxContacts;
    colData.request.enable_cached_gjk_guess = true;
    colData.mCollDetecter = this;

    _updateCollisionObjects();

    mBroadPhaseCollMgr->update();
    mBroadPhaseCollMgr->collide(&colData, defaultCollisionFunction);

    unsigned int numContacts = colData.result.numContacts();

    for (unsigned int m = 0; m < numContacts; ++m)
    {
        const fcl::Contact& contact = colData.result.getContact(m);

        Contact contactPair;
        contactPair.point(0) = contact.pos[0];
        contactPair.point(1) = contact.pos[1];
        contactPair.point(2) = contact.pos[2];
        contactPair.normal(0) = -contact.normal[0];
        contactPair.normal(1) = -contact.normal[1];
        contactPair.normal(2) = -contact.normal[2];
        contactPair.collisionNode1 = findCollisionNode(contact.o1);
        contactPair.collisionNode2 = findCollisionNode(contact.o2);
        assert(contactPair.collisionNode1 != NULL);
        assert(contactPair.collisionNode2 != NULL);
        contactPair.penetrationDepth = contact.penetration_depth;

        mContacts.push_back(contactPair);
    }

    std::vector<bool> markForDeletion(numContacts, false);
    for (int m = 0; m < numContacts; m++)
    {
        for (int n = m + 1; n < numContacts; n++)
        {
            Eigen::Vector3d diff = mContacts[m].point - mContacts[n].point;

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
            mContacts.erase(mContacts.begin() + m);
    }
#else
    fcl::CollisionResult result;

    // only evaluate contact points if data structure for returning the contact
    // points was provided
    fcl::CollisionRequest request;
    request.enable_contact = _calculateContactPoints;
    request.num_max_contacts = mNumMaxContacts;
    request.enable_cached_gjk_guess = true;
    request.gjk_solver_type = fcl::GST_INDEP;

    _updateCollisionObjects();

    for(int i = 0; i < mCollisionNodes.size(); i++)
    for(int j = i + 1; j < mCollisionNodes.size(); j++)
    {
        result.clear();
        FCLCollisionNode* collNode1 = dynamic_cast<FCLCollisionNode*>(mCollisionNodes[i]);
        FCLCollisionNode* collNode2 = dynamic_cast<FCLCollisionNode*>(mCollisionNodes[j]);

        if (!isCollidable(collNode1, collNode2))
            continue;

        for(int k = 0; k < collNode1->getNumCollisionObjects(); k++)
        for(int l = 0; l < collNode2->getNumCollisionObjects(); l++)
        {
            int currContactNum = mContacts.size();

            dynamics::Shape* shape1 =
                    static_cast<dynamics::Shape*>(collNode1->getCollisionObject(k)->getUserData());
            dynamics::Shape* shape2 =
                    static_cast<dynamics::Shape*>(collNode2->getCollisionObject(l)->getUserData());

            std::vector<Contact> tmp;

            unsigned int numContacts = 0;

            if ((shape1->getShapeType() != dynamics::Shape::P_MESH) &&
                (shape2->getShapeType() != dynamics::Shape::P_MESH))
            {
                Eigen::Isometry3d T1 = collNode1->getBodyNode()->getWorldTransform() * shape1->getLocalTransform();
                Eigen::Isometry3d T2 = collNode2->getBodyNode()->getWorldTransform() * shape2->getLocalTransform();

                collide(shape1, T1, shape2, T2, tmp);

                numContacts = tmp.size();

                for (unsigned int m = 0; m < numContacts; ++m)
                {
                    tmp[m].collisionNode1 = collNode1;
                    tmp[m].collisionNode2 = collNode2;
                    mContacts.push_back(tmp[m]);
                }
            }
            else
            {
                fcl::collide(collNode1->getCollisionObject(k),
                             collNode2->getCollisionObject(l),
                             request, result);

                numContacts = result.numContacts();

                for (unsigned int m = 0; m < numContacts; ++m)
                {
                    const fcl::Contact& contact = result.getContact(m);

                    Contact contactPair;
                    contactPair.point(0) = contact.pos[0];
                    contactPair.point(1) = contact.pos[1];
                    contactPair.point(2) = contact.pos[2];
                    //--------------------------------------------------------------
                    // box(mesh)-box(mesh)        : -normal
                    // box(fcl:mesh)-box(fcl:mesh): -normal
                    // box(mesh)-sphere           : +normal
                    // sphere-sphere              : +normal
                    // sphere-box                 : +normal
                    // box-box                    : -normal
                    //--------------------------------------------------------------
                    contactPair.normal(0) = -contact.normal[0];
                    contactPair.normal(1) = -contact.normal[1];
                    contactPair.normal(2) = -contact.normal[2];
                    contactPair.collisionNode1 = findCollisionNode(contact.o1);
                    contactPair.collisionNode2 = findCollisionNode(contact.o2);
                    assert(contactPair.collisionNode1 != NULL);
                    assert(contactPair.collisionNode2 != NULL);
                    //contactPair.bdID1 = collisionNodePair.collisionNode1->getBodyNodeID();
                    //contactPair.bdID2 = collisionNodePair.collisionNode2->getBodyNodeID();
                    contactPair.penetrationDepth = contact.penetration_depth;

                    mContacts.push_back(contactPair);
                }
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
#endif
    return !mContacts.empty();
}

bool FCLCollisionDetector::checkCollision(CollisionNode* _node1,
                                          CollisionNode* _node2,
                                          bool _calculateContactPoints)
{
    assert(false); // function not implemented
    return false;
}

void FCLCollisionDetector::_updateCollisionObjects()
{
    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        FCLCollisionNode* fclCollNode =
                static_cast<FCLCollisionNode*>(mCollisionNodes[i]);

        for (int j = 0; j < fclCollNode->getNumCollisionObjects(); j++)
        {
            fcl::CollisionObject* fclCollObj =
                    fclCollNode->getCollisionObject(j);

            dynamics::Shape* collShape =
                    static_cast<dynamics::Shape*>(fclCollObj->getUserData());
            fclCollObj->setTransform(
                        convTransform(
                            fclCollNode->getBodyNode()->getWorldTransform() *
                            collShape->getLocalTransform()));
        }
    }
}

CollisionNode* FCLCollisionDetector::findCollisionNode(
        const fcl::CollisionGeometry* _fclCollGeom) const
{
    int numCollNodes = mCollisionNodes.size();
    for (int i = 0; i < numCollNodes; ++i)
    {
        FCLCollisionNode* collisionNode = dynamic_cast<FCLCollisionNode*>(mCollisionNodes[i]);
        for(int j = 0; j < collisionNode->getNumCollisionObjects(); j++)
        {
            if (collisionNode->getCollisionObject(j)->getCollisionGeometry() == _fclCollGeom)
                return mCollisionNodes[i];
        }
    }
    return NULL;
}

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
{
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const fcl::CollisionRequest& request = cdata->request;
  fcl::CollisionResult& result = cdata->result;

  if (cdata->done)
      return true;


  if (cdata->mCollDetecter->findCollisionNode(o1->getCollisionGeometry()) ==
          cdata->mCollDetecter->findCollisionNode(o2->getCollisionGeometry()))
      return cdata->done;

  if (!cdata->mCollDetecter->isCollidable(
          cdata->mCollDetecter->findCollisionNode(o1->getCollisionGeometry()),
          cdata->mCollDetecter->findCollisionNode(o2->getCollisionGeometry())))
  {
      return cdata->done;
  }

//  dynamics::Shape* shape1 =
//          static_cast<dynamics::Shape*>(o1->getUserData());
//  dynamics::Shape* shape2 =
//          static_cast<dynamics::Shape*>(o2->getUserData());

//  std::vector<Contact> tmp;

//  unsigned int numContacts = 0;

//  if ((shape1->getShapeType() != dynamics::Shape::P_MESH) &&
//      (shape2->getShapeType() != dynamics::Shape::P_MESH))
//  {
//      Eigen::Isometry3d T1 = collNode1->getBodyNode()->getWorldTransform() * shape1->getLocalTransform();
//      Eigen::Isometry3d T2 = collNode2->getBodyNode()->getWorldTransform() * shape2->getLocalTransform();

//      collide(shape1, T1, shape2, T2, tmp);

//      numContacts = tmp.size();

//      for (unsigned int m = 0; m < numContacts; ++m)
//      {
//          tmp[m].collisionNode1 = collNode1;
//          tmp[m].collisionNode2 = collNode2;
//          mContacts.push_back(tmp[m]);
//      }
//  }
//  else
  {
      fcl::collide(o1, o2, request, result);
  }

  if (result.numContacts() > 0)
      int a = 10;

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

fcl::Transform3f convTransform(const Eigen::Isometry3d& _T)
{
    return fcl::Transform3f(
                fcl::Matrix3f(_T(0,0), _T(0,1), _T(0,2),
                              _T(1,0), _T(1,1), _T(1,2),
                              _T(2,0), _T(2,1), _T(2,2)),
                fcl::Vec3f(_T(0,3), _T(1,3), _T(2,3))
                );
}

} // namespace collision
} // namespace dart
