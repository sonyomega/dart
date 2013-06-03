/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/23/2013
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "math/LieGroup.h"
#include "dynamics/BodyNode.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

/******************************************************************************/
TEST(BUILDING, BASIC)
{
	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
	// Bodies
    BodyNode body1;
    BodyNode body2;
    BodyNode body3;

	// Joints
	RevoluteJoint joint1;
	RevoluteJoint joint2;
	RevoluteJoint joint3;

	// Skeletons
    Skeleton skel1;

	// World
	World world;

	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
	// Bodies
	body1;
	body2;
	body3;

	// Joints
	joint1.setParentBody(NULL);	// world
	joint1.setChildBody(&body1);
	joint1.setLocalTransformFromParentBody(SE3());
	joint1.setLocalTransformFromChildBody(SE3());
	joint1.setAxis(so3(1.0, 0.0, 0.0));

	joint2.setParentBody(&body1);
	joint2.setChildBody(&body2);
	joint2.setLocalTransformFromParentBody(SE3());
	joint2.setLocalTransformFromChildBody(SE3());
	joint2.setAxis(so3(1.0, 0.0, 0.0));

	joint3.setParentBody(&body1);
	joint3.setChildBody(&body3);
	joint3.setLocalTransformFromParentBody(SE3());
	joint3.setLocalTransformFromChildBody(SE3());
	joint3.setAxis(so3(1.0, 0.0, 0.0));

	// Skeleton
	skel1.addBody(&body1, false);
	skel1.addBody(&body2, false);
	skel1.addBody(&body3, false);
	skel1.addJoint(&joint1);
	skel1.addJoint(&joint2);
	skel1.addJoint(&joint3);

	// World
	world.addSkeleton(&skel1);

	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
	EXPECT_TRUE(body1.getParentBody() == NULL);
	EXPECT_TRUE(body1.getChildBodies().size() == 2);
	EXPECT_TRUE(body1.getChildBody(0) == &body2);

	EXPECT_TRUE(body2.getParentBody() == &body1);
	EXPECT_TRUE(body2.getChildBodies().size() == 0);

	EXPECT_TRUE(body3.getParentBody() == &body1);
	EXPECT_TRUE(body3.getChildBodies().size() == 0);

	EXPECT_TRUE(joint1.getParentBody() == NULL);
	EXPECT_TRUE(joint1.getChildBody() == &body1);

	EXPECT_TRUE(joint2.getParentBody() == &body1);
	EXPECT_TRUE(joint2.getChildBody() == &body2);

	EXPECT_TRUE(joint3.getParentBody() == &body1);
	EXPECT_TRUE(joint3.getChildBody() == &body3);

    EXPECT_TRUE(skel1.getNumNodes() == 3);
	EXPECT_TRUE(skel1.getNumJoints() == 3);
	EXPECT_TRUE(skel1.getNumDofs() == 3);

	EXPECT_TRUE(world.getNumSkeletons() == 1);

	//--------------------------------------------------------------------------
	//
	//--------------------------------------------------------------------------
	skel1.updateForwardKinematics();
}

/******************************************************************************/
TEST(BUILDING, REVOLUTE_JOINT)
{

}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
