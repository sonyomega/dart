#include "constraint/PointConstraint.h"

#include "math/UtilsMath.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"
#include "dynamics/BodyNode.h"

using namespace Eigen;
using namespace dart;
using namespace math;

namespace dart {
namespace constraint {

PointConstraint::PointConstraint(dynamics::BodyNode *_body, Vector3d _offset, Vector3d _target, int _skelIndex) {
    mBody = _body;
    mOffset = _offset;
    mTarget = _target;
    mSkelIndex = _skelIndex;
    mJ = MatrixXd::Zero(3, mBody->getSkel()->getDOF());
    mNumRows = 3;
}

PointConstraint::~PointConstraint() {
}

void PointConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
    getJacobian();
    dynamics::Skeleton *skel = (dynamics::Skeleton*)mBody->getSkel();
    _J[mSkelIndex].block(_rowIndex, 0, 3, skel->getDOF()) = mJ;
    Vector3d worldP = math::xformHom(mBody->getWorldTransform().matrix(), mOffset);
    VectorXd qDot = skel->get_dq();
    _C.segment(_rowIndex, 3) = worldP - mTarget;
    _CDot.segment(_rowIndex, 3) = mJ * qDot;
}

void PointConstraint::getJacobian() {
    MatrixXd JBody = mBody->getJacobianWorldAtPoint_LinearPartOnly(mOffset);
    for(int i = 0; i < mBody->getNumDependentDofs(); i++) {
        int dofIndex = mBody->getDependentDof(i);
        mJ.col(dofIndex) = JBody.col(dofIndex);
    }
}

} // namespace constraint
} // namespace dart
