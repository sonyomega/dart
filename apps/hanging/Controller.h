#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <vector>

namespace dart{
namespace dynamics {
class SkeletonDynamics;
class ConstraintDynamics;
}
}

namespace dart {
namespace kinematics {
class BodyNode;
}
}

class Controller {
public:
    Controller(dart::dynamics::SkeletonDynamics *_skel,
               dart::dynamics::ConstraintDynamics *_constraintHandle,
               double _t);
    virtual ~Controller() {}

    Eigen::VectorXd getTorques() { return mTorques; }
    double getTorque(int _index) { return mTorques[_index]; }
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; }
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    dart::dynamics::SkeletonDynamics* getSkel() { return mSkel; }
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; }
    Eigen::MatrixXd getKp() {return mKp; }
    Eigen::MatrixXd getKd() {return mKd; }
    void setConstrForces(const Eigen::VectorXd& _constrForce) { mConstrForces = _constrForce; }

protected:
    dart::dynamics::SkeletonDynamics *mSkel;
    dart::dynamics::ConstraintDynamics *mConstraintHandle;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    int mFrame;
    double mTimestep;
    double mPreOffset;
    Eigen::VectorXd mConstrForces; // SPD utilizes the current info about constraint forces
};
#endif // #CONTROLLER_H
