#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#define SPD
namespace dart {
namespace dynamics{
    class SkeletonDynamics;
}
}
namespace dart {
namespace kinematics{
    class FileInfoDof;
}
}

class Controller {
 public:
    Controller(dart::kinematics::FileInfoDof *_motion, dart::dynamics::SkeletonDynamics *_skel, double _t);
    virtual ~Controller() {}

    Eigen::VectorXd getTorques() { return mTorques; }
    void computeTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel);
    dart::dynamics::SkeletonDynamics* getSkel() { return mSkel; }
    Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; }
    void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; }

 protected: 
    dart::kinematics::FileInfoDof *mMotion;
    dart::dynamics::SkeletonDynamics *mSkel;
    Eigen::VectorXd mTorques;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    double mTimestep;
    int mSimFrame;
    int mInterval;
};
    
    

#endif // #CONTROLLER_H
