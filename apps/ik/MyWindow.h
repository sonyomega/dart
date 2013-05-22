#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"

namespace dart {
namespace kinematics {
    class Skeleton;
}
}

namespace dart {
namespace optimizer {
    class ObjectiveBox;
    class Var;
}
}

class MyWindow : public dart::yui::Win3D {
public:
 MyWindow(dart::kinematics::Skeleton* _skel): Win3D(), mSkel(_skel) {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;		
        mPersp = 30.f;
        mTrans[2] = -1.f;

        initIK();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

 protected:
    dart::kinematics::Skeleton *mSkel;
    dart::optimizer::ObjectiveBox *mObjBox;
    std::vector<dart::optimizer::Var*> mVariables;

    void initIK();
    void solveIK();
};

#endif
