#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "dart/yui/Win3D.h"
#include "dart/simulation/SimWindow.h"

class MyWindow : public dart::simulation::SimWindow
{
 public:
 MyWindow(): SimWindow() 
        {
            mForce = Eigen::Vector3d::Zero();
        }
    virtual ~MyWindow() {}
    
    virtual void timeStepping();
    virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    //  virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
 private:
    Eigen::Vector3d mForce;
};

#endif
