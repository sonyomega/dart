#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"

/// @brief
class MyWindow : public dart::simulation::SimWindow
{
public:
    /// @brief
    MyWindow();

    /// @brief
    virtual ~MyWindow() {}

    /// @brief
    virtual void timeStepping();

private:
    /// @brief
    Eigen::VectorXd computeDamping();
};

#endif
