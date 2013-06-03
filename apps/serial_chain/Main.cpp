#include "utils/Paths.h"
#include "math/UtilsMath.h"
#include "simulation/ParserDART.h"
#include "dynamics/Skeleton.h"
#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World *myWorld
            = dart::simulation::readDARTFile(
                  DART_DATA_PATH"/dart/test/serial_chain.dart");
    assert(myWorld != NULL);

    int nDof = myWorld->getSkeleton(0)->getDOF();
    Eigen::VectorXd initPose(nDof);
    for (int i = 0; i < nDof; i++)
        initPose[i] = dart::math::random(-0.5, 0.5);
    myWorld->getSkeleton(0)->setPose(initPose);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);

    std::cout << "space bar: simulation on/off" << std::endl;
    std::cout << "'p': playback/stop" << std::endl;
    std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
    std::cout << "'v': visualization on/off" << std::endl;
    std::cout << "'1'--'4': programmed interaction" << std::endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Boxes");
    glutMainLoop();

    return 0;
}
