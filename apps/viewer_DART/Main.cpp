#include "utils/Paths.h"
#include "simulation/ParserDART.h"
#include "MyWindow.h"

int main(int argc, char* argv[])
{
    // load a skeleton file
    // create and initialize the world
    dart::simulation::World *myWorld
            = dart::simulation::readDARTFile(
                  //DART_DATA_PATH"/dart/single_pendulum.dart");
                  //DART_DATA_PATH"/dart/double_pendulum.dart");
                  DART_DATA_PATH"/dart/boxes.dart");
    assert(myWorld != NULL);

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);

    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1'--'4': programmed interaction" << endl;

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Boxes");
    glutMainLoop();

    return 0;
}
