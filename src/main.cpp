#include <iostream>
#include "RobotNavigation.h"

using namespace std;
using namespace arma;

int main(int argc, const char** argv)
{
    RobotNavigation Lucy;

    mat x;

    x.zeros(3,1);

    Lucy.set_state(x);

    cout << "RobotNavigation" << endl;
}