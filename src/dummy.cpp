#include "dummy.h"
#include <iostream>

using namespace std;

mat do_process(mat A, mat B, mat x, mat u)
{
    x = A*x + B*u;
    cout << "Robot State = " << x << endl;

    return x;
}