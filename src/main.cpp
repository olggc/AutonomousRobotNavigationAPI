#include <iostream>
#include "dummy.h"

using namespace std;

int main(int argc, const char** argv)
{
    float time = 30, t = 0, dt = 0.1;
    mat A,B,x,u;

    x.zeros(3,1);

    A.eye(3,3);

    B = {{cos(x(2,0))*dt, 0},
        {sin(x(2,0))*dt, 0},
        {0, dt}};

    u = {0.2,1};

    u = u.t();

    while(t<time)
    {
        x = do_process(A,B,x,u);
        t+=dt;
    }
}