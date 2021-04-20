#ifndef DUMMY_H
#define DUMMY_H
#include <armadillo>

using namespace arma;
/** Uma breve descrição da classe.
* Dummy class with do_process function
*/ 
class Dummy
{
    public:
        mat do_process(mat, mat, mat, mat);
};


#endif