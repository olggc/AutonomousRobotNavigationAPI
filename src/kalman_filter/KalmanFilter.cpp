//
// Created by vinicius on 23/04/2021.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{

}
KalmanFilter::~KalmanFilter()
{

}
void KalmanFilter::setMatTransitionProcess(mat Anew)
{
    this->A()=Anew;
}
mat KalmanFilter::getMatTransitionProcess()
{
    return A();
}
void KalmanFilter::setMatTransitionSensor(mat Cnew)
{
    this->C=Cnew;
}
mat KalmanFilter::getMatTransitionSensor()
{
    return C();
}
void KalmanFilter::setMatCovarianceProcess(mat Qnew)
{
    this->Q()=Qnew;
}
mat KalmanFilter::getMatCovarianceProcess()
{
    return Q();
}
void KalmanFilter::setMatCovarianceSensor(mat Rnew)
{
    this->R()=Rnew;
}
mat KalmanFilter::getMatCovarianceSensor()
{
    return R();
}
void KalmanFilter::setMatCovarianceError(mat Pnew)
{
    this->P()=Pnew;
}
mat KalmanFilter::getMatCovarianceError()
{
    return P();
}
void KalmanFilter::setSamplingPeriod(float dtnew)
{
    this->dt = dtnew;
}
float KalmanFilter::getSamplingPeriod()
{
    return dt;
}
void KalmanFilter::setCurrentState(mat Xnew)
{
    this->X()=Xnew;
}
mat KalmanFilter::getCurrentState()
{
    return X();
}
void KalmanFilter::init(mat X0, float t0)
{

}
void KalmanFilter::predict(mat u)
{

}
void KalmanFilter::update(mat u)
{

}
void KalmanFilter::estimate(mat Xcurrent, mat Ysensor, mat u)
{

}