//
// Created by vinicius on 23/04/2021.
//

#ifndef AUTONOMOUSROBOTNAVIGATIONAPI_KALMANFILTER_H
#define AUTONOMOUSROBOTNAVIGATIONAPI_KALMANFILTER_H


class KalmanFilter {
public:

    mat A(3,3); // Process transition matrix
    mat B(3,2); // Sensor transition matrix
    mat Q(3,3); // Covariance process matrix
    mat R(3,3); // Covariance sensor matrix
    mat P(3,3);  //Covariance error matrix
    mat S(3,3); //Aux matrix
    mat Rsqrt(3,3); //Sensor standard deviation matrix
    mat Qsqrt(3,3); //Process standard deviation matrix
    float t0; //Initial time
    float t;  //Elapsed time
    float dt; //Sampling period
    mat X(3,1); //State actual matrix
    mat newX(3,1); //State updated matrix
    mat K(3,3); //Gain Kalman matrix
    mat e(3,1); //Error matrix
    mat z(3,1); //Hope sensor matrix


    /** Constructor method
     *This method will define some key elements for the filter.
     */
    KalmanFilter();
    /** Destructor method.
     *
     */
    ~KalmanFilter();
    /** This method uses the process transition matrix
     * Aims to set the state transition matrix attribute
     * @param Anew New transition process matrix
     */
    void setMatTransitionProcess(mat Anew);
    /** Method getter for transition process matrix
     * Will return a process transition matrix
     * @return Transition Process matrix
     */
    mat getMatTransitionProcess();
    /** This method uses the sensor transition matrix
     * Aims to set the sensor transition matrix attribute
     * @param Cnew New sensor transition matrix
     */
    void setMatTransitionSensor(mat Cnew);
    /** Method get for sensor transition matrix
     * Will return a sensor transition matrix
     * @return Sensor transition matrix
     */
    mat getMatTransitionSensor();
    /**This method uses the process covariance matrix
     * Aims to set the process covariance matrix attribute
     * @param Qnew New process covariance matrix
     */
    void setMatCovarianceProcess(mat Qnew);
    /** Method get for process covariance matrix
     * Will return a process covariance matrix
     * @return Process covariance matrix
     */
    mat getMatCovarianceProcess();
    /**This method uses the sensor covariance matrix
     * Aims to set the sensor covariance matrix attribute
     * @param Rnew New sensor covariance matrix
     */
    void setMatCovarianceSensor(mat Rnew);
    /** Method get for sensor covariance matrix
     * Will return a sensor covariance matrix
     * @return Sensor covariance matrix
     */
    mat getMatCovarianceSensor();
    /** This method uses the error covariance matrix
     * Aims to set the error covariance matrix attribute
     * @param Pnew New error covariance matrix
     */
    void setMatCovarianceError(mat Pnew);
    /** Method get for error covariance matrix
     * Will return a error covariance matrix
     * @return Error covariance matrix
     */
    mat getMatCovarianceError();
    /** Method set for sampling period
     * Aims to set the sampling period attribute
     * @param dtnew New sampling period
     */
    void setSamplingPeriod(float dtnew);
    /** Method get for sampling period
     * Will return a sampling period
     * @return Sampling period
     */
    float getSamplingPeriod();
    /** Method set for current state
     *  Aims to set the current state.
     * @param Xnew New current state
     */
    void setCurrentState(mat Xnew);
    /** Method get for current state
     *  Will return a current state
     * @return Current state
     */
    mat getCurrentState();
    /** Required function to start the simulation.
     * Will set the necessary matrix values for the Kalman Filter
     *
     * @param X0 Initial state
     * @param t0 Initial time
     */
    void init(mat X0, float t0);
    /** Function to calculate values of the states of the perfect system.
     * Required to start calculating real system values.
     * @param u Control vector
     */
    void predict(mat u);
    /** Function will calculate and update the values ​​of the states of the real robot.
     * It will update the kalman gain and error covariance values.
     * Values of the auxiliary matrix to calculate the kalman gain will also be updated.
     * @param u Control vector
     */
    void update(mat u);
    /** Function that aims to estimate the state he robot is.
     *
     * @param Xcurrent Current state
     * @param Ysensor  Sensor measurements
     * @param u Control vector
     */
    void estimate(mat Xcurrent,mat Ysensor, mat u);

};


#endif //AUTONOMOUSROBOTNAVIGATIONAPI_KALMANFILTER_H
