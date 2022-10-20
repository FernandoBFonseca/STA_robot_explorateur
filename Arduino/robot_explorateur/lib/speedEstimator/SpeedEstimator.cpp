#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "speedEstimator.hpp"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
speedEstimator::speedEstimator(double* Position, double* Voltage, double* VelocityEst, double K, double tau, double const *L, unsigned long sample_time_ms){

    myPosition = Position;
    myVoltage = Voltage;
    myVelocityEst = VelocityEst;

    setParameters(K,tau,L,sample_time_ms);

    theta_est = 0.0;
    d_est = 0.0;

}

/* Compute() **********************************************************************
 *     This is where the magic happens, here we read the position and applied voltage
 *   and estimate the speed using a Luenberger observer.
 **********************************************************************************/
void speedEstimator::Compute(){ // * This is where the magic happens, here we read the position and applied voltage
                                //   and estimate the speed using a Luenberger observer.

    double theta = *myPosition;
    double u = *myVoltage;
    double v_est = *myVelocityEst;

    float dot_v_est     = a*v_est + b*u + b*d_est + L1*(theta-theta_est);
    float dot_theta_est = 1.0*v_est               + L2*(theta-theta_est);
    float dot_d_est     =                           L3*(theta-theta_est);  
    v_est     += dot_v_est*SampleTimeInSec;
    theta_est += dot_theta_est*SampleTimeInSec;
    d_est += dot_d_est*SampleTimeInSec;

    *myVelocityEst = v_est;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void speedEstimator::setParameters(double K, double tau, double const *L, unsigned long sample_time_ms){

    if (K<0 || tau<0 || sample_time_ms<0) return;

    dispK = K;
    dispTau = tau;

    a = (-1.0/tau);
    b = K/tau;
    
    L1 = L[0]; L2 = L[1]; L3 = L[2];

    SampleTime = sample_time_ms;
    SampleTimeInSec = ((double)SampleTime)/1000;

}

/* Status Funcions*************************************************************
 * Just because you set the K=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the Estimator. they're here for display
 * purposes.  this are the functions the Front-end uses for example
 ******************************************************************************/
double speedEstimator::getK(){return dispK;}
double speedEstimator::getTau(){return dispTau;}
double speedEstimator::getThetaEst(){return theta_est;}
double speedEstimator::getDEst(){return d_est;}