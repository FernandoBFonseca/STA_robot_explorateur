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
speedEstimator::speedEstimator(double* Position, double* Voltage, double* VelocityEst, double const *sys, double const *L, unsigned long sample_time_ms){

    myPosition = Position;
    myVoltage = Voltage;
    myVelocityEst = VelocityEst;

    setParameters(sys,L,sample_time_ms);

    theta_est = 0.0;
    d_est = 0.0;

}

/* Compute() **********************************************************************
 *     This is where the magic happens, here we read the position and applied voltage
 *   and estimate the speed using a Luenberger observer.
 **********************************************************************************/
void speedEstimator::Compute(){ // * This is where the magic happens, here we read the position and applied voltage
                                //   and estimate the speed using a discreate Luenberger observer.

    /*
      y = theta
  
      X_est = [v_est,theta_est,d_est]'
      y_est = theta_est
      
      X_est_n+1 = A*X_est_n + B*u + L*(y-y_est)
      y_est_n = C*X_est_n

      Where 

      A = [a1 0 a2;
           b1 1 b2;
            0 0  1;]
      B = [a2;
           b2;
            0;]
      C = [0 1 0]

      L = [L1;
           L2;
           L3;]
    
    */

    double theta = *myPosition;
    double u = *myVoltage;
    double v_est = *myVelocityEst;

    double new_v_est,new_theta_est,new_d_est;

    new_v_est     = a1*v_est +             a2*d_est + a2*u + L1*(theta-theta_est);
    new_theta_est = b1*v_est + theta_est + b2*d_est + b2*u + L2*(theta-theta_est);
    //new_d_est     =                           d_est +        L3*(theta-theta_est);  


    v_est     = new_v_est;
    theta_est = new_theta_est;
    //d_est     = new_d_est; 

    *myVelocityEst = v_est;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void speedEstimator::setParameters(double const *sys, double const *L, unsigned long sample_time_ms){

    //if (K<0 || tau<0 || sample_time_ms<0) return;
    if (sys == NULL || L == NULL || sample_time_ms<0) return;

    //dispK = K;
    //dispTau = tau;

    //a = (-1.0/tau);
    //b = K/tau;
    
    a1 = sys[0], a2 = sys[1], b1 = sys[2], b2 = sys[3];
    L1 = L[0]; L2 = L[1]; L3 = L[2];

    SampleTime = sample_time_ms;
    SampleTimeInSec = ((double)SampleTime)/1000;

}

/* Status Funcions*************************************************************
 * Just because you set the K=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the Estimator. they're here for display
 * purposes.  this are the functions the Front-end uses for example
 ******************************************************************************/
//double speedEstimator::getK(){return dispK;}
//ouble speedEstimator::getTau(){return dispTau;}
double speedEstimator::getThetaEst(){return theta_est;}
double speedEstimator::getDEst(){return d_est;}