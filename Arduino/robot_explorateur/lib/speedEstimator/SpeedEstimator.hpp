//This lib unfortunately does not work :(
//I accept help  

#ifndef speedEstimator_h
#define speedEstimator_h
#define LIBRARY_VERSION	1.0.0

class speedEstimator{

    public:
    
    speedEstimator(double* Position, double* Voltage, double* VelocityEst, double const *sys, 
                   double const *L, unsigned long sample_time_ms);

    void Compute(); // * Performs the Estimation calculation
    
    void setParameters(double const *sys, double const *L, 
                       unsigned long sample_time_ms); // * While most users will set the parameters once in the 
      	                                              //   constructor, this function gives the user the option
                                                      //   of changing parameters during runtime

    //double getK();              // * we'll hold on to the tuning parameters in user-entered 		
    //double getTau();            //   format for display purposes
    
    double getThetaEst();       // * internal estimation parameters not needed in the real 
    double getDEst();           //   application but important for debugging

    private:

    double *myPosition;         // * Pointers to the Position, Voltage, and VelocityEst variables
    double *myVoltage;          //   This creates a hard link between the variables and the
    double *myVelocityEst;         //   Estimator, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.  

    double a1,a2,b1,b2;             
    //double dispK, dispTau;  // * we'll hold on to the tuning parameters in user-entered 
	        				//   format for display purposes
	
    double d_est, theta_est; // * internal estimation parameters
    double L1, L2, L3;       // * Luenberger observer parameters

    unsigned long SampleTime;
    double SampleTimeInSec;

};

#endif