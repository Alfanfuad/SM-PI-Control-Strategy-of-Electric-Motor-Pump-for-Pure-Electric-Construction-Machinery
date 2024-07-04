#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME SMC
#include "simstruc.h"
#include <math.h>
#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/


static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumDiscStates(S, 2);
  if (!ssSetNumInputPorts(S, 1))
    return;

  ssSetInputPortWidth(S, 0, 4);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortOverWritable(S, 0, 1);

  if (!ssSetNumOutputPorts(S, 1))
    return;

  ssSetOutputPortWidth(S, 0, 1);
  ssSetNumSampleTimes(S, 1);
  ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_DISCRETE_VALUED_OUTPUT));
}
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, 1e-3);
  ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
  real_T *X0 = ssGetRealDiscStates(S);
  int_T nXStates = ssGetNumDiscStates(S);
  InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
  int_T i;
  /* initialize the states to 0.0 */
  for (i = 0; i < nXStates; i++)
  {
    X0[i] = 0.0;
  }
}
static void mdlOutputs(SimStruct *S, int_T tid)
{
  real_T *Y = ssGetOutputPortRealSignal(S, 0);
  real_T *X = ssGetRealDiscStates(S);
  Y[0] = X[1];
}


#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
  real_T *X = ssGetRealDiscStates(S);
  InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
  
  //deklarasi parameter SMC
  real_T dt = 1e-3;
  real_T error;
  real_T wreference, q_function, Epsilon, omega_actual; 
  real_T D_function, c_function, s_function, error_old;
  real_T integral_u_old, integral_u, signum_s, iq_new;
          
  wreference = U(0);
  Epsilon = U(1);
  q_function = U(2);
  omega_actual = U(3);
  
  //Input parameter SMC
  real_T J = 0.01;
  real_T Pn = 4;
  real_T flux = 5;

  // SMC Control
  error = wreference - omega_actual;           
  D_function = (3 * Pn * flux)/(2 * J);
  s_function = c_function*(wreference - omega_actual) - omega_actual;
  
  if (s_function < 0){
      signum_s = -1;
  }
  else if(s_function > 0){
      signum_s = 1;
  }
  else{
      signum_s = 0;
  }
  
  integral_u_old = X[0];
  integral_u = (1/D_function) * ((q_function*c_function*error)+((c_function + q_function)*((error - error_old)/dt)) + Epsilon*signum_s);
  X[0] = integral_u;
  
  iq_new = integral_u_old + integral_u * dt;
  
  
  X[1] = iq_new;
}

static void mdlTerminate(SimStruct *S)
{
} /*Keep this function empty since no memory is allocated*/
#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /*MEX-file interface mechanism*/
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif
