#define S_FUNCTION_LEVEL 2

#define S_FUNCTION_NAME threephase
#include "simstruc.h"
#include <math.h>
#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/

static void mdlInitializeSizes(SimStruct *S){
if (!ssSetNumInputPorts(S, 1)) return;
ssSetInputPortWidth(S, 0, 2);
ssSetInputPortDirectFeedThrough(S, 0, 1);
ssSetInputPortOverWritable(S, 0, 1);
if (!ssSetNumOutputPorts(S, 1)) return;
ssSetOutputPortWidth(S, 0, 3);
ssSetNumSampleTimes(S, 1);

ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); }

static void mdlInitializeSampleTimes(SimStruct *S) {
ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
ssSetOffsetTime(S, 0, 0.0); }

static void mdlOutputs(SimStruct *S, int_T tid) {
real_T *Y = ssGetOutputPortRealSignal(S,0);
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

real_T t = ssGetT(S);
real_T A,f,Va,Vb,Vc;
real_T phi = 3.14159;

A = U(0);
f = U(1);

Va = A*sin(2*phi*f*t);
Vb = A*sin(2*phi*f*t - (2*phi/3));
Vc = A*sin(2*phi*f*t + (2*phi/3));

Y[0] = Va;
Y[1] = Vb;
Y[2] = Vc;

}

static void mdlTerminate(SimStruct *S)
{ } /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif