#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME PMSM2
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/

static void mdlInitializeSizes(SimStruct *S){
ssSetNumContStates(S, 4);
if (!ssSetNumInputPorts(S, 1)) return;
ssSetInputPortWidth(S, 0, 4);
ssSetInputPortDirectFeedThrough(S, 0, 1);
ssSetInputPortOverWritable(S, 0, 1);

if (!ssSetNumOutputPorts(S, 1)) return;
ssSetOutputPortWidth(S, 0, 6);
ssSetNumSampleTimes(S, 1);

ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); }

static void mdlInitializeSampleTimes(SimStruct *S) {
ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
ssSetOffsetTime(S, 0, 0.0); }


#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S) {

real_T *X0 = ssGetContStates(S);
int_T nStates = ssGetNumContStates(S);
int_T i;

/* initialize the states to 0.0 */
for (i=0; i < nStates; i++) {X0[i] = 0.0;} }

static void mdlOutputs(SimStruct *S, int_T tid) {
real_T *Y = ssGetOutputPortRealSignal(S,0);
real_T *X = ssGetContStates(S);
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);


//Deklarasi Variabel 3 fasa, 2 fasa, Id Iq, dll
real_T Va, Vb, Vc;
real_T V_alfa, V_beta;
real_T Vd, Vq;
real_T Ia, Ib, Ic;
real_T I_alfa, I_beta;
real_T Id, Iq, Id_dot, Iq_dot;
real_T omega_actual, omega_actual_dot, theta_m;
real_T theta_e, theta_e_dot;
real_T Te, TL;

//Parameter Given
real_T N = 4;
real_T flux = 0.121;
real_T Lsd = 16.61e-3;
real_T Lsq = 16.22e-3;
real_T Rs = 0.55;
real_T J = 0.01;
real_T pi = 3.141592654;


//Input
Va = U(0);
Vb = U(1);
Vc = U(2);

//Variabel State
omega_actual = X[0];
theta_e = X[1];
Id = X[2];
Iq = X[3];

//Transformasi 3 Fasa ke Alfa-Beta
V_alfa = 0.816497*(Va -0.5*Vb -0.5*Vc);
V_beta = 0.816497*(0.866025*Vb - 0.866025*Vc);

//Transformasi Alfa-Beta ke DQ
Vd = (V_alfa*cos(theta_e)) + (V_beta*sin(theta_e));
Vq = (-V_alfa*sin(theta_e)) + (V_beta*cos(theta_e));

//Transformasi Arus DQ ke Alfa-Beta
I_alfa = Id*cos(theta_e) - Iq*sin(theta_e);
I_beta = Id*sin(theta_e) + Iq*cos(theta_e);

//Transformasi Alfa-Beta ke 3 Fasa
Ia = 0.816497 * I_alfa;
Ib = 0.816497 * (-0.5 * I_alfa + 0.866025 * I_beta);
Ic = 0.816497 * (-0.5 * I_alfa - 0.866025 * I_beta);

//Model Mekanikal
Te = N*(flux + Id*(Lsd - Lsq))*Iq;
theta_m = theta_e / N;


//Output
Y[0] = Ia;
Y[1] = Ib;
Y[2] = Ic;
Y[3] = Te;
Y[4] = theta_e;
Y[5] = omega_actual;

}

#define MDL_DERIVATIVES
static void mdlDerivatives(SimStruct *S) {
real_T *dX = ssGetdX(S);
real_T *X = ssGetContStates(S);
InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

//Deklarasi Variables
real_T Va, Vb, Vc;
real_T V_alfa, V_beta;
real_T Vd, Vq;
real_T Ia, Ib, Ic;
real_T Id, Iq, Id_dot, Iq_dot;
real_T omega_actual, omega_actual_dot, theta_m;
real_T theta_e, theta_e_dot;
real_T Te, TL;

//Parameter
real_T N = 4;
real_T flux = 0.121;
real_T Lsd = 16.61e-3;
real_T Lsq = 16.22e-3;
real_T Rs = 0.55;
real_T J = 0.01;
real_T pi = 3.141592654;


//input 
Va = U(0);
Vb = U(1);
Vc = U(2);
TL = U(3);          //load

//Variabel memory
omega_actual = X[0];
theta_e = X[1];
Id = X[2];
Iq = X[3];

//Model Mechanical
Te = N*(flux + Id*(Lsd - Lsq))*Iq;
omega_actual_dot = (Te - TL)/J;
theta_e_dot = N*omega_actual;

//Transformasi 3 fasa ke 2 fasa (Alfa-Beta)
V_alfa = 0.816497 * (Va - 0.5 * Vb - 0.5 * Vc);
V_beta = 0.816497 * (0.866025 * Vb - 0.866025 * Vc);


//Transformasi 2 fasa (Alfa-Beta) Ke DQ
Vd = (V_alfa*cos(theta_e)) + (V_beta*sin(theta_e));
Vq = (-V_alfa*sin(theta_e)) + (V_beta*cos(theta_e));


//Persamaan State-Space
Id_dot = (1/Lsd)*(-Rs*Id + N*omega_actual*Lsq*Iq +Vd);
Iq_dot = (1/Lsq)*(-Rs*Iq - N*omega_actual*(Lsd*Id + flux) + Vq);


//Turunan State
dX[0] = omega_actual_dot;
dX[1] = theta_e_dot;
dX[2] = Id_dot;
dX[3] = Iq_dot;

}

static void mdlTerminate(SimStruct *S)
{} /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif