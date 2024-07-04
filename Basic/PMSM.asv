#define S_FUNCTION_LEVEL 2

#define S_FUNCTION_NAME PMSM
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

//Deklarasi Variables
real_T Va, Vb, Vc;
real_T V_alfa, V_beta;
real_T Vd, Vq;
real_T Ia, Ib, Ic;
real_T I_alfa, I_beta;
real_T Id, Iq, Id_dot, Iq_dot;
real_T omega_m, omega_m_dot, theta_m;
real_T theta_e, theta_e_dot;
real_T Te, TL;

//Parameter
real_T N = 4;
real_T flux = 0.175;
real_T Lsd = 0.0025;
real_T Lsq = 0.0075;
real_T Rs = 2.875;
real_T J = 0.0008;
real_T pi = 3.141592654;
real_T c = 0.816497;
real_T x = 0.866025;

//Input
Va = U(0);
Vb = U(1);
Vc = U(2);

//Variabel State
omega_m = X[0];
theta_e = X[1];
Id = X[2];
Iq = X[3];

//Transformasi 3 Fasa ke Alfa-Beta
V_alfa = c*(Va -0.5*Vb -0.5*Vc);
V_beta = c*(x*Vb - x*Vc);

//Transformasi Alfa-Beta ke DQ
Vd = (V_alfa*cos(theta_e)) + (V_beta*sin(theta_e));
Vq = (-V_alfa*sin(theta_e)) + (V_beta*cos(theta_e));

//Transformasi Arus DQ ke Alfa-Beta
I_alfa = Id*cos(theta_e) - Iq*sin(theta_e);
I_beta = Id*sin(theta_e) + Iq*cos(theta_e);

//Transformasi Alfa-Beta ke 3 Fasa
Ia = c*I_alfa;
Ib = c*(-0.5*I_alfa + x*I_beta);
Ic = c*(-0.5*I_alfa - x*I_beta);

//Model Mekanikal
Te = N*(flux + Id*(Lsd - Lsq))*Iq;
theta_m = theta_e / N;

//Grafik segitiga Theta e
while(theta_e > 2*pi){
theta_e -= 2*pi; }
while(theta_e < -2*pi){
theta_e += 2*pi; }

//Output
Y[0] = Ia;
Y[1] = Ib;
Y[2] = Ic;
Y[3] = Te;
Y[4] = N*omega_m;
Y[5] = theta_e;
Y[6] = Id;
Y[7] = Iq;
Y[8] = Vd;
Y[9] = Vq;

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
real_T omega_m, omega_m_dot, theta_m;
real_T theta_e, theta_e_dot;
real_T Te, TL;

//Parameter
real_T N = 4;
real_T flux = 0.175;
real_T Lsd = 0.0025;
real_T Lsq = 0.0075;
real_T Rs = 2.875;
real_T J = 0.0008;
real_T pi = 3.141592654;
real_T c = 0.816497; //nilai desimal dari akar dari 2/3
real_T x = 0.866025; //nilai desimal dari akar 3 / 2

//input
Va = U(0);
Vb = U(1);
Vc = U(2);
TL = U(3);

//Variabel State
omega_m = X[0];
theta_e = X[1];
Id = X[2];
Iq = X[3];

//Model Mekanik
Te = N*(flux + Id*(Lsd - Lsq))*Iq;
omega_m_dot = (Te - TL)/J;
theta_e_dot = N*omega_m;

//Transformasi 3 fasa ke Alfa-Beta
V_alfa = c*(Va -0.5*Vb -0.5*Vc);
V_beta = c*(x*Vb - x*Vc);

//Transformasi Alfa-Beta Ke DQ
Vd = (V_alfa*cos(theta_e)) + (V_beta*sin(theta_e));
Vq = (-V_alfa*sin(theta_e)) + (V_beta*cos(theta_e));

//Persamaan State-Space
Id_dot = (1/Lsd)*(-Rs*Id + N*omega_m*Lsq*Iq +Vd);
Iq_dot = (1/Lsq)*(-Rs*Iq - N*omega_m*(Lsd*Id + flux) + Vq);

//Turunan State
dX[0] = omega_m_dot;
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