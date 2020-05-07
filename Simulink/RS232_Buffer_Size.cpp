/*
 * File : RS232_Buffer_Size.c
 */

#define S_FUNCTION_NAME  RS232_Buffer_Size
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include <ctype.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	1
#define SAMP_TIME_ARG				ssGetSFcnParam(S,0)

#define NO_P_WORKS              	0
#define NO_I_WORKS					0

static char_T msg[256];

/*****************************************************************************
+FUNCTION: mdlInitializeSizes

+DESCRIPTION: Setup sizes of the various vectors.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
static void mdlInitializeSizes(SimStruct *S)
{
#ifdef MATLAB_MEX_FILE
	ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		sprintf(msg, "Error in RS232_Buffer_Size: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif

	// Input port
	if (!ssSetNumInputPorts(S, 1)) return;
	ssSetInputPortWidth(S, 0, 2); /* Width of input port 1 (index 0) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	
	// Output port
	if (!ssSetNumOutputPorts(S, 1)) return;
	ssSetOutputPortWidth(S, 0, 3); /* Width of output port 1 (index 0) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	
	ssSetNumSampleTimes(S, 0);
	
	ssSetNumPWork(S, NO_P_WORKS);   /* number of pointer work vector elements*/
	ssSetNumIWork(S, NO_I_WORKS);   /* number of integer work vector elements*/
}

/*****************************************************************************
+FUNCTION: mdlInitializeSampleTimes

+DESCRIPTION: Specifiy that we inherit our sample time from the driving block.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, mxGetPr(SAMP_TIME_ARG)[0]);
	
	if (mxGetN((SAMP_TIME_ARG))==1) 
	{
		ssSetOffsetTime(S, 0, 0.0);
	} 
	else 
	{
		ssSetOffsetTime(S, 0, mxGetPr(SAMP_TIME_ARG)[1]);
	}
}
 
/*****************************************************************************
+FUNCTION: mdlStart

+DESCRIPTION: Routine used to initialize data

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
#define MDL_START  /* Change to #undef to remove function */
static void mdlStart(SimStruct *S)
{
}


/*****************************************************************************
+FUNCTION: mdlOutputs

+DESCRIPTION: 

+PARAMETERS: 
SimStruct *S
int_T tid

+RETURN: static void 
*******************************************************************************/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	int		*y	= (int*)ssGetOutputPortSignal(S,0);
	int		**u0	= (int**)ssGetInputPortSignalPtrs(S,0); 

	T_Buffer* buff = (T_Buffer*)Addr64(*u0).addr;

	if (buff != NULL)
	{
		y[0] = buff->i_Fill;
		y[1] = buff->i_Size;
		y[2] = buff->i_Read;
	}
	else
	{
		y[0] = -1;
		y[1] = -1;
		y[2] = -1;
	}
}

/*****************************************************************************
+FUNCTION: mdlTerminate

+DESCRIPTION: No termination needed, but we are required to have this routine.
	Function to perform housekeeping at execution termination

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
static void mdlTerminate(SimStruct *S)
{
}


#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
