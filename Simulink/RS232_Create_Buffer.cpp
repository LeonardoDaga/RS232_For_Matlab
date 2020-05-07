/*
 * File : RS232_Setup.cpp
 */

#define S_FUNCTION_NAME  RS232_Create_Buffer
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	2
#define BUFFER_MIN_SIZE		(int)(*mxGetPr(ssGetSFcnParam(S,0)))
#define SAMP_TIME_ARG          ssGetSFcnParam(S,1)

#define NO_P_WORKS		1

// LOCAL VARIABLES
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
		sprintf(msg, "Error in RS232_Write_Format: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif

	if (!ssSetNumOutputPorts(S, 1)) return;
	if (!ssSetNumInputPorts(S, 0)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port one (index 0) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */

	ssSetNumSampleTimes(S, 1);

	ssSetNumPWork(S, NO_P_WORKS);   /* number of integer work vector elements*/
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
	T_Buffer* buffer = new T_Buffer;
	buffer->c_Buffer = (char*)malloc(BUFFER_MIN_SIZE);
	buffer->i_Size = BUFFER_MIN_SIZE;
	buffer->i_Read = 0;
	buffer->i_Fill = 0;

	printf("Buffer allocated: Pt=%llu, Size=%d \n", (UINT64)buffer->c_Buffer, 
															(int)buffer->i_Size);

	ssSetPWorkValue(S, 0, buffer);
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
	int  *y0     = (int*)ssGetOutputPortSignal(S,0);

	Addr64	buffer	 = (Addr64)ssGetPWorkValue(S,0); 

	y0[0] = buffer.iaddr[0];
	y0[1] = buffer.iaddr[1];
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
	T_Buffer* buff	 = (T_Buffer*)ssGetPWorkValue(S,0); 

	if (buff != NULL)
	{
		printf("Buffer freeing: Pt=%llu, Size=%d\n", (Addr64)buff->c_Buffer, 
																(int)buff->i_Size);
		if (buff->c_Buffer != NULL)
			free(buff->c_Buffer);
		delete buff;

		printf("Buffer freed\n");
	}
	buff = NULL;
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
