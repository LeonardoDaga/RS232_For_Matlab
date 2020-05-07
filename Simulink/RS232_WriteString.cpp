/*
 * File : RS232_WriteString.c
 */

#define S_FUNCTION_NAME  RS232_WriteString
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	1
#define WRITESTRING		ssGetSFcnParam(S,0)

#define NO_I_WORKS		1
#define NO_P_WORKS      1

// LOCAL VARIABLES
static char_T msg[256];
static int hPort;  // Local because in others module may be different
static int rsStarted; // Status variable trasmitted via output
static char* writeString = NULL;

/*****************************************************************************
+FUNCTION: mdlInitializeSizes

+DESCRIPTION: Setup sizes of the various vectors.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

	if (!ssSetNumOutputPorts(S, 2)) return;
	if (!ssSetNumInputPorts(S, 2)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port one (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port one (index 0) */
	ssSetInputPortWidth(S, 0, 2); /* Width of output port one (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of output port one (index 0) */
	ssSetInputPortDataType(S, 0, 6); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, 8); /* Type of input PORT 2 (index 0) */
	ssSetOutputPortDataType(S, 0, 6); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, 8); /* Type of output PORT 2 (index 0) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);

	ssSetNumSampleTimes(S, 0);

	ssSetNumIWork(S, NO_I_WORKS);   /* number of integer work vector elements*/
	ssSetNumPWork(S, NO_P_WORKS);   /* number of pointer work vector elements*/
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
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
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
	int i_Step = 0;
	int len = 0;
	char c_Message[256];

	len = mxGetN(WRITESTRING);
	ssSetIWorkValue(S,0,len); // Set the first element

	writeString=(char *)malloc(len+10);
	if (writeString==NULL) 
	{
		printf("Error allocating write string memory\n");
		sprintf(c_Message, "Error allocating write string memory\n");
		ssSetErrorStatus(S,c_Message);
		return;
	}
	else
	{
		mxGetString(WRITESTRING, writeString, len+1);
		writeString[len+1] = 0;
		ssSetPWorkValue(S,0,(void *)writeString);
	}
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
	int	*y0		= (int*)ssGetOutputPortSignal(S,0);
	int	*y1		= (int*)ssGetOutputPortSignal(S,1);
	int	**u0	= (int**)ssGetInputPortSignalPtrs(S,0); 
	int	**u1	= (int**)ssGetInputPortSignalPtrs(S,1); 
	int		i_writeLen;
	
	Addr64 hPort = *u0;
	rsStarted = (int)(*u1[0]);

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = rsStarted;

	// Check handle validity
	if (hPort.addr == 0)
		return;

	// If this condition is not valid, the previous block (a read or setup block)
	// doesn't completed its work. Then skip the actual operation
	if (!rsStarted)
	{
		// Not allowed to do anything
		return;
	}

	// Get the synch string length
	i_writeLen = ssGetIWorkValue(S,0); 
	writeString = (char*)ssGetPWorkValue(S,0);

	RS232WriteL(hPort, writeString, i_writeLen);

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = rsStarted;
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
