/*
 * File : RS232_Setup.cpp
 */

#define S_FUNCTION_NAME  RS232_Setup
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	5
#define PORT		(int)(*mxGetPr(ssGetSFcnParam(S,0)))
#define BAUDRATE	(int)(*mxGetPr(ssGetSFcnParam(S,1)))
#define BYTESIZE	(int)(*mxGetPr(ssGetSFcnParam(S,2)))
#define STOPBITS	(int)(*mxGetPr(ssGetSFcnParam(S,3)))
#define PARITY		(int)(*mxGetPr(ssGetSFcnParam(S,4)))

#define LEN_QUEUE			256
#define NO_I_WORKS		1
#define NO_P_WORKS		1

// LOCAL VARIABLES
static char_T msg[256];
static int timeout = 0; // Visibile solo internamente a questo file
static int i_DataPackets = 0;

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
	if (!ssSetNumInputPorts(S, 0)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port one (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port one (index 0) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */

   ssSetNumSampleTimes(S, 0);

   ssSetNumPWork(S, NO_P_WORKS);   /* number of pointer work vector elements*/
   ssSetNumIWork(S, NO_I_WORKS);   /* number of integer work vector elements*/
}


/*****************************************************************************
+FUNCTION: Initialize

+DESCRIPTION: Serial port initialization

+PARAMETERS: 
SimStruct *S

+RETURN: void 
*******************************************************************************/
Addr64 RS232Initialize(SimStruct *S)
{
	char c_Message[256];
	int baudrates[]={230400,115200,57600,38400,19200,9600,4800,2400,1200,300,110,230400,460800,921600};
	int bytesize[]={0,8,7,6,5};
	int stopbits[]={0,1,2};
	int parity[]={0,0,1,2};
	Addr64 hPort;

	////////////////////////////////////////////////////////
	// Open and configure serial port
	hPort = RS232OpenC(PORT, baudrates[BAUDRATE], bytesize[BYTESIZE],
					parity[PARITY], stopbits[STOPBITS]); 
	printf("RS232 port config:%d,%d,%d,%d,%d: hPort = [%d,%d]\n",
					PORT, baudrates[BAUDRATE], bytesize[BYTESIZE],
					parity[PARITY], stopbits[STOPBITS], hPort.iaddr[0], hPort.iaddr[1]);
	if (hPort.addr == 0) 
	{
		sprintf(c_Message, "RS232 port open Error: choosen COM-port |%d| not initialized\n",(int)PORT);
		ssSetErrorStatus(S,c_Message);
		printf(" RS232 port open Error: choosen COM-port |%d| not initialized\n",PORT);
		RS232Close(hPort);
		return hPort;
	}

	printf(" RS232 Initialized\n");

	return hPort;
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
	Addr64 hPort;
	int RS232_Started = 0;

	ssSetPWorkValue(S,0,hPort.addr); 
	ssSetIWorkValue(S,0,RS232_Started); 
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
	int  *y1     = (int*)ssGetOutputPortSignal(S,1);

	Addr64 hPort	 = ssGetPWorkValue(S,0);
	int	RS232_Started = ssGetIWorkValue(S,0); 

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = RS232_Started;

	// Start = 0 se il sistema è ancora da inizializzare
	if (RS232_Started==0) 
	{
		// If not initalized, initialize the RS232 port
		if (hPort.addr == 0) 
		{
			printf(" Initializing\n");
			hPort = RS232Initialize(S);

			ssSetPWorkValue(S,0,hPort.addr);
			
			// Re-test after initialization
			if (hPort.addr == 0)
			{
				printf("Step 0: RS232 I/O-receive: choosen COM-port |%d| not initialized\n",PORT);
				return;
			}
			else
			{
				printf("Step 0: RS232 successfully initialized: choosen COM-port |%d|\n",PORT);
				RS232_Started = 1;
				ssSetIWorkValue(S,0,RS232_Started);
			}
		}
	}

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = RS232_Started;
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
	Addr64 hPort = ssGetPWorkValue(S, 0);

	printf("RS232_Close. %d, [%d,%d]\n", PORT, hPort.iaddr[0], hPort.iaddr[1]);
	RS232Close(hPort);
	hPort.addr = 0;
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
