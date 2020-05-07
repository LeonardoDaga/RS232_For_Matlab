/*
* File : RS232_Write.c
*/

#define S_FUNCTION_NAME  RS232_Write_Format
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	2
#define FORMAT_SEND_ARG        ssGetSFcnParam(S,0)
#define SAMP_TIME_ARG          ssGetSFcnParam(S,1)

#define NO_I_WORKS		1
#define NO_P_WORKS		1

#define TMP_SIZE			128

// LOCAL VARIABLES
static char_T msg[256];

static int datacount(char *formatstring);
static void build(char *formatstring,real_T	**data, Addr64 hPort);

/*****************************************************************************
+FUNCTION: mdlInitializeSizes

+DESCRIPTION: Setup sizes of the various vectors.

+PARAMETERS: 
SimStruct *S

 +RETURN: static void 
*******************************************************************************/
static void mdlInitializeSizes(SimStruct *S)
{
	char_T *buffer;
	
	
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
	
	buffer=(char *)calloc(mxGetN(FORMAT_SEND_ARG)+1,sizeof(char));
	if (buffer==NULL) 
	{
		printf("Error in RS232_Write_Format: could not allocate memory\n");
		return;
	}

	mxGetString(FORMAT_SEND_ARG, buffer, mxGetN(FORMAT_SEND_ARG)+1);
	
	// Input port
	int numsize = datacount(buffer);

	if (numsize > 0)
	{
		if (!ssSetNumInputPorts(S, 3)) return;
		ssSetInputPortWidth(S, 2, numsize); // Width of input port 3 (index 2) 
		ssSetInputPortDirectFeedThrough(S, 2, 1);
	}
	else
	{
		if (!ssSetNumInputPorts(S, 2)) return;
	}

	ssSetInputPortWidth(S, 0, 2); /* Width of input port 1 (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of input port 2 (index 1) */


	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, SS_INT32); /* Type of input PORT 2 (index 0) */

	// Output port
	if (!ssSetNumOutputPorts(S, 2)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port one (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port one (index 0) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */
	
	ssSetNumSampleTimes(S, 1);
	
	ssSetNumIWork(S, NO_I_WORKS);   /* number of integer work vector elements*/
	ssSetNumPWork(S, NO_P_WORKS);   /* number of pointer work vector elements*/

	free(buffer);
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
+FUNCTION: mdlInitializeConditions

+DESCRIPTION: Routine used to initialize data

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
	char *buffer;

	buffer=(char *)calloc(mxGetN(FORMAT_SEND_ARG)+5,sizeof(char));
	if (buffer==NULL) 
	{
		printf("Error in RS232_Write_Format: could not allocate memory\n");
		return;
	}

	mxGetString(FORMAT_SEND_ARG, buffer, mxGetN(FORMAT_SEND_ARG)+1);
	
	ssSetPWorkValue(S,0,(void *)buffer);
}


/*****************************************************************************
+FUNCTION: mdlSetInputPortWidth
*******************************************************************************/
#define MDL_SET_INPUT_PORT_WIDTH
static void mdlSetInputPortWidth(SimStruct *S, int portIndex, int inWidth)
{
	ssSetInputPortWidth(S, portIndex, inWidth);
}

/*****************************************************************************
+FUNCTION: mdlSetOutputPortWidth
*******************************************************************************/
#define MDL_SET_OUTPUT_PORT_WIDTH
static void mdlSetOutputPortWidth(SimStruct *S, int portIndex, int outWidth)
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
	int		*y0	= (int*)ssGetOutputPortSignal(S,0);
	int		*y1	= (int*)ssGetOutputPortSignal(S,1);
	int		**u0	= (int**)ssGetInputPortSignalPtrs(S,0); 
	int		**u1	= (int**)ssGetInputPortSignalPtrs(S,1); 
	real_T	**u2	= (real_T**)ssGetInputPortSignalPtrs(S,2); 
	int		NUMOFWRITE = 	ssGetInputPortWidth(S,1);
	char		*formatsend;
	int		rsStarted; // Status variable trasmitted via output
	
	Addr64 hPort = *u0;
	rsStarted = (int)(*u1[0]);
	
	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = rsStarted;
	
	// Check handle validity
	if (hPort.addr == 0)
		return;
	
	// If this condition is not valid, the previous block (a read or setup block)
	// didn't completed its work. Then skip the actual operation
	if (!rsStarted)
	{
		//printf("no send \n");
		return;
	}
	else
	{
		//printf("send: rsStarted = %d, %d \n", hPort, rsStarted);
	}

	formatsend=(char *)ssGetPWorkValue(S,0);

	build(formatsend, u2, hPort);

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
	char *buffer;
	
	buffer=(char *)ssGetPWorkValue(S,0);
	
	if (buffer!=NULL) 
	{
		free(buffer);
	}
}

/*****************************************************************************
+FUNCTION: datacount
*******************************************************************************/
static int datacount(char *formatstring)
{

	unsigned int i;
	int n;

	n=0;
	for (i=0;i<strlen(formatstring);i++) {
		if (formatstring[i]=='%') n++;
	}
	return(n);
}

/*****************************************************************************
+FUNCTION: build
*******************************************************************************/
static void build(char *formatstring,real_T	**data, Addr64 hPort)
{

	int i, found, k;
	unsigned int fi, j;
	char tmp[TMP_SIZE];
	char token[TMP_SIZE];
	
	i=0;
	fi=0;
	while (fi<strlen(formatstring)) 
	{
		if (formatstring[fi]=='\\') 
		{
			switch (formatstring[fi+1])
			{
			case 'r': sprintf(tmp,"\r"); break;
			case 'n': sprintf(tmp,"\n"); break;
			case 't': sprintf(tmp,"\t"); break;
			case '\\': sprintf(tmp,"\\"); break;
			default:
				strncpy(token,formatstring+fi,2);
				token[2]='\0';
				sprintf(tmp,(const char*)token);
				break;
			}

			RS232WriteL(hPort, (char *)tmp, strlen(tmp));
			fi += 2;
		}
		else if (formatstring[fi]=='%') 
		{
			// search format token
			found=0;
			j=fi;
			k=0;
			while (!found) 
			{
				switch (formatstring[j]) 
				{
	         case 'x':
					strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
					sprintf(tmp,token,(int)*data[i]);
					RS232WriteL(hPort, (char *)tmp, strlen(tmp));
					found=1;
					break;
				case 'd':
					strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
					sprintf(tmp,token,(int)*data[i]);
					RS232WriteL(hPort, (char *)tmp, strlen(tmp));
					found=1;
					break;
				case 'u':
					strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
					sprintf(tmp,token,(int)*data[i]);
					RS232WriteL(hPort, (char *)tmp, strlen(tmp));
					found=1;
					break;
				case 'f':
					strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
					sprintf(tmp,token,*data[i]);
					RS232WriteL(hPort, (char *)tmp, strlen(tmp));
					found=1;
					break;
				case 'e':
					strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
					sprintf(tmp,token,*data[i]);
					RS232WriteL(hPort, (char *)tmp, strlen(tmp));
					found=1;
					break;
				case 'g':
					strncpy(token,formatstring+fi,k+1);
					token[k+1]='\0';
					sprintf(tmp,token,*data[i]);
					RS232WriteL(hPort, (char *)tmp, strlen(tmp));
					found=1;
					break;
				default:
					//mexPrintf("in\n");
					k++;
					j++;
					if (j>strlen(formatstring)-1) 
					{
						printf("error in format string\n");
					}
					break;
				}
			}
			
			// set new fi
			fi=j+1;
			i++;
			
		} 
		else 
		{
			RS232WriteL(hPort, (char*)&formatstring[fi], 1);
       	fi++;
		}
		
	}
	
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
