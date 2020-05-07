/*
 * File : RS232_Read.c
 */

#define S_FUNCTION_NAME  RS232_SynchB_Buffer
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include <ctype.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	5
#define SYNCHSIZE			        	(int)(*mxGetPr(ssGetSFcnParam(S,0)))
#define MASK							ssGetSFcnParam(S,1)
#define VALUE							ssGetSFcnParam(S,2)
#define SAMP_TIME_ARG				ssGetSFcnParam(S,3)
#define STEP_ALLOC					(int)(*mxGetPr(ssGetSFcnParam(S,4)))

#define TMPSIZE			128

#define NO_P_WORKS              	2
#define NO_I_WORKS					3

static char_T msg[256];

static void ResizeBuffer(SimStruct *S, T_Buffer* buffer, int i_NeedSize);

/*****************************************************************************
+FUNCTION: GetHexParam
/****************************************************************************/
int GetHexParam(const struct mxArray_tag * vParam, int* i_Value)
{
	char* c_String=(char *)calloc(mxGetN(vParam)+1,sizeof(char));
	if (c_String==NULL) 
	{
		printf("Error in RS232_CheckB_Buffer (mdlStart.1) : could not allocate memory\n");
		return 0;
	}

	mxGetString(vParam, 
				   c_String, 
				(int)mxGetN(vParam)+1);

	int iscan = sscanf(c_String, "%x", i_Value);
	free(c_String);

	return iscan;
}

/*****************************************************************************
+FUNCTION: GetHexParam
/****************************************************************************/
bool checkSynch(char* c_P, int i_Mask, int i_Value, int i_SyncSize)
{
	int val = 0;
	char* c_val = (char*)&val;

	for (int i=0; i<i_SyncSize; i++)
	{
		c_val[i] = c_P[i_SyncSize - i - 1];
	}

	return ((val & i_Mask) == i_Value);
}

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
		sprintf(msg, "Error in RS232_SynchB_Buffer: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif

	if (STEP_ALLOC <= 16) 
	{
		sprintf(msg, "Error in RS232_SynchB_Buffer: \n");
		sprintf(&msg[strlen(msg)],
				 "Allocation step too small: %d.\n",
				 STEP_ALLOC);
		ssSetErrorStatus(S,msg);
		return;
	}
	
	// Input port
	if (!ssSetNumInputPorts(S, 3)) return;
	ssSetInputPortWidth(S, 0, 1); /* Width of input port 1 (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of input port 2 (index 1) */
	ssSetInputPortWidth(S, 2, 1); /* Width of input port 3 (index 2) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, SS_INT32); /* Type of input PORT 2 (index 0) */
	ssSetInputPortDataType(S, 2, SS_POINTER); /* Type of input PORT 3 (index 0) */
	
	// Output port
	if (!ssSetNumOutputPorts(S, 2)) return;
	ssSetOutputPortWidth(S, 0, 1); /* Width of output port 1 (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port 2 (index 1) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */
	
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
	int i_rdStat = 0;
	ssSetIWorkValue(S,0,i_rdStat); // Set the first element

	int i_Mask = 0;
	int i_Value = 0;

	if (!GetHexParam(MASK, &i_Mask))
	{
		printf("Error in RS232_CheckB_Buffer (mdlStart.1) : Mask parameter is not valid\n");
		return;
	}

	if (!GetHexParam(VALUE, &i_Value))
	{
		printf("Error in RS232_CheckB_Buffer (mdlStart.2) : Value parameter is not valid\n");
		return;
	}

	ssSetIWorkValue(S,1,i_Mask);
	ssSetIWorkValue(S,2,i_Value);
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
	int		**u2	= (int**)ssGetInputPortSignalPtrs(S,2); 
	int		i_len;
	int		i_outStat = 0;

	Addr64 hPort = *u0;
	int i_inStat = (int)(*u1[0]);
	T_Buffer* buff = (T_Buffer*)Addr64(*u2).addr;

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = i_outStat;

	// Synch handle validity
	if (hPort.addr == 0)
	{
		printf("RS232 Synch Binary Buffer error: choosen COM-port not initialized\n");
		return;
	}

	// Get the previous internal read status
	int i_rdStat = ssGetIWorkValue(S,0);

	// If this condition is valid, the previous block (a read or setup block)
	// completed its work or this block, after activated, didn't received 
	// all the expected character. Then skip the actual operation.
	if ((!i_inStat)&&(!i_rdStat))
		return;

	// Synching the actual length of the queue
	i_len = RS232InQueue(hPort);
	// i_len = 128;
	// printf( "Queue length: %d\n" , i_len);

	ResizeBuffer(S, buff, i_len + buff->i_Fill);

	if (buff->c_Buffer == NULL)
	{
		printf("RS232 Synch Binary Buffer error: error managing buffers\n");
		buff->i_Fill = 0;
		i_rdStat = 0;
		ssSetIWorkValue(S,0,i_rdStat);
		return;
	}

	if (i_len != 0)
	{
		/*
		FILE* stream;

		if( (stream  = fopen( "clust.dat", "r" )) == NULL )
		{
			printf( "The file 'data' was not opened\n" );
			return;
		}
		else
		{
			printf( "The file 'data' was opened\n" );
		}

		fread(&buff->c_Buffer[buff->i_Fill], sizeof(char), i_len, stream);

		fclose(stream);

		*/

		RS232Read(hPort, &buff->c_Buffer[buff->i_Fill], i_len);
		buff->i_Read += i_len;
	}
	
	buff->i_Fill += i_len;
	buff->c_Buffer[buff->i_Fill]=0; // Null-terminate the buffer

	int i_SyncSize = SYNCHSIZE;
	// printf("SyncSize = %d\n", i_SyncSize);

	// If the total message is greater then the message terminator, look for the 
	// terminator
	if (buff->i_Fill >= i_SyncSize) 
	{
		int i_Mask = ssGetIWorkValue(S,1);
		int i_Value = ssGetIWorkValue(S,2);
		char*		c_P = buff->c_Buffer;
		char*		c_End = buff->c_Buffer + buff->i_Fill - SYNCHSIZE;

		//printf("totSize = %d\n", (int)totSize);
		//printf("c_P = %x\n", (int)c_P);

		while (c_P < c_End)
		{
			if (checkSynch(c_P, i_Mask, i_Value, i_SyncSize))
			{
				// if synchro found
				i_outStat = 1;  // Read done
				i_rdStat = 0;  // End looking for a synch
				ssSetIWorkValue(S,0,i_rdStat);
				break;
			}

			c_P++;
		}

		int totSize = c_P - buff->c_Buffer;

		// Calculate the remaining qty of chars and
		// update the buff->i_Fill: 
		buff->i_Fill = buff->i_Fill - totSize;

		memmove(buff->c_Buffer, // Start of the buffer
				  buff->c_Buffer + totSize, // The end of last message
				  buff->i_Fill); // the remaining qty of chars

		buff->c_Buffer[buff->i_Fill]=0; // Null-terminate the buffer

	}
	else
	{
		i_outStat = 0;  // Read not done
		//printf("readb buf short: i_outStat = %d, %d, |%s| \n", hPort, i_outStat, buff->c_Buffer);
		i_rdStat = 1;  // Still looking for a synch
		ssSetIWorkValue(S,0,i_rdStat);
	}
	
	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = i_outStat;
}

/*****************************************************************************
+FUNCTION: ResizeBuffer
*******************************************************************************/
static void ResizeBuffer(SimStruct *S, T_Buffer* buffer, int i_NeedSize)
{
	if (buffer == NULL)
		return;

	if (i_NeedSize >= buffer->i_Size)
	{
		if (buffer->c_Buffer == NULL)
		{
			buffer->c_Buffer = (char*)malloc(STEP_ALLOC);
			buffer->i_Size = STEP_ALLOC;
			printf("Allocated memory\n");

			if (buffer->c_Buffer == NULL)
			{
				printf("RS232 Synch Binary Buffer error: could not allocate memory\n");
				buffer->i_Size = 0;
			}
		}
		else
		{
			int newSize = buffer->i_Size;

			while (newSize <= i_NeedSize)
			{
				newSize += STEP_ALLOC;
			}

			buffer->c_Buffer = (char*)realloc(buffer->c_Buffer, newSize);
			buffer->i_Size = newSize;

			printf("Required size is %d\n", i_NeedSize);
			printf("Allocated size is %d\n", buffer->i_Size);
			if (buffer->c_Buffer == NULL)
			{
				printf("RS232 Synch Binary Buffer error: could not allocate memory\n");
				buffer->i_Size = 0;
			}
		}
	}
	else if ((i_NeedSize < buffer->i_Size - STEP_ALLOC)&&(i_NeedSize > 0))
	{
		buffer->c_Buffer = (char*)realloc(buffer->c_Buffer, buffer->i_Size - STEP_ALLOC);
		buffer->i_Size -= STEP_ALLOC;

		printf("Required size is %d\n", i_NeedSize);
		printf("Reduced size is %d\n", buffer->i_Size);

		if (buffer->c_Buffer == NULL)
		{
			printf("RS232 Synch Binary Buffer error: could not allocate memory\n");
			buffer->i_Size = 0;
		}
	}
	else 
	{
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
