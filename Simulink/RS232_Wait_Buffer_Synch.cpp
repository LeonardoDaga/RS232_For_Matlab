/*
 * File : RS232_Read.c
 */

#define S_FUNCTION_NAME  RS232_Wait_Buffer_Synch
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	3
#define SYNCH_ARG        			ssGetSFcnParam(S,0)
#define SAMP_TIME_ARG				ssGetSFcnParam(S,1)
#define STEP_ALLOC					(int)(*mxGetPr(ssGetSFcnParam(S,2)))

#define TMPSIZE			128
#define TOKEN_SEP			", "

#define NO_P_WORKS      2
#define NO_I_WORKS		3

static char_T msg[256];

static void ResizeBuffer(SimStruct *S, T_Buffer* buffer, int i_NeedSize);
static void tokcnt(char* string, char* seps, int* nTok, unsigned int* nMaxLen);
static int strfirsttok(char* buff, char* item, int len, int num);

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
		sprintf(msg, "Error in RS232_Wait_Buffer_Synch: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif

	int i_NumItem;
	unsigned int i_LenItem;
	char* syncharg = (char *)calloc(mxGetN(SYNCH_ARG)+1,sizeof(char));
	if (syncharg==NULL) 
	{
		printf("RS232 Read Format error: could not allocate memory\n");
		return;
	}
	
	mxGetString(SYNCH_ARG, syncharg, (int)mxGetN(SYNCH_ARG)+1);

	tokcnt(syncharg, TOKEN_SEP, &i_NumItem, &i_LenItem);

	free(syncharg);
	
	// Input port
	if (!ssSetNumInputPorts(S, 3)) return;
	ssSetInputPortWidth(S, 0, 2); /* Width of input port 1 (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of input port 2 (index 1) */
	ssSetInputPortWidth(S, 2, 2); /* Width of input port 3 (index 2) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, SS_INT32); /* Type of input PORT 2 (index 0) */
	ssSetInputPortDataType(S, 2, SS_INT32); /* Type of input PORT 3 (index 0) */
	
	// Output port
	if (!ssSetNumOutputPorts(S, 2)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port 1 (index 0) */
	ssSetOutputPortWidth(S, 1, i_NumItem); /* Width of output port 2 (index 1) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */
	
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
	int i_chkStat = 0;
	ssSetIWorkValue(S,0,i_chkStat); // Set the first element

	char* syncharg = (char *)calloc(mxGetN(SYNCH_ARG)+1,sizeof(char));
	if (syncharg==NULL) 
	{
		printf("RS232 Read Format error: could not allocate memory\n");
		return;
	}
	
	mxGetString(SYNCH_ARG, syncharg, (int)(mxGetN(SYNCH_ARG)+1));
	ssSetPWorkValue(S,0,(void *)syncharg);

	int i_NumItem = 0;
	unsigned int ui_LenItem = 0;
	char* c_Item = NULL;
	char* c_copysyncharg = (char*)malloc(strlen(syncharg)+1);
	strcpy(c_copysyncharg, syncharg);

	tokcnt(c_copysyncharg, TOKEN_SEP, &i_NumItem, &ui_LenItem);

	c_Item = (char*)malloc(i_NumItem * (ui_LenItem + 1));
	char* c_Temp = c_Item;

	strcpy(c_copysyncharg, syncharg);
   char* token = strtok( c_copysyncharg, TOKEN_SEP );
   while( token != NULL )
   {
		strcpy(c_Temp, token); 
		c_Temp = c_Temp + ui_LenItem + 1;
      token = strtok( NULL, TOKEN_SEP );
   }

	ssSetPWorkValue(S,1,(void *)c_Item);
	ssSetIWorkValue(S,1,(int)ui_LenItem);
	ssSetIWorkValue(S,2,i_NumItem);

	free(c_copysyncharg);
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
	char		*syncharg;
	int		i_outStat = 0, i_chkStat;
	
	Addr64 hPort = *u0;
	int i_inStat = (int)(*u1[0]);
	T_Buffer* buff = (T_Buffer*)Addr64(*u2).addr;
	int i_LenItem = ssGetIWorkValue(S,1);
	int i_NumItem = ssGetIWorkValue(S,2);

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];

	for (int i=0; i<i_NumItem; i++)
	{
		y1[i] = 0;
	}
	
	// Check handle validity
	if (hPort.addr == 0)
	{
		printf("RS232 Read Format error: choosen COM-port not initialized\n");
		return;
	}

	// Get the previous internal check status
	i_chkStat = ssGetIWorkValue(S,0);

	// If this condition is valid, the previous block (a read or setup block)
	// completed its work or this block, after activated, didn't received 
	// all the expected character. Then skip the actual operation.
	if ((!i_inStat)&&(!i_chkStat))
		return;

	// Retrieve the work values for pointers and the actual buffer size
	syncharg = (char *)ssGetPWorkValue(S,0);
	
	// Checking the actual length of the queue
	i_len = RS232InQueue(hPort);

	ResizeBuffer(S, buff, i_len + buff->i_Fill);

	if (buff->c_Buffer == NULL)
	{
		printf("RS232 Read Format error: error managing buffers\n");
		buff->i_Fill = 0;
		i_chkStat = 0;
		ssSetIWorkValue(S,0,i_chkStat);
		return;
	}

	if (i_len != 0)
	{
		RS232Read(hPort, &buff->c_Buffer[buff->i_Fill], i_len);
		buff->i_Read += i_len;
	}
	
	buff->i_Fill += i_len;
	buff->c_Buffer[buff->i_Fill]=0; // Null-terminate the buffer

	// If the total message is greater then the message terminator, look for the 
	// terminator
	if (buff->i_Fill>=sizeof(i_LenItem)) 
	{
		char* c_Item = (char*)ssGetPWorkValue(S,1);

		// Look for the message sentence terminator
		int msgSynch = strfirsttok(buff->c_Buffer, c_Item, i_LenItem+1, i_NumItem);

		if (msgSynch != -1)
		{
			i_outStat = 1;  // Synch done
			//printf("check: i_outStat = %d, %d, |%s| \n", hPort, i_outStat, buff->c_Buffer);
			i_chkStat = 0;  // End looking for a synch
			ssSetIWorkValue(S,0,i_chkStat);

			y1[msgSynch] = 1;
		}
		else
		{
			i_outStat = 0;  // Synch not done
			//printf("check: i_outStat = 0\n");
			i_chkStat = 1;  // Continue to look for a synch
			ssSetIWorkValue(S,0,i_chkStat);
		}
	}
	else
	{
		i_outStat = 0;  // Synch not done
		//printf("check: i_outStat = 0\n");
		i_chkStat = 1;  // Continue to look for a synch
		ssSetIWorkValue(S,0,i_chkStat);
	}

	/*
	printf("y1 = ");
	for (i=0; i<i_NumItem; i++)
	{
		printf("%d:%d,", i, y1[i]);
	}
	printf("\n");
	*/

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
				printf("RS232 Read Format error: could not allocate memory\n");
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
				printf("RS232 Read Format error: could not allocate memory\n");
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
			printf("RS232 Read Format error: could not allocate memory\n");
			buffer->i_Size = 0;
		}
	}
	else 
	{
		/*
		printf ("Space is enough: allocated (%d): %d, needed: %d\n", 
							(int)buffer->c_Buffer, 
							buffer->i_Size, 
							i_Size);
							*/
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
	char* syncharg=(char *)ssGetPWorkValue(S,0);
	
	if (syncharg!=NULL) {
		free(syncharg);
	}
	
	char* c_Item=(char *)ssGetPWorkValue(S,1);
	
	if (c_Item!=NULL) {
		free(c_Item);
	}
	
}

/*****************************************************************************
+FUNCTION: tokcnt
*******************************************************************************/
static void tokcnt(char* string, char* seps, int* nTok, unsigned int* nMaxLen)
{
	char* token = NULL;

	*nTok = 0;
	*nMaxLen = 0;

   /* Establish string and get the first token: */
   token = strtok( string, seps );
   while( token != NULL )
   {
		(*nTok)++;
		(*nMaxLen) = (*nMaxLen < strlen(token))?strlen(token):*nMaxLen;

      token = strtok( NULL, seps );
   }
}

/*****************************************************************************
+FUNCTION: strfirsttok
*******************************************************************************/
static int strfirsttok(char* buff, char* item, int len, int num)
{
	char* c_first = buff + strlen(buff);
	int   i_first = -1;

	for (int i=0; i<num; i++)
	{
		char* c_thisitem = strstr(buff, &item[len*i]);

		if (c_thisitem != NULL)
		{
			if (c_thisitem < c_first)
			{
				i_first = i;
				c_first = c_thisitem;
			}
		}
	}
	
	return i_first;
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
