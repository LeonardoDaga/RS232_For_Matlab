/*
 * File : RS232_Read.c
 */

#define S_FUNCTION_NAME  RS232_Write_Binary
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include <ctype.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	2
#define FORMAT_REC_ARG        	ssGetSFcnParam(S,0)
#define SAMP_TIME_ARG				ssGetSFcnParam(S,1)

#define TMPSIZE			128

#define NO_P_WORKS              	2
#define NO_I_WORKS					3

#define SEPS				","

static int typesize[9] = 
{
	8, // SS_DOUBLE
	4, // SS_SINGLE
	1, // SS_INT8
	1, // SS_UINT8
	2, // SS_INT16
	2, // SS_UINT16
	4, // SS_INT32
	4, // SS_UINT32
	1, // SS_BOOLEAN
};

typedef struct
{
	int		num;
	int		type;
} T_DataDescr;

static char_T msg[256];

static int extract(char *formatstring, char* head, char *in, double *data);
static int datacount(char *formatstring);
static void ResizeBuffer(SimStruct *S, T_Buffer* buffer, int i_NeedSize);
int translateFormat(T_DataDescr*	pDescr, char* formatrec, int* size);
int countFormatFields(char* formatrec);
int	writeSignal(void** u, char* buff, T_DataDescr* pDescr);

/*****************************************************************************
+FUNCTION: mdlInitializeSizes

+DESCRIPTION: Setup sizes of the various vectors.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
static void mdlInitializeSizes(SimStruct *S)
{
	char_T *formatrec;	
	
#ifdef MATLAB_MEX_FILE
	ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		sprintf(msg, "Error in RS232_Write_Binary: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif
	
	formatrec=(char *)calloc(mxGetN(FORMAT_REC_ARG)+1,sizeof(char));
	if (formatrec==NULL) 
	{
		printf("Error in RS232_Write_Binary (mdlInitializeSizes.1) : could not allocate memory\n");
		return;
	}

	mxGetString(FORMAT_REC_ARG, formatrec, mxGetN(FORMAT_REC_ARG)+1);

	T_DataDescr*	pDescr = NULL;
	int numForm = countFormatFields(formatrec);
	pDescr = (T_DataDescr*) malloc(numForm * sizeof(T_DataDescr));
	if (pDescr==NULL) 
	{
		printf("RS232 Write Binary (mdlInitializeSizes.2) error: could not allocate memory\n");
		return;
	}

	int totSize = 0;
	int nErr = translateFormat(pDescr, formatrec, &totSize);
	if (nErr != numForm)
	{
		free(pDescr);
		sprintf(msg, "Error in RS232_Write_Binary: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong format string.\n");
		ssSetErrorStatus(S,msg);
		return;
	}
	
	// Input port
	if (!ssSetNumInputPorts(S, 2 + numForm)) return;
	ssSetInputPortWidth(S, 0, 2); /* Width of input port 1 (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of input port 2 (index 1) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, SS_INT32); /* Type of input PORT 2 (index 0) */

	for (int i=0; i<numForm; i++)
	{
		ssSetInputPortDataType(S, 2 + i, pDescr[i].type); // Type of PORT 
		ssSetInputPortWidth(S, 2 + i, pDescr[i].num); // Width of port
		ssSetInputPortDirectFeedThrough(S, 2 + i, 1);
	}
	
	// Output port
	if (!ssSetNumOutputPorts(S, 2)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port 1 (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port 2 (index 1) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */
	
	ssSetNumSampleTimes(S, 0);
	
	ssSetNumPWork(S, NO_P_WORKS);   /* number of pointer work vector elements*/
	ssSetNumIWork(S, NO_I_WORKS);   /* number of integer work vector elements*/

	free(formatrec);
	free(pDescr);
}

/*****************************************************************************
+FUNCTION: countFormatFields
*******************************************************************************/
int countFormatFields(char* formatrec)
{
	int	count = 0;
	char * frmcopy = (char*)malloc(strlen(formatrec)+1);
	strcpy(frmcopy, formatrec);

   char* token = strtok( frmcopy, SEPS );

   while(token != NULL)
   {
      // While there are tokens in "string"
      count++;

      // Get next token: 
      token = strtok( NULL, SEPS );
   }

	free(frmcopy);
	return count;
}

/*****************************************************************************
+FUNCTION: translateFormat
*******************************************************************************/
int translateFormat(T_DataDescr*	pDescr, char* formatrec, int* size)
{
	int num = 0;
	int sz = 0;
	char * frmcopy = (char*)malloc(strlen(formatrec)+1);
	strcpy(frmcopy, formatrec);

   char* token = strtok( frmcopy, SEPS );

   while(token != NULL)
   {
		// get the first field
		if (isdigit(token[0]))
		{
			pDescr[num].num = atoi(&token[0]);
			while (isdigit(token[0]))
				token++;
		}
		else
		{
			pDescr[num].num = 1;
		}

		// translate type
		switch(token[0])
		{
		case 'd':
			switch(token[1])
			{
			case '4': pDescr[num].type = SS_SINGLE;  sz+=4*pDescr[num].num; break;
			case '8': pDescr[num].type = SS_DOUBLE;  sz+=8*pDescr[num].num; break;
			default: printf("Bad format on type double: %s\n", token); free(frmcopy); return -1;
			}
			break;
		case 'u':
			switch(token[1])
			{
			case '1': pDescr[num].type = SS_UINT8;  sz+=1*pDescr[num].num; break;
			case '2': pDescr[num].type = SS_UINT16;  sz+=2*pDescr[num].num; break;
			case '4': pDescr[num].type = SS_UINT32;  sz+=4*pDescr[num].num; break;
			default: printf("Bad format on type unsigned: %s\n", token); free(frmcopy); return -1;
			}
			break;
		case 'i':
			switch(token[1])
			{
			case '1': pDescr[num].type = SS_INT8;  sz+=1*pDescr[num].num; break;
			case '2': pDescr[num].type = SS_INT16;  sz+=2*pDescr[num].num; break;
			case '4': pDescr[num].type = SS_INT32;  sz+=4*pDescr[num].num; break;
			default: printf("Bad format on type integer: %s\n", token); free(frmcopy); return -1;
			}
			break;
		case 'b':
			pDescr[num].type = SS_BOOLEAN; sz+=1*pDescr[num].num; 
			break;
		default:
			printf("Bad format on unknown type: %s\n", token); free(frmcopy); return -1;
			break;
		}

      // Get next token: 
		num++;
      token = strtok( NULL, SEPS );
   }

	printf("Total Size = %d\n", sz);
	*size = sz;

	free(frmcopy);
	return num;
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
	char *formatrec = NULL;
	char *head = NULL;
	
	formatrec = (char *)calloc(mxGetN(FORMAT_REC_ARG)+1,sizeof(char));
	if (formatrec==NULL) 
	{
		printf("RS232 Write Binary (mdlStart.1) error: could not allocate memory\n");
		return;
	}
	
	mxGetString(FORMAT_REC_ARG, formatrec, mxGetN(FORMAT_REC_ARG)+1);
	ssSetPWorkValue(S,0,(void *)formatrec);

	T_DataDescr*	pDescr = NULL;
	int numForm = countFormatFields(formatrec);

	pDescr = (T_DataDescr*) malloc(numForm * sizeof(T_DataDescr));
	if (pDescr==NULL) 
	{
		printf("RS232 Write Binary (mdlStart.2) error: could not allocate memory\n");
		return;
	}
	
	int totSize = 0;
	int nErr = translateFormat(pDescr, formatrec, &totSize);
	if (nErr != numForm)
	{
		free(pDescr);
		sprintf(msg, "Error in RS232_Write_Binary: \n");
		sprintf(&msg[strlen(msg)], "Wrong format string.\n");
		return;
	}

	ssSetPWorkValue(S,1,(void *)pDescr);
	ssSetIWorkValue(S,0,numForm);
	ssSetIWorkValue(S,1,totSize);
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
	int		i_outStat = 0;

	Addr64 hPort = *u0;
	int i_inStat = (int)(*u1[0]);

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = i_outStat;

	// Check handle validity
	if (hPort.addr == 0)
	{
		printf("RS232 Write Binary error: choosen COM-port not initialized\n");
		return;
	}

	// If this condition is valid, the previous block (a read or setup block)
	// completed its work or this block, after activated, didn't received 
	// all the expected character. Then skip the actual operation.
	if (!i_inStat)
		return;

	// Retrieve the work values for pointers and integers
	char*				formatrec = (char *)ssGetPWorkValue(S,0);
	
	// If the total message is greater then the message terminator, look for the 
	// terminator
	int	totSize = ssGetIWorkValue(S,1);
	int	numForm = ssGetIWorkValue(S,0);
	T_DataDescr*	pDescr = (T_DataDescr*)ssGetPWorkValue(S,1);
	char*		c_Output = (char*)malloc(totSize);
	char*		c_P = c_Output;
	//printf("totSize = %d\n", (int)totSize);
	//printf("c_P = %x\n", (int)c_P);

	for (int n=0; n<numForm; n++)
	{
		void** u2	= (void**)ssGetInputPortSignalPtrs(S,2+n); 

		//printf("u2(%d) val: %x\n", n, *((int*)(*u2)));
		//printf("pDescr[n] val: type:%d, num:%d\n", pDescr[n].type, pDescr[n].num);

		c_P += writeSignal(u2, c_P, &pDescr[n]);

		//printf("c_P = %x\n", (int)c_P);
	}

	/*
	FILE* stream;

	if( (stream  = fopen( "clust.dat", "w+" )) == NULL )
	{
		printf( "The file 'data' was not opened\n" );
		return;
	}
	else
	{
		printf( "The file 'data' was opened\n" );
	}

	fwrite(c_Output, sizeof(char), totSize, stream);

	fclose(stream);
	*/
	RS232WriteL(hPort, c_Output, totSize);

	// The binary message has been read		
	i_outStat = 1;  // Read done

	// Set the outputs
	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = i_outStat;

	free(c_Output);
}

/*****************************************************************************
+FUNCTION: writeSignal
*******************************************************************************/
int	writeSignal(void** uin, char* buff, T_DataDescr* pDescr)
{
	switch(pDescr->type)
	{
	case SS_SINGLE:
	case SS_INT32:
	case SS_UINT32:
		{
			int* b = (int*)buff;
			int** u = (int**)uin;
			for (int n=0; n<pDescr->num; n++)
			{
				//printf ("*u[n] = %d\n", *u[n]);
				b[n] = *u[n];
			}
		}
		break;
	case SS_DOUBLE:
		{
			double* b = (double*)buff;
			double** u = (double**)uin;
			for (int n=0; n<pDescr->num; n++)
			{
				//printf ("*u[n] = %f\n", *u[n]);
				b[n] = *u[n];
			}
		}
		break;
	case SS_INT16:
	case SS_UINT16:
		{
			short* b = (short*)buff;
			short** u = (short**)uin;
			for (int n=0; n<pDescr->num; n++)
			{
				//printf ("*u[n] = %d\n", *u[n]);
				b[n] = *u[n];
			}
		}
		break;
	case SS_INT8:
	case SS_UINT8:
	case SS_BOOLEAN:
		{
			char* b = (char*)buff;
			char** u = (char**)uin;
			for (int n=0; n<pDescr->num; n++)
			{
				//printf ("*u[n] = %d\n", *u[n]);
				b[n] = *u[n];
			}
		}
		break;
	}

	return pDescr->num * typesize[pDescr->type];
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
	char *formatrec;
	T_DataDescr *pDescr;

	formatrec=(char *)ssGetPWorkValue(S,0);
	
	if (formatrec!=NULL) {
		free(formatrec);
	}
	
	pDescr=(T_DataDescr *)ssGetPWorkValue(S,1);
	
	if (pDescr!=NULL) {
		free(pDescr);
	}
}


#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
