/*
 * File : RS232_Read.c

	The format string fr is structured as follows:
	%[type=d,i,u,b][width=8,4,2,1]%
	Others characters, out of the '%' symbols, are not considered but their
	presence is verified, otherwise the translations process stops.

	In the channel descriptor (the T_ChanDescr structure) the organization of 
	the output channels is stored.
	This organization is fixed into the translateFormat function.

	In the data description (the T_DataDescr strcuture) the organization of 
	the format string is reported.
	The  organization is fixed into the translateFormat function.
	
*/

#define S_FUNCTION_NAME  RS232_Read_Fix_Format
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

typedef	int				INT32;
typedef	short				INT16;
typedef  unsigned int	UINT32;
typedef  unsigned short	UINT16;
typedef  unsigned char	UINT8;


// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	2
#define FORMAT_REC_ARG        	ssGetSFcnParam(S,0)
#define SAMP_TIME_ARG				ssGetSFcnParam(S,1)
#define MAX_SIZE_STRING	16

#define STEP_ALLOC		256
#define TMPSIZE			128

#define NO_I_WORKS              	5
#define NO_P_WORKS              	2

static char_T msg[256];

static int typesize[11] = 
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
	-1, // DATA_STRING
	-1, // DATA_PERC
};

#define NUM_CH_TYPE	9

typedef enum
{
	DATA_DOUBLE,
	DATA_SINGLE,
	DATA_INT8,
	DATA_UINT8,
	DATA_INT16,
	DATA_UINT16,
	DATA_INT32,
	DATA_UINT32,
	DATA_BOOLEAN,
	DATA_STRING,
	DATA_PERC,
	DATA_UNKNOWN,
}e_Type;

typedef struct
{
	int		out_ch;
	e_Type	type;
	int		size;
	char		str[MAX_SIZE_STRING];
} T_DataDescr;

typedef struct
{
	int		ch_num;
	int		ch_size;
} T_ChanDescr;

static int extract(T_DataDescr*	pDescr,		// The format descriptor
						 char*			isi,			// Input string 
						 void*			data);		// output channels pointer?
static void ResizeBuffer(SimStruct *S, char** buffer, int i_Size);
static void ConvertSpecialChars(char* msg);
int translateFormat(T_DataDescr*	pDescr, char* fr, int* size);
int countFormatFields(char* fr);
int countChannels(char* fr);
char* checksynch(T_DataDescr*	pDescr,		// The format descriptor
						 int				numDescr,	// number of format descriptors
						 char *in);					// output channels pointer?

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
		sprintf(msg, "Error in RS232_Read_Format: \n");
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
		printf("Error in RS232_Write_Format: could not allocate memory\n");
		return;
	}

	mxGetString(FORMAT_REC_ARG, formatrec, mxGetN(FORMAT_REC_ARG)+1);
	
	// Input port
	if (!ssSetNumInputPorts(S, 2)) return;
	ssSetInputPortWidth(S, 0, 2); /* Width of input port 1 (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of input port 2 (index 1) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, SS_INT32); /* Type of input PORT 2 (index 0) */
	
	// Output port
	int	nChan = countChannels(formatrec);

	if (!ssSetNumOutputPorts(S, 2 + nChan)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port 1 (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port 2 (index 1) */
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */
	
	int	nForm = countFormatFields(formatrec);

	T_DataDescr*	pDescr = NULL;

	pDescr = (T_DataDescr*) malloc(nForm * sizeof(T_DataDescr));
	if (pDescr==NULL) 
	{
		printf("RS232 Read Binary Buffer (mdlStart.2) error: could not allocate memory\n");
		return;
	}

	int totSize = 0, i = 0;
	int nErr = translateFormat(pDescr, formatrec, &totSize);
	for (i=0; i<nForm; i++)
	{
		printf("mdlInitializeSizes.1 (%d) (type = %d) str = %s\n", 
			i, pDescr[i].type, (pDescr[i].str==NULL)?"NULL":pDescr[i].str);
	}

	nChan = 0;
	for (i=0; i<nForm; i++)
	{
		if ((pDescr[i].type >= DATA_DOUBLE)&&(pDescr[i].type <= DATA_BOOLEAN))
		{
			ssSetOutputPortWidth(S, 2 + nChan, 1); // Width of output port 2 + i 
			ssSetOutputPortDataType(S, 2 + nChan, pDescr[i].type); /* Type of output PORT 2 (index 0) */
			nChan++;
			printf("mdlInitializeSizes.2 (%d) (type = %d)\n", i, pDescr[i].type);
		}
	}

	ssSetNumSampleTimes(S, 0);
	
	ssSetNumIWork(S, NO_I_WORKS);   /* number of integer work vector elements*/
	ssSetNumPWork(S, NO_P_WORKS);   /* number of pointer work vector elements*/

	free(formatrec);
	free(pDescr);
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
	// Set the buffer size
	int szBuff = 0;
	int i_AllocSize = 0;
	int i_rdStat = 0;

	ssSetIWorkValue(S,0,szBuff); // Set the first element
	ssSetIWorkValue(S,1,i_AllocSize); // Set the actual allocated buffer size
	ssSetIWorkValue(S,2,i_rdStat); // Set the read status

	char *formatrec = NULL;
	char *buffer = NULL;
	
	formatrec = (char *)calloc(mxGetN(FORMAT_REC_ARG)+1,sizeof(char));
	if (formatrec==NULL) 
	{
		printf("RS232 Read Format error: could not allocate memory\n");
		return;
	}
	
	mxGetString(FORMAT_REC_ARG, formatrec, mxGetN(FORMAT_REC_ARG)+1);
	ConvertSpecialChars(formatrec);

	// Initialize a null buffer
	ssSetPWorkValue(S,0,(void *)buffer);
	
	T_DataDescr*	pDescr = NULL;
	int	numForm = countFormatFields(formatrec);


	pDescr = (T_DataDescr*) malloc(numForm * sizeof(T_DataDescr));
	if (pDescr==NULL) 
	{
		free (formatrec);
		printf("RS232 Read Binary Buffer (mdlStart.2) error: could not allocate memory\n");
		return;
	}
	
	//printf ("mdlStart.0: numForm = %d\n", numForm);
	int totSize = 0;
	int nErr = translateFormat(pDescr, formatrec, &totSize);
	if (nErr != numForm)
	{
		free (pDescr);
		free (formatrec);
		sprintf(msg, "Error in RS232_ReadB_Buffer: \n");
		sprintf(&msg[strlen(msg)], "Wrong format string.\n");
		return;
	}
	//printf ("mdlStart.1: numForm = %d\n", numForm);

	ssSetPWorkValue(S,1,(void *)pDescr);
	ssSetIWorkValue(S,3,numForm);
	ssSetIWorkValue(S,4,totSize);

	//printf ("mdlStart.2: numForm = %d\n", numForm);

	free (formatrec);
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
	int		i_len;
	int		szBuff;
	char		*buffer;
	int		i_inStat; // Status variable received via input
	int		i_outStat = 0, i_rdStat;
	int		i_totsize = 0;
	
	Addr64 hPort = *u0;
	i_inStat = (int)(*u1[0]);

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = i_outStat;

	// Check handle validity
	if (hPort.addr == 0)
	{
		printf("RS232 Read Format error: choosen COM-port not initialized\n");
		return;
	}

	// Get the previous internal read status
	i_rdStat = ssGetIWorkValue(S,2);

	// If this condition is valid, the previous block (a read or setup block)
	// completed its work or this block, after activated, didn't received 
	// all the expected character. Then skip the actual operation.
	if ((!i_inStat)&&(!i_rdStat))
		return;

	// Retrieve the work values for pointers and the actual buffer size
	buffer = (char *)ssGetPWorkValue(S,0);
	szBuff = ssGetIWorkValue(S,0);
	
	// Checking the actual length of the queue
	i_len = RS232InQueue(hPort);

	ResizeBuffer(S, &buffer, i_len + szBuff);

	if (buffer == NULL)
	{
		printf("RS232 Read Format error: error managing buffers\n");
		szBuff = 0;
		i_rdStat = 0;
		ssSetIWorkValue(S,2,i_rdStat);
		return;
	}

	if (i_len != 0)
	{
		RS232Read(hPort, &buffer[szBuff], i_len);
	}
	
	szBuff += i_len;
	buffer[szBuff]=0; // Null-terminate the buffer

	// Get the size of the fixed message
	i_totsize = ssGetIWorkValue(S,4);
	
	// Counter of descarded chars
	int	st = 0;

	// If the total message is greater then the message terminator, look for the 
	// terminator
	while (szBuff-st>=i_totsize) // Only one read is performed
	{
		T_DataDescr* pDescr =  (T_DataDescr *)ssGetPWorkValue(S,1);
		int numForm = ssGetIWorkValue(S,3);

		char* isi = checksynch(pDescr, numForm, &buffer[st]);
		printf("start buffer = %s\n", &buffer[st]);

		if (isi == NULL)
		{
			st++;
			i_outStat = 0;  // Read not done
			i_rdStat = 1;  // Still looking for a synch
			continue;
		}

		printf("isi = %s\n", isi);

		int nChan = 0;

		for (int i=1; i<numForm; i++)
		{
			if ((pDescr[i].type >= DATA_DOUBLE)&&(pDescr[i].type <= DATA_BOOLEAN))
			{
				void*	y	= (void*)ssGetOutputPortSignal(S,2+nChan);
				extract(&pDescr[i], isi, y);
				nChan ++;
			}

			isi += pDescr[i].size;
		}

		printf("buffer = %s\n", buffer);
		
		if (isi - buffer >= szBuff)
			printf("new isi = no data\n");
		else
			printf("new isi = %s\n", isi);

		// Calculate the remaining qty of chars and
		// update the szBuff: 
		szBuff = szBuff - (int)(isi - buffer);

		printf("szBuff = %d\n", szBuff);

		if (szBuff > 0)
		{
			memmove(buffer, // Start of the buffer
					  isi, // The end of last message
					  szBuff); // the remaining qty of chars
		}

		if (szBuff >= 0)
		{
			buffer[szBuff]=0; // Null-terminate the buffer
		}

		if (szBuff > 0)
			printf("new buffer = %s\n", buffer);
		else
			printf("new buffer = empty\n");

		i_outStat = 1;  // Read done
		i_rdStat = 0;  // Synch found

		break;
	}
	
	ssSetIWorkValue(S,2,i_rdStat);

	// Save the last szBuff
	ssSetIWorkValue(S,0,szBuff);
	ssSetPWorkValue(S,0,buffer);

	y0[0] = hPort.iaddr[0];
	y0[1] = hPort.iaddr[1];
	y1[0] = i_outStat;
}

/*****************************************************************************
+FUNCTION: ResizeBuffer
*******************************************************************************/
static void ResizeBuffer(SimStruct *S, char** buffer, int i_Size)
{
	int i_AllocSize = ssGetIWorkValue(S,1);

	if (i_Size >= i_AllocSize)
	{
		if (*buffer == NULL)
		{
			*buffer = (char*)malloc(STEP_ALLOC);
			i_AllocSize = STEP_ALLOC;
			printf("Allocated memory\n");

			if (*buffer == NULL)
			{
				printf("RS232 Read Format error: could not allocate memory\n");
				i_AllocSize = 0;
			}
		}
		else
		{
			int newSize = i_AllocSize;

			while (newSize <= i_Size)
			{
				newSize += STEP_ALLOC;
			}

			*buffer = (char*)realloc(*buffer, newSize);
			i_AllocSize = newSize;

			printf("Required size is %d\n", i_Size);
			printf("Allocated size is %d\n", i_AllocSize);
			if (*buffer == NULL)
			{
				printf("RS232 Read Format error: could not allocate memory\n");
				i_AllocSize = 0;
			}
		}
	}
	else if ((i_Size < i_AllocSize - STEP_ALLOC)&&(i_Size > 0))
	{
		*buffer = (char*)realloc(*buffer, i_AllocSize - STEP_ALLOC);
		i_AllocSize -= STEP_ALLOC;

		printf("Required size is %d\n", i_Size);
		printf("Reduced size is %d\n", i_AllocSize);

		if (*buffer == NULL)
		{
			printf("RS232 Read Format error: could not allocate memory\n");
			i_AllocSize = 0;
		}
	}
	else 
	{
		/*
		printf ("Space is enough: allocated (%d): %d, needed: %d\n", 
							(int)*buffer, 
							i_AllocSize, 
							i_Size);
							*/
	}

	ssSetIWorkValue(S,1,i_AllocSize);
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
	char *buffer=(char *)ssGetPWorkValue(S,0);
	
	if (buffer!=NULL) {
		free(buffer);
	}

	T_DataDescr*	pDescr =  (T_DataDescr *)ssGetPWorkValue(S,1);
	
	if (pDescr!=NULL) {
		free(pDescr);
	}
}

/*****************************************************************************
+FUNCTION: ConvertSpecialChars
*******************************************************************************/
static void ConvertSpecialChars(char* msg)
{
	int ic,oc; // Input and output counter

	ic = 0;
	oc = 0;

	while (msg[ic] != 0)
	{
		if (msg[ic] == '\\')
		{
			switch(msg[ic+1])
			{
			case 'n': msg[oc] = '\n'; oc+=1; ic +=2; break;
			case 'r': msg[oc] = '\r'; oc+=1; ic +=2; break;
			case 't': msg[oc] = '\t'; oc+=1; ic +=2; break;
			case '\\': msg[oc] = '\\'; oc+=1; ic +=2; break;
			default: 
				msg[oc] = msg[ic];
				msg[oc+1] = msg[ic+1];
				oc+=2; ic +=2;
				break;
			}
		}
		else
		{
			msg[oc] = msg[ic];
			oc+=1; ic +=1;
		}
	}
	msg[oc] = 0;
}

/*****************************************************************************
+FUNCTION: strstrn
Extension of the strstr function, look into the input string for the firsts n 
char of the strCharSet string. The return value is the same of the strstr
function.
*******************************************************************************/
char* strstrn(const char *string, const char *strCharSet, int n)
{
	char* searchstr = (char*)malloc(n+1);
	strncpy(searchstr, strCharSet, n);
	searchstr[n] = 0;
	char* res = (char*)strstr(string, searchstr);
	free(searchstr);
	return res;
}

/*****************************************************************************
+FUNCTION: checksynch
returns the second field pointer (the first field is the header, the last
is the terminator)
if it returns a null char, the synch is not good

	TODO: Check initially if the input buffer is large enough to contain the
	full data packet

*******************************************************************************/
char* checksynch(T_DataDescr*		pDescr,		// The format descriptor
						 int				numDescr,	// number of format descriptors
						 char*			in)					// output channels pointer?
{
	if (numDescr <= 0)
		return NULL;  // Invalid descriptor

	printf("checksynch.1 (numDescr=%d)\n", numDescr);
	printf("checksynch.2 (str=%s, size=%d, in=%s)\n", pDescr[0].str, pDescr[0].size, in);

	// First string in the format descriptor should be considered as the 
	// message header
	char* head = strstrn(in, pDescr[0].str, pDescr[0].size);

	if (head == NULL) 
		return NULL;  // Header not found

	//printf("checksynch.3 (head=%s)\n", head);

	char* isi = NULL; // Input string init pointer
	isi = head + pDescr[0].size;

	char* esi = isi; // Terminator init pointer

	// Check for the terminator char
	for (int i = 1; i < numDescr-1; i++)
	{
		esi += pDescr[i].size;
	}

	if (strncmp(esi, pDescr[numDescr-1].str, pDescr[numDescr-1].size) != 0)
		return NULL;	// Terminator not found

	return isi;
}

/*****************************************************************************
+FUNCTION: extract
*******************************************************************************/
static int extract(T_DataDescr*	pDescr,		// The format descriptor
						 char*			isi,			// Input string 
						 void*			data)			// output channels pointer?
{
	int	res = 0;
	char* c_Data = (char*)malloc(pDescr->size + 1);
	if (c_Data == NULL) 
		return -1;
	memcpy(c_Data, isi, pDescr->size);
	c_Data[pDescr->size] = 0;

	switch (pDescr->type)
	{
	case DATA_STRING:
		// Data string cannot be translated
		break;
	case DATA_INT32:
		{
			INT32* idata = (INT32*)data;
			*idata = (INT32)atoi(c_Data);
		}
		break;
	case DATA_INT16:
		{
			INT16* idata = (INT16*)data;
			*idata = (INT16)atoi(c_Data);
		}
		break;
	case DATA_INT8:
		{
			INT8* idata = (INT8*)data;
			*idata = (INT8)atoi(c_Data);
		}
		break;
	case DATA_UINT32:
		{
			UINT32* idata = (UINT32*)data;
			*idata = (UINT32)atoi(c_Data);
		}
		break;
	case DATA_UINT16:
		{
			UINT16* idata = (UINT16*)data;
			*idata = (UINT16)atoi(c_Data);
		}
		break;
	case DATA_UINT8:
		{
			UINT8* idata = (UINT8*)data;
			*idata = (UINT8)atoi(c_Data);
		}
		break;
	case DATA_DOUBLE:
		{
			double* fdata = (double*)data;
			*fdata = (double)atof(c_Data);
			printf ("extract.double = %lf\n", *fdata);
		}
		break;
	case DATA_SINGLE:
		{
			float* fdata = (float*)data;
			*fdata = (float)atof(c_Data);
			printf ("extract.single = %f\n", *fdata);
		}
		break;
	case DATA_BOOLEAN:
		{
			bool* bdata = (bool*)data;
			*bdata = (bool)(atoi(c_Data) != 0);
		}
		break;
	default:
		res = -1;
		break;
	}

	free (c_Data);

	return res;
}

/*****************************************************************************
+FUNCTION: translateFormat
*******************************************************************************/
int translateFormat(T_DataDescr*	pDescr, char* fr, int* size)
{
	int num = 0;
	int fsi; // format specifier token init
	int fse; // format specifier token end  
	bool found = false;
	char* pos = NULL; // in buffer token init 
	*size = 0;

	fsi = 0;
	fse = strlen(fr);

	int ch_size = 0;

   while(fsi < fse)
   {
		// Look for a token init
		pos = strchr(&fr[fsi], '%');

		// No token
		if (pos == 0)
		{
			// Add a data descriptor containing the string
			pDescr[num].out_ch = -1;
			pDescr[num].type = DATA_STRING;
			pDescr[num].size = strlen(&fr[fsi]);
			
			if (pDescr[num].size >= MAX_SIZE_STRING)
				pDescr[num].size = MAX_SIZE_STRING - 1;

			strncpy(pDescr[num].str, &fr[fsi], pDescr[num].size);
			pDescr[num].str[pDescr[num].size] = 0;

			*size += pDescr[num].size;
			num++;
			break; // Terminate the loop
		}

		// if token is the first char
		if ((fr[fsi] == '%')&&(fr[fsi+1] == '%'))
		{
			// Add a data descriptor containing a '%' character
			pDescr[num].out_ch = -1;
			pDescr[num].type = DATA_PERC;
			pDescr[num].size = 1;
			fsi+=2;
			num++;
		}
		else if (fr[fsi] == '%')
		{
			// look for the token end
			pos = strchr(&fr[fsi+1], '%');
			
			if (pos == 0)
			{
				// The format string is badly formatted
				printf("The format string contains an unclosed format token: %s\n", &fr[fsi]);
				break;
			}

			int	tke = pos - fr; // Position of the token end
			e_Type	type = DATA_UNKNOWN;

			if (fr[tke-2] == 'd')
			{
				type = (fr[tke-1] == '4')? DATA_SINGLE:
						 (fr[tke-1] == '8')? DATA_DOUBLE: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'i')
			{
				type = (fr[tke-1] == '4')? DATA_INT32:
						 (fr[tke-1] == '2')? DATA_INT16:
						 (fr[tke-1] == '1')? DATA_INT8: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'u')
			{
				type = (fr[tke-1] == '4')? DATA_UINT32:
						 (fr[tke-1] == '2')? DATA_UINT16:
						 (fr[tke-1] == '1')? DATA_UINT8: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'b')
			{
				type = DATA_BOOLEAN;
			}
			
			if (type == DATA_UNKNOWN)
			{
				// The format string is badly formatted
				printf("The format string contains an unknown format token: %s\n", &fr[fsi]);
				break;
			}

			pDescr[num].type = type;
			pDescr[num].out_ch = ch_size++;
			pDescr[num].size = atoi(&fr[fsi+1]);
			pDescr[num].str[0] = 0;

			printf("translateFormat.5 (type = %d, size = %d)\n", pDescr[num].type, 
																					pDescr[num].size);

			// Update the pointer
			fsi = pos - fr + 1;
		}
		else
		{
			// Store the string between formats
			
			// look for the next token start
			pos = strchr(&fr[fsi+1], '%');

			// Add a data descriptor containing the string
			pDescr[num].out_ch = -1;
			pDescr[num].type = DATA_STRING;
			pDescr[num].size = pos - &fr[fsi];
			
			if (pDescr[num].size >= MAX_SIZE_STRING)
				pDescr[num].size = MAX_SIZE_STRING - 1;

			strncpy(pDescr[num].str, &fr[fsi], pDescr[num].size);
			pDescr[num].str[pDescr[num].size] = 0;

			// Update the pointer
			fsi = pos - fr;
		}

		*size += pDescr[num].size;

		num++;
   }

	return num;
}

/*****************************************************************************
+FUNCTION: countFormatFields
Counts format fields and dimensions the T_ChanDescr structure.
*******************************************************************************/
int countFormatFields(char* fr)
{
	int num = 0;
	int fsi; // format specifier token init
	int fse; // format specifier token end  
	char* pos = NULL; // in buffer token init 

	fsi = 0;
	fse = strlen(fr);

   while(fsi < fse)
   {
		// if token is the first char
		if ((fsi < (fse-1))&&(fr[fsi] == '%')&&(fr[fsi+1] == '%'))
		{
			// There's a data descriptor containing a '%' character
			fsi+=2;
			num++;

			// This case is used as a separated case
		}
		else if ((fsi < (fse-1))&&(fr[fsi] == '%'))
		{
			// look for the token end
			pos = strchr(&fr[fsi+1], '%');
			
			if (pos == 0)
			{
				// The format string is badly formatted
				printf("The format string contains an unclosed format token: %s\n", &fr[fsi]);
				return 0;
			}

			int	tke = pos - fr; // Position of the token end
			e_Type	type = DATA_UNKNOWN;

			if (fr[tke-2] == 'd')
			{
				type = (fr[tke-1] == '4')? DATA_SINGLE:
						 (fr[tke-1] == '8')? DATA_DOUBLE: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'i')
			{
				type = (fr[tke-1] == '4')? DATA_INT32:
						 (fr[tke-1] == '2')? DATA_INT16:
						 (fr[tke-1] == '1')? DATA_INT8: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'u')
			{
				type = (fr[tke-1] == '4')? DATA_UINT32:
						 (fr[tke-1] == '2')? DATA_UINT16:
						 (fr[tke-1] == '1')? DATA_UINT8: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'b')
			{
				type = DATA_BOOLEAN;
			}
			
			if (type == DATA_UNKNOWN)
			{
				// The format string is badly formatted
				printf("The format string contains an unknown format token: %s\n", &fr[fsi]);
				break;
			}

			num++;

			// Update the pointer
			fsi = pos - fr + 1;
		}
		else if (fsi < fse) // The token is a simple string, at least longer than a byte
		{
			// look for the next token start
			pos = strchr(&fr[fsi+1], '%');

			if (pos == 0)
			{
				// next token start not found: this is the last string
				num++;
				break;
			}

			// Update the pointer
			fsi = pos - fr;

			// There is a string format field
			num++;
		}
   }

	printf("countFormatFields.8 (num=%d)\n", num);
	return num;
}

/*****************************************************************************
+FUNCTION: countChannels
Counts format fields and dimensions the T_ChanDescr structure.
*******************************************************************************/
int countChannels(char* fr)
{
	int fsi; // format specifier token init
	int fse; // format specifier token end  
	char* pos = NULL; // in buffer token init 

	fsi = 0;
	fse = strlen(fr);

	int ch_size = 0;

	// printf("countChannels.0\n");

   while(fsi < fse)
   {
		// printf("countChannels.1\n");

		// if token is the first char
		if ((fsi < (fse-1))&&(fr[fsi] == '%')&&(fr[fsi+1] == '%'))
		{
			// printf("countChannels.2\n");
			// There's a data descriptor containing a '%' character
			fsi+=2;
		}
		else if ((fsi < (fse-2))&&(fr[fsi] == '%'))
		{
			// printf("countChannels.3\n");
			// look for the token end
			pos = strchr(&fr[fsi+1], '%');
			
			if (pos == 0)
			{
				// The format string is badly formatted
				printf("The format string contains an unclosed format token: %s\n", &fr[fsi]);
				return 0;
			}

			int	tke = pos - fr; // Position of the token end
			e_Type	type = DATA_UNKNOWN;

			if (fr[tke-2] == 'd')
			{
				type = (fr[tke-1] == '4')? DATA_SINGLE:
						 (fr[tke-1] == '8')? DATA_DOUBLE: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'i')
			{
				type = (fr[tke-1] == '4')? DATA_INT32:
						 (fr[tke-1] == '2')? DATA_INT16:
						 (fr[tke-1] == '1')? DATA_INT8: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'u')
			{
				type = (fr[tke-1] == '4')? DATA_UINT32:
						 (fr[tke-1] == '2')? DATA_UINT16:
						 (fr[tke-1] == '1')? DATA_UINT8: DATA_UNKNOWN;
			}
			else if (fr[tke-2] == 'b')
			{
				type = DATA_BOOLEAN;
			}

			// printf("countChannels.5 (type = %d)\n", type);
			
			if (type == DATA_UNKNOWN)
			{
				// The format string is badly formatted
				printf("The format string contains an unknown format token: %s\n", &fr[fsi]);
				break;
			}

			ch_size++;
			// printf("countChannels.7 (ch_size=%d)\n", ch_size);

			// Update the pointer
			fsi = pos - fr + 1;
		}
		else if (fsi < fse)
		{
			// printf("countChannels.4\n");

			// look for the next token start
			pos = strchr(&fr[fsi+1], '%');

			if (pos == 0)
			{
				// next token start not found: this is the last string
				break;
			}

			// Update the pointer
			fsi = pos - fr;
		}
   }

	printf("countChannels.8 (ch_size=%d)\n", ch_size);
	return ch_size;
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
