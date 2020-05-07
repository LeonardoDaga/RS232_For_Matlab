/*
 * File : RS232_Read.c
 */

#define S_FUNCTION_NAME  RS232_Read_Format
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"
#include "RS232.h"

// INPUT ARGUMENTS
#define NUMBER_OF_ARGS	3
#define FORMAT_REC_ARG        	ssGetSFcnParam(S,0)
#define NUM_CH_TERM					(int)(*mxGetPr(ssGetSFcnParam(S,1)))
#define SAMP_TIME_ARG				ssGetSFcnParam(S,2)

#define STEP_ALLOC		256
#define TMPSIZE			128

#define NO_I_WORKS              	3
#define NO_P_WORKS              	3

static char_T msg[256];

static int extract(char *formatstring, char* head, char *in, double *data);
static int datacount(char *formatstring);
static void ResizeBuffer(SimStruct *S, char** buffer, int i_Size);
static void ConvertSpecialChars(char* msg);

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
		sprintf(msg, "Error in RS232_Read_Format: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif
	
	buffer=(char *)calloc(mxGetN(FORMAT_REC_ARG)+1,sizeof(char));
	if (buffer==NULL) 
	{
		printf("Error in RS232_Write_Format: could not allocate memory\n");
		return;
	}

	mxGetString(FORMAT_REC_ARG, buffer, mxGetN(FORMAT_REC_ARG)+1);
	
	// Input port
	if (!ssSetNumInputPorts(S, 2)) return;
	ssSetInputPortWidth(S, 0, 2); /* Width of input port 1 (index 0) */
	ssSetInputPortWidth(S, 1, 1); /* Width of input port 2 (index 1) */
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDataType(S, 0, SS_INT32); /* Type of input PORT 1 (index 0) */
	ssSetInputPortDataType(S, 1, SS_INT32); /* Type of input PORT 2 (index 0) */
	
	// Output port
	if (!ssSetNumOutputPorts(S, 3)) return;
	ssSetOutputPortWidth(S, 0, 2); /* Width of output port 1 (index 0) */
	ssSetOutputPortWidth(S, 1, 1); /* Width of output port 2 (index 1) */
	ssSetOutputPortWidth(S, 2, datacount(buffer)); // Width of output port 3 (index 2) 
	ssSetOutputPortDataType(S, 0, SS_INT32); /* Type of output PORT 1 (index 0) */
	ssSetOutputPortDataType(S, 1, SS_INT32); /* Type of output PORT 2 (index 0) */
	
	ssSetNumSampleTimes(S, 0);
	
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
	ssSetIWorkValue(S,0,szBuff); // Set the first element
	ssSetIWorkValue(S,1,i_AllocSize); // Set the actual allocated buffer size

	int i_rdStat = 0;
	ssSetIWorkValue(S,2,i_rdStat); // Set the first element

	char *formatrec = NULL;
	char *buffer = NULL;
	char *head = NULL;
	
	formatrec = (char *)calloc(mxGetN(FORMAT_REC_ARG)+1,sizeof(char));
	if (formatrec==NULL) 
	{
		printf("RS232 Read Format error: could not allocate memory\n");
		return;
	}
	
	mxGetString(FORMAT_REC_ARG, formatrec, mxGetN(FORMAT_REC_ARG)+1);
	ConvertSpecialChars(formatrec);
	ssSetPWorkValue(S,0,(void *)formatrec);

	// Initialize a null buffer
	ssSetPWorkValue(S,1,(void *)buffer);
	
	// find the message sentence header. The message header is all that preceeds
	// the first "%" character
	char* head_end = strchr(formatrec, '%');
	head = (char*)calloc(head_end - formatrec + 1,sizeof(char)); // 1 is for null char

	if (head == NULL)
	{
		printf("RS232 Read Format error: could not allocate memory (%d bytes) for header\n",
			    head_end - formatrec);
		return;
	}
	
	memcpy(head, formatrec, head_end - formatrec);
	head[head_end - formatrec] = 0;  // null terminate the header

	// Save the reference for the head buffer
	ssSetPWorkValue(S,2,(void *)head);
	
	//printf("Head=%s\n", head);
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
	real_T	*y2	= (real_T*)ssGetOutputPortSignal(S,2);
	int		**u0	= (int**)ssGetInputPortSignalPtrs(S,0); 
	int		**u1	= (int**)ssGetInputPortSignalPtrs(S,1); 
	int		n_Term;
	int		i_len;
	int		szBuff;
	char		*buffer;
	char		*formatrec;
	char		*head;
	int		i_inStat; // Status variable received via input
	int		i_outStat = 0, i_rdStat;
	
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
	formatrec = (char *)ssGetPWorkValue(S,0);
	buffer = (char *)ssGetPWorkValue(S,1);
	head = (char *)ssGetPWorkValue(S,2);
	szBuff = ssGetIWorkValue(S,0);
	
	n_Term = NUM_CH_TERM;

	// Checking the actual length of the queue
	i_len = RS232InQueue(hPort);

	// if ((i_len + szBuff) == 0)
	//	return;

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
		/*
		char msg[64];
		memcpy(msg,buffer,63);
		msg[63] = 0;
		printf("Old buffer = %s\n",msg);
		*/
		
		RS232Read(hPort, &buffer[szBuff], i_len);
		
		/*
		memcpy(msg,buffer,63);
		msg[63] = 0;
		printf("New buffer = %s\n",msg);
		*/
	}
	
	szBuff += i_len;
	buffer[szBuff]=0; // Null-terminate the buffer

	/*
	if (i_len != 0)
		printf("New msg [%d,%d]: %s\n", szBuff, i_len, &buffer[szBuff-i_len]);
		*/


	// If the total message is greater then the message terminator, look for the 
	// terminator
	if (szBuff>=n_Term) 
	{
		// Look for the message sentence terminator
		char *msgTerm= &formatrec[strlen(formatrec)-n_Term];
		char *msgEnd = strstr(buffer,msgTerm);
		char *msgStart = strstr(buffer, head);

		if (msgStart > buffer)
		{
			printf("Some characters (%d) rejected\n", msgStart - buffer);
		}

		if ((msgEnd != 0)&&(msgStart != 0))
		{
			//printf("Terminator found: (%d)\n", (int)(msgEnd - buffer));
			
			// if a message terminator has been found
			/*
			char msg[64];
			memcpy(msg,buffer,63);
			msg[63] = 0;
			printf("buffer = %s\n",msg);
			printf("Header found: (%d)\n", (int)(msgStart - buffer));
			*/

			if (msgStart < msgEnd)
				extract(formatrec,head,buffer,(double *)y2 );

			// Calculate the remaining qty of chars and
			// update the szBuff: 
			szBuff = szBuff - (int)(msgEnd - buffer) - n_Term;

			memmove(buffer, // Start of the buffer
					  msgEnd + n_Term, // The end of last message
					  szBuff); // the remaining qty of chars

			buffer[szBuff]=0; // Null-terminate the buffer

			// if a message terminator has been found
			/*
			memcpy(msg,buffer,63);
			msg[63] = 0;
			printf("new buffer (%d): %s\n", szBuff, msg);
			*/
			i_outStat = 1;  // Read done
			//printf("readf: i_outStat = %d, %d, |%s| \n", hPort, i_outStat, buffer);
			i_rdStat = 0;  // End looking for a synch
			ssSetIWorkValue(S,2,i_rdStat);
		}
		else if ((msgEnd != 0)&&(msgStart == 0))
		{
			//printf("Terminator found without an head: (%d)\n", (int)(msgEnd - buffer));

			// Calculate the remaining qty of chars and
			// update the szBuff: 
			szBuff = szBuff - (int)(msgEnd - buffer) - n_Term;

			memmove(buffer, // Start of the buffer
					  msgEnd + n_Term, // The end of last message
					  szBuff); // the remaining qty of chars

			buffer[szBuff]=0; // Null-terminate the buffer

			i_outStat = 1;  // Read done
			//printf("readf: i_outStat = %d, %d, |%s| \n", hPort, i_outStat, buffer);
			i_rdStat = 0;  // End looking for a synch
			ssSetIWorkValue(S,2,i_rdStat);
		}
		else
		{
			/*
			if (i_len != 0)
			{
				printf("msgEnd: %d\n", msgEnd - buffer);
				printf("msgStart: %d\n", msgStart - buffer);

				char msg[64];
				memcpy(msg,buffer,63);
				msg[63] = 0;
				printf("buffer = %s\n",msg);
			}
			*/
			i_outStat = 0;  // Read not done
			
			//printf("readf: i_outStat = %d, %d, |%s| \n", hPort, i_outStat, buffer);
			i_rdStat = 1;  // Still looking for a synch
			ssSetIWorkValue(S,2,i_rdStat);
		}
	}
	else
	{
		i_outStat = 0;  // Read not done
		//printf("readf: i_outStat = %d, %d, |%s| \n", hPort, i_outStat, buffer);
		i_rdStat = 1;  // Still looking for a synch
		ssSetIWorkValue(S,2,i_rdStat);
	}
	
	// Save the last szBuff
	ssSetIWorkValue(S,0,szBuff);
	ssSetPWorkValue(S,1,buffer);

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
	char *formatrec;
	char *buffer;
	char *head;

	formatrec=(char *)ssGetPWorkValue(S,0);
	
	if (formatrec!=NULL) {
		free(formatrec);
	}
	
	buffer=(char *)ssGetPWorkValue(S,1);
	
	if (buffer!=NULL) {
		free(buffer);
	}

	head=(char *)ssGetPWorkValue(S,2);
	
	if (head!=NULL) {
		free(head);
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
	for (i=0; i<strlen(formatstring); i++) 
	{
		if (formatstring[i]=='%') n++;
	}
	
	return(n);
}


/*****************************************************************************
+FUNCTION: extract

+DESCRIPTION: When this function is called, the sentence terminator has been
 detected. then, if there isn't no sentence header, the message cannot be 
 translated. In this case, the calling function is in rule of discarding 
 the part of the message preceeding and containing the sentence terminator
*******************************************************************************/
static int extract(char *formatstring, char* head, char *in, double *data)
{
	int i;
	int fsi; // format specifier token init
	int ini; // in buffer token init 
	int fse; // format specifier token end  
	int found;
	int ine; // in buffer token end
	char token[TMPSIZE];
	char endchar;
	char substr[TMPSIZE];
	int type;
	int tmpi;
	long int tmpli;
	float tmpf;
	double tmpd;
	
	fsi=0;
	ini=fsi;
	
	i=0;
	
	// look for the header
	char* head_start = strstr(in, head);

	// check if found
	if(head_start == 0)
	{
		printf("RS232 Read Format: header not found\nMsg = %s\nHead = %s",
			    in, head);
		return -1;
	}


	fsi = 0; // head_start - in;
	ini = head_start - in; // fsi;

	//printf("Init: fsi=%d, ini=%d\n", fsi, ini);

	// fsi indicate the position of the format token start 
	while (fsi<strlen(formatstring)) 
	{
		if (formatstring[fsi]=='%') 
		{
			found=0; // search format token
			fse=fsi; // fse indicate the position of the format token end
	
			// look for the format token end
			while (!found) 
			{
				switch (formatstring[fse]) 
				{
         	case 'e':
         	case 'f':
         	case 'g':
         	case 'E':
         	case 'G':
					if (formatstring[fse-1] == 'l')
						type=3; 
					else
						type=1; 
					found=1;
					break;
				case 'd':
				case 'x':
				case 'o':
				case 'i':
					if (formatstring[fse-1] == 'l')
						type=4; 
					else
						type=2; 
					found=1;
					break;
				default:
					fse++;
					if (fse>=strlen(formatstring)) 
					{
						printf("RS232 Read Format: error in format string");
						return -1;
					}
					break;
				}
			} // end while (!found)
			
			// token found
			// save it
			strncpy(token,formatstring+fsi,fse-fsi+1);
			token[fse-fsi+1]='\0';
			
			// endchar: is the first char after the last format token
			endchar=formatstring[fse+1];

			// search endchar inside the buffer
			char* in_endchar = strchr(&in[ini], endchar);
			if (in_endchar == NULL)
			{
				printf("RS232 Read Format: error in lookin'for endchar str=(%s), endc=%c\n",
						 &in[ini],
						 endchar);
				return -1;
			}

			found = 1;
			ine = in_endchar - in;
				
			// save the part of the string relative to the token in a 
			// temporary substring
			strncpy(substr,in+ini,ine-ini);
			substr[ine-ini]='\0';
			int nscan = 0;
			
			if (type==1) 
			{
				nscan = sscanf(substr,token,&tmpf);
				if (nscan != 1)
				{
					printf("RS232 Read Format: error in translating received string (type=1)\n");
					return -1;
				}

				//printf("%f\n",tmpf);
				data[i]=(double)tmpf;
			} 
			else if (type==2) 
			{
				nscan = sscanf(substr,token,&tmpi);
				if (nscan != 1)
				{
					printf("RS232 Read Format: error in translating received string (type=2)\n");
					return -1;
				}

				//printf("substr = %s\ntoken = %s\n", substr,token);

				data[i]=(double)tmpi;
			}      								   
			else if (type==3) 
			{
				nscan = sscanf(substr,token,&tmpd);
				if (nscan != 1)
				{
					printf("RS232 Read Format: error in translating received string (type=3)\n");
					return -1;
				}


				data[i]=tmpd;
			}      								   
			else if (type==4) 
			{
				nscan = sscanf(substr,token,&tmpli);
				if (nscan != 1)
				{
					printf("RS232 Read Format: error in translating received string (type=4)\n");
					return -1;
				}


				data[i]=(double)tmpli;
			}      								   
			
			// printf("nscan=%d\n", nscan);

			// set new ini and fsi
			fsi=fse;
			ini=ine-1;   
			i++;

			// Increment the counter
			fsi++;	// Format string counter
			ini++;	// Input buffer counter
		} // end if (formatstring[fsi]=='%')
		else if (formatstring[fsi]=='\\')
		{
			fsi+=2;	// Format string counter
			ini++;	// Input buffer counter
		}
		else 
		{
			if (formatstring[fsi]==in[ini])
			{
				fsi++;	// Format string counter
				ini++;	// Input buffer counter
			}
			else
			{
				printf("Input string doesn't correspond to format string: Error in In[%d]:%c, FormatStr[%d]:%c",
							ini, in[ini], fsi, formatstring[fsi]);
				printf("\n");
				fsi++;	// Format string counter
				ini++;	// Input buffer counter
			}
		}
		
	}
	
	return 0;
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

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
