/*******************************************************************************
   
Project Name:			SCIROCCO / VTIP

Module Name:         CSerial.h

Releases:
Issue    Date        Comment
-----    --------    --------------------------------------------------------
1.0.0		3Q-1998		First issue of the software

DESCRIPTION:
-----------
Include file for entire CSerial.cpp module

*******************************************************************************/
#ifndef _CSERIAL_CONNECTION_H
#define _CSERIAL_CONNECTION_H

#include "Globals.h"

struct S_SER_Config 
{
	int	i_Simulated;
	int	i_SerialPort;
	int	i_Baud_Rate;
	int	i_Data_Bits;
	int	i_Parity;
	int	i_StopBits;
	int	i_InQueue;
	int	i_OutQueue;
	int	i_Handshake_Type;
};

struct S_SER_Status
{
	int	i_Line_Number;
	int	i_Connection_Status;
	int	i_Trasmitting_Data;
	int	i_Receiving_Data;
};

class CSerial      
{ 
private:
	int						i_Alarm;
	char*						c_ConfigFile;

public:
	S_SER_Config			SER_Config;
	S_SER_Status			SER_Status;
	int						i_EventMask;
	HANDLE					hEvent;
	HANDLE					hComm;
	HANDLE					hThread;
	OVERLAPPED				o;
	HANDLE					hCommEvent;
	CRITICAL_SECTION		gcsDataEvent;
	CRITICAL_SECTION		gcsCommEvent;

public:
	// Constructor and destructor
	CSerial();
	CSerial(char* c_FormatString,...);
	~CSerial();

	// Configuration functions
	int	Save_Configuration();
	int	Load_Configuration();

	// CSerial communication functions
	int	Open(int	i_Configure);
	int	Write(	char* c_Message,...);
	int	WriteL(	char* c_Message, int i_Len);
	int	Read(char* c_MessageOut,int	i_MaxLenMessage);
	int	Verify(char* c_Message,	...);
	int	Close();
	int	WriteABuffer(char * c_message, DWORD dwToWrite);
	int	InQueue();
	int SetEventChar(char c);

	// Callback functions
	int InstallCallback(int i_EventMask = 1, int i_ByteNotify = 1,
										void* callbackdata = NULL);
	int CheckForCommEvent(int i_EventMask = 1, int i_ByteNotify = 1);
};

//////////////////////////////////
// Error management
//////////////////////////////////
typedef enum {
	SER_NOERROR,
	SER_FAILED_TO_CONN,
	SER_ERRFAILED,
	SER_READFAILED,
	SER_WRITEFAILED,
	SER_INVALIDPARAM,
	SER_TIMEOUTERR,
	SER_CONN_CLOSED,
	SER_NOVERIFY,
	SER_ALREADYOPEN,
	SER_ALARM_TIMEOUT,
	SER_LAST_ERROR
}SER_ErrorType;

typedef enum {
	SER_HANDSHAKE_NONE,
	SER_HANDSHAKE_CTS,
	SER_HANDSHAKE_XON_XOFF
}SER_ConnType;


#endif