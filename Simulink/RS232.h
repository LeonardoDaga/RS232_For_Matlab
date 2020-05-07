#include <BaseTsd.h>

//#ifdef __cplusplus
//    extern "C" {
//#endif

#define N_MAX_PORT 12

/* Baudrate value: 110, 300, 600, 1200, 2400, 4800, 9600, 14400,
38400, 56000, 115200, 128000, 256000 */
/* Bytesize value: 5, 6, 7, 8 */
/* Parity: 0=even, 1=Mark, 2=No parity, 3=Odd, 4=Space */
/* StopBits: 1:One Stop Bit, 2=Two Stop Bits, 0:1.5 stop bits */

typedef struct
{
	int lo;
	int hi;
} Handle64; 

union Addr64
{
	Handle64 addr64;
	void*	 addr;
	int		 iaddr[2];

	Addr64(void* pointer)
	{
		addr = pointer;
	}
	Addr64()
	{
		addr = 0;
	}
	Addr64(int iaddr_in[])
	{
		iaddr[0] = iaddr_in[0];
		iaddr[1] = iaddr_in[1];
	}
};

Addr64 RS232Open(char* c_filename);
Addr64 RS232OpenC(int	n_Port,
			int n_BaudRate,
			int	n_ByteSize,
			int	n_Parity,
			int n_StopBits);
int RS232Close(Addr64 h_RS232);
int RS232Write(Addr64 h_RS232,char* c_Message);
int RS232WriteL(Addr64 h_RS232,char* c_Message, int i_len);
int RS232InQueue(Addr64 h_RS232);
int RS232Read(Addr64 h_RS232,char* c_Message, int i_len);

typedef struct
{
	char*	c_Buffer;
	int		i_Size;
	int		i_Read;
	int		i_Fill;
} T_Buffer;

enum RS232_Type
{
	TYPE_NONE,
	TYPE_CHAR,
	TYPE_UCHAR,
	TYPE_WORD,
	TYPE_UWORD,
	TYPE_INT,
	TYPE_UINT,
	TYPE_FLOAT,
	TYPE_DOUBLE
};

//#ifdef __cplusplus
//    }
//#endif
