//
//	Interface to LoggingServices
//

#ifdef __cplusplus
    extern "C" {
#endif

void BeginLogging(char *c_LoggingFileName);

void SetTimeDelta(int i_Delta);

void TraceLog(char *c_Module,
			  int  i_Line,
			  char *c_Function,
			  char *c_Message,
			  ...);

void Dump(unsigned char *c_Data,
		  int           i_Size);

void EndLogging();

#ifdef __cplusplus
    }
#endif
