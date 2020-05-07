# RS232_For_Matlab (aka RS232 Blockset)

<h1>Title: RS232 Blockset<h1>
Summary: This blockset allows Simulink to communicate with remote devices using RS232 interface
Description: RS232 Blockset is a block library that allows to a simulation model to communicate directly and in real time (using, for example, the RT Blockset also published in Matlab Central) with a remote device.
Actually, it's possible to use a simple communication protocol based on formatted string (in which simulation data is used to compose the message) using a ANSI C printf-like formatting standard. 
12 blocks are actually available:
Initialization:
<ul>
<li>RS232_Setup: Initialize and close the serial communication. The settings of the serial communication are selectable using the block panel</li>
Unbuffered Formatted Communication: 
<li>RS232_Read_Format: Reads from the serial port a formatted string (function analogue to the standard-C sscanf function).</li>
<li>RS232_Write_Format: writes to the serial port a formatted string (function analogue to the standard-C sprintf function). </li>
<li>RS232_Write_String: writes to the serial port the string indicated as block parameter.</li>
<li>RS232_Read_Fix_Format: Read fix formatted string from a buffer. This block has been generated specifically to read Flight Simulator's formatted data string.</li>
Buffered Formatted Communication:
<li>RS232_Create_Buffer: Initialize and allocate an input communication buffer.</li>
<li>RS232_Read_Buffer: Reads bytes from the serial port, add them to the input buffer and looks into the buffer for a formatted string (function analogue to the standard-C sscanf function), leaving into the buffer all the bytes that doesn't correspond to the searched string (except the ones that are before the found string). </li>
<li>RS232_Wait_Buffer_Synch: Looks for messages headers into the buffer and returns the "code" of first message in the buffer. Read data from the serial port and add it to the buffer, but doesn't remove any byte from it.</li>
Bynary Formatted Communication:
<li>RS232_Write_Binary: Write a binary formatted data to the serial output.</li>
Binary Buffered Formatted Communication:
<li>RS232_Read_Binary_Buffer: This block reads bytes from the serial port, add them to the input buffer and gets, starting at the start of the buffer, formatted data (the format is declared in the format string argument), leaving into the buffer all the bytes that exceeds the read operation. </li>
<li>RS232_Peek_Binary_Buffer: This block peeks bytes from the serial port, add them to the input buffer and gets, starting at the start of the buffer, formatted data (the format is declared in the format string argument), leaving unmodified the buffer (no bytes are removed from the buffer, eventually some data are added in the case that new data is in the input port). 
The interface of this function is the same respect to the RS232_Read_Binary_Buffer block, except that after the read is complete, the read portion of data is NOT removed from the buffer.</li>
<li>RS232_Synch_Binary_Buffer:  Looks for data header into the buffer and returns "1" at the output flag when the header is found.</li>

The library blocks allow to write a simple string, to read and write a formatted string, to buffer strings and read multiple string types, to read and write a fix formatted string and to read and write a binary formatted strings. 

<h2>Caution:</h2>
Please define the windows system variable "MatlabFolder" (in Windows Control Panel/System/Advanced System Settings/Environment Variables) with the Matlab folder that contains the Matlab bin folder.
