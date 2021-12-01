Simulink model created by Juan Manuel Jacinto Villegas based on the RS232 libraries made by Leonardo Daga
email:john87mjv@gmail.com

1.- The Simulink model called "EXT_RS232_r2019b_v1.mdl" has been created using Matlab r2019b (it also works with Matlab r2020b, tested)
and configured as "External mode", it is possible to generate an excutable program of the Simulink model (see point 4). 

2.- The external compiler used is Visual Studio Community 2019, is enough with installing the option "Development for desktop 
with c++" to get recognized by Matlab as compiler (Matlab command to check compiler: mex -setup).

3.- The "sfun_rttime.c" is a function that must be compiled (see the command inside for windows or linux) to be used with the Simulink model
to run the simulation in real time, it works for any Simulink model not only for the RS232 libraries. 

4.- To generate an excutable (.exe) of the Simulink model using the RS232 libraries, is necessary to modify in "Configuration Parameters" 
tap "Code Generation" then in "Custom Code", add the path location of the Simulink RS232 libraries in "Includes directories" and the 
RS232.lib in "Libraries". Then to compile the model use the command "slbuild('EXT_RS232_r2019b_v1')" l, remember that an external compiler
is required.

5.- The orange block inside the Simulink model "EXT_RS232_r2019b_v1.mdl", it has a callback to automatically RUN in "External mode" after 
pressing it (to stop the simulation press the stop boton in the toolbar), it means automatically open the executable program (.exe) in 
background mode and the Simulink model works just as display. This allows reducing the computational load of Simulink. 
Note: if something is modified in the Simulink model then it must be compiled (slbuild) to generate a new executable (to update it).
