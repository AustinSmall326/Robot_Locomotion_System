About:

	The shm module is an independent module loaded into NAOQI that provides shared
 	memory for transferring joint angles and joint stiffness values between the 
	stepHandler module and the fastGetSetDCM module.

Compilation:

	In order to compile the shm module, copy the shm folder onto a directory in virtual 
	box and run make all.  Copy the resulting .so shared library file onto a directory on 
	the Nao robot, 	and include the path to this .so file in the 
	/home/nao/naoqi/preferences/autoload.ini file, 	under [USER] defined library files.  
	Upon starting NAOQI, this module will automatically load.

