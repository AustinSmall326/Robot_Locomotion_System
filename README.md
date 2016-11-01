About Code Base:
------------------

This project encompasses a full robotics platform (Locomotion, Vision, Localization, and Behavior), designed  
and built by the University of Pennsylvania's Robot Soccer Team (UPennalizers).  Using this code base, the team  
regularly competes in robotic soccer tournaments including the annual RoboCup competition (www.robocup.org).

The code contained in this repository represents our team's efforts to re-develop the code base in order to  
improve modularity, documentation, and the implementation.  Currently, emphasis has been placed on re-developing  
the locomotion system.  To date, the system includes a fully functioning open-loop omni-directional gait.

Future work will include continued improvements to the locomotion system as well as migration / development of  
the vision, localization, and behavior systems.  

Contact Information:
--------------------

Primary Contributors:	Austin Small and Alex Baucom  
Email:					ausmall@seas.upenn.edu  
Website:				www.AustinSmall.com  

UPenn Team Email:		upennalizers@gmail.com  
UPenn Team Website:		http://fling.seas.upenn.edu/~robocup/wiki  

Setup To Run Code Base:
-------------------------------

- You will need to install Matlab.  

- Make sure that you have aliased the mex command.  This can be done by adding the following line to  
your ~/.bashrc file: 
   ```
   alias mex = “path to mex executable”  
   ```

- You will need to have installed and set up VirtualBox.  

   **Introduction:**

   VirtualBox is an Oracle software package that allows you to run an instance of an operating system within your  
   currently running OS.  We use VirtualBox because the Nao robot does not have any compilers.  Instead, we  
   simulate the hardware/software configuration of the Nao through VirtualBox, where we write and compile our code  
   before transferring it to the robot.

   **Downloading/Installing VirtualBox:**

   VirtualBox can be downloaded from the following URL:

   https://www.virtualbox.org/wiki/Downloads

   After downloading the correct package for your OS, please following Oracle’s instructions for installing the software.

   Alternative option for Ubuntu:

   You can use apt-get to install VirtualBox via the terminal:
   ```
   sudo apt-get install virtualbox-qt
   ```
   If this installs correctly you should be able to start virtualbox with the terminal command:
   ```
   virtualbox
   ```
   You may see warnings beginning with:
   ```
   Qt WARNING: void DBusMenuExporterPrivate::addAction(QAction*, int): Already tracking action [rest of error message]
   ```
   These are fine to ignore (according to some googling).

   **Configuring VirtualBox:**

   As stated before, we use VirtualBox so that we can write and compile code that can be copied directly to the Nao  
   robot.  In order to do this, we must configure VirtualBox to run an OS identical to that on the Nao.

   ⁃	Ask a team member for access to the VirtualBox disk images.  Store those images on your computer.  
   ⁃	Start VirtualBox.  
   ⁃	Choose File -> Import Appliance  
   ⁃	Browse and open the *.ova file that you just downloaded.  
   ⁃	Click the Import button to start the importation.  

   **Using VirtualBox/Further Configuration:**
   
   ⁃	In VirtualBox, select the NAOqi OS virtual machine that you just imported, and click the Start button.  
   ⁃	You will be prompted to login and should use the following username and password:  
   ```
   Username:	nao  
   Password:	nao  
   ```
   Note that you can also ssh into the virtual machine as follows:  
   ```
   ssh -p 2222 nao@localhost  
   ```
   ⁃	There is one final step before you’re ready to start coding.  Download and uncompress  
        naoqi-sdk-2.1.4.13-linux32.tar.gz.  You may need to ask a team member for access to this file.  
   ⁃	Copy this folder to /home/nao on your NAOqi OS.  
   ⁃	You’re done.  You can now use this newly installed OS to create and compile c++ code for the Nao!  

- You will need to install boost on your computer.

   First download boost from the official website:  http://www.boost.org
   
   Issue the following commands in the shell:  
   ```
   cd path/to/boost  
   ./bootstrap.sh  
   ```

   Make sure that the following directories exist: 
   ``` 
   /usr/local/lib  
   /usr/local/include  
   ```
   Finally,  
   ```
   $ sudo ./b2 install  
   ```
   You may now remove the downloaded boost folder.  

Project Directory Organization 
---------------------------------------------------------------

![code base hierarchy](https://cloud.githubusercontent.com/assets/9031637/19909743/68c5ba34-a05f-11e6-9082-11a70ec33702.png)

Source Directory Organization (Alphabetical Order)
---------------------------------------------------------------

**Libs:**

This folder is the destination for all compiled .so files.  The folder should be copied over  
to the root directory on the robot.

Since the compiled .so files will be loaded into NaoQi, you will have to double check the  
~/naoqi/preferences/autoload.ini file on the robot.  Make sure that the path to all .so  
files in the Libs folder are included under [user] in this file.

**LocalizationDev:**

This folder contains Localization related code.

**LocomotionDev:**

This folder contains Locomotion related code.

**Makefile:**

Commands to this makefile can compile any and all components of the code base.

**TestingSuite:**

Any testing related code that is run from the computer is stored here.  More specifically,  
this folder stores Matlab code that enables for unit testing code that is run on the robot  
from a computer GUI interface.

**Utils:**

This folder stores any code that is common to different components of the code base.

**VisionDev:**

This folder contains Vision related code.

Module Overview
----------------------

**shm (.so):**

Location:	~/Utils/shm  
Purpose:   	This module, which operates as an independent module in the NaoQi, allows for storing   
data to be commonly accessed from different applications.  
Compilation: 	make shm  
Note:  This module is to be compiled in VirtualBox.  Copy UPennRSDev to VirtualBox and continue  
compilation there.

**dcm (.so):**

Location:		~/LocomotionDev/Modules/Utils/fastGetSetDCM  
Purpose:		This module, which operates as an independent module in the NaoQi, updates actuator  
position and stiffness every 10 ms.  It reads values from shared memory (the shm module)  
and updates the motors with these values.  
Compilation:	make dcm  
Note:  This module is to be compiled in VirtualBox.  Copy UPennRSDev to VirtualBox and continue  
compilation there.

**stepHandler (.so):**

Location:		~/LocomotionDev/Modules/StepHandler  
Purpose:		This module, which operates as an independent module in the NaoQi, is the highest level  
abstraction of locomotion.  It can perform high level commands such as kicking, standing up,  
and walking.  
Compilation:	make stepHandler  
Note:  This module is to be compiled in VirtualBox.  Copy UPennRSDev to VirtualBox and continue  
compilation there.  

**keyframeGen (.so):**

Location:		~/LocomotionDev/Modules/Utils/keyframeGen  
Purpose:		This module, which operates as an independent module in the NaoQi, assists with the Matlab  
key frame generator GUI that runs as part of the testing suite.  
Compilation:	make keyframeGen  
Note:  This module is to be compiled in VirtualBox.  Copy UPennRSDev to VirtualBox and continue  
compilation there.  

**testingRobot (.so):**

Location:		~/TestingSuite/Utils/  
Purpose:		This module, which operates as an independent module in the NaoQi, handles TCP communication  
between the robot and computer, facilitating testing.  
Compilation:	make testingRobot  
Note:  This module is to be compiled in VirtualBox.  Copy UPennRSDev to VirtualBox and continue  
compilation there.  

**testingComputer:**

Location:		~/TestingSuite/Utils  
Purpose:		This code includes TCP communication code (for the computer) as well as c++ mex code to  
support the Matlab testing suite.  
Compilation:	make testingComputer  
You will also need to run the following command from ~/TestingSuite/Utils/TCPComputerComm:  
```
mex TCPComm.cpp -I/usr/local/include -L/usr/local/lib -lboost_thread -lboost_system TCPHeartbeatWrapper.o ../TCPLibrary/TCPStream.o ../TCPLibrary/TCPConnector.o ../TCPLibrary/TCPAcceptor.o
```
