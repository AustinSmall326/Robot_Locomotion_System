# Master Makefile to compile all C++ libraries.
# Variables
CWD                 = $(shell pwd)
SHM_DIR             = Utils/shm 
DCM_DIR             = LocomotionDev/Modules/Utils/fastGetSetDCM
REPRESENTATIONS_DIR = LocomotionDev/Modules/Representations 
KINEMATICS_DIR      = LocomotionDev/Modules/Kinematics
ITERP_DIR           = LocomotionDev/Modules/Utils/Iterp
STEPHANDLER_DIR     = LocomotionDev/Modules/StepHandler
KEYFRAMEGEN_DIR     = LocomotionDev/Modules/Utils/keyframeGen
TCPLIB_DIR          = TestingSuite/Utils/TCPLibrary
TCPCOMPUTERCOMM_DIR = TestingSuite/Utils/TCPComputerComm
TCPROBOTCOMM_DIR    = TestingSuite/Utils/TCPRobotComm

# Shared memory module (SHM).
shm:
	@cd $(SHM_DIR) && make clean && make all && cd $(CWD)

# DCM communication module (fastGetSetDCM).
dcm:
	@cd $(DCM_DIR) && make clean && make all && cd $(CWD)

# Step handler module.
stepHandler:
	@cd $(REPRESENTATIONS_DIR) && make clean && make all && cd $(CWD)
	@cd $(KINEMATICS_DIR) && make clean && make all && cd $(CWD)
	@cd $(ITERP_DIR) && make clean && make all && cd $(CWD)
	@cd $(STEPHANDLER_DIR) && make clean && make all && cd $(CWD)
	@cp LocomotionDev/Modules/LocomotionParameters.txt Libs

# Keyframe generator module.
keyframeGen:
	@cd $(REPRESENTATIONS_DIR) && make clean && make all && cd $(CWD)
	@cd $(KINEMATICS_DIR) && make clean && make all && cd $(CWD)
	@cd $(ITERP_DIR) && make clean && make all && cd $(CWD)
	@cd $(KEYFRAMEGEN_DIR) && make clean && make all && cd $(CWD)

# All testing related modules (including TCP libraries for robot, as well as cpp/mex files for computer).
testingRobot:
	@cd $(TCPLIB_DIR) && make clean && make all && cd $(CWD)
	@cd $(TCPROBOTCOMM_DIR) && make clean && make all && cd $(CWD)

testingComputer:
	@cd $(TCPLIB_DIR) && make clean && make all && cd $(CWD)
	@cd $(TCPCOMPUTERCOMM_DIR) && make clean && make all && cd $(CWD)
